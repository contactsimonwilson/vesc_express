/*
    Copyright 2022‑2026 Benjamin Vedder     <benjamin@vedder.se>
    Copyright 2022      Joel Svensson      <svenssonjoel@yahoo.se>

    This file is part of the VESC firmware and is released under the
    terms of the GNU General Public License, version 3 (or any later).

    -----------------------------------------------------------------
    FreeRTOS / ESP‑IDF PORT
    -----------------------------------------------------------------
    All ChibiOS threading primitives have been replaced with their
    FreeRTOS counterparts so that `(spawn …)` and related Lisp helpers
    work unmodified on ESP32 targets.

    This implements the VESC Express native lib interface (see
    c_libs/vesc_c_if.h) - a fresh interface separate from the bldc
    one, starting from the platform-neutral core: LispBM access,
    threads, timing, mutexes/semaphores, memory and printf. New slots
    are appended to the struct; VESC_C_IF_VERSION is only bumped on
    breaking layout changes.
*/

#include "lbm_defines.h"
#pragma GCC optimize("Os")
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "esp_timer.h"
#include "esp_rom_sys.h"
#include "esp_heap_caps.h"
#include "esp_memory_utils.h"
#include "heap_memory_layout.h"
#include "driver/gpio.h"
#include "commands.h"
#include "comm_can.h"
#include "comm_wifi.h"
#include "conf_custom.h"
#include "flash_helper.h"
#include "imu.h"
#include "ahrs.h"
#include "nmea.h"
#include "bms.h"
#include "comm_mqtt.h"
#include "comm_espnow.h"
#include "comm_ble.h"
#include "custom_ble.h"
#include "conf_general.h"
#include "esp_mac.h"
#include "buffer.h"
#include "extensions.h"
#include "lbm_flat_value.h"
#include "lispif.h"
#include "lispif_rgbled_extensions.h"
#include "lispbm.h"
#include "utils.h"
#include "c_libs/vesc_c_if.h"
#include "freertos/task.h"

_Static_assert(sizeof(vesc_c_if) <= 2048, "cif pad too small");

typedef struct {
	const char *name;
	void *arg;
	void (*func)(void*);
	volatile bool should_terminate;
	TaskHandle_t handle;
	UBaseType_t base_prio;
} lib_thd_info;

#define LIB_MAX_THREADS 20
static lib_thd_info *lib_thread_infos[LIB_MAX_THREADS] = {0};
static size_t lib_thread_infos_cnt = 0;

// Optional: protect lib_thread_infos[] edits/reads if accessed from multiple tasks
static portMUX_TYPE lib_thr_mux = portMUX_INITIALIZER_UNLOCKED;
#define LIB_THR_LOCK()   portENTER_CRITICAL(&lib_thr_mux)
#define LIB_THR_UNLOCK() portEXIT_CRITICAL(&lib_thr_mux)

#define LIB_NUM_MAX 10

static lib_info loaded_libs[LIB_NUM_MAX] = {0};

// The flash (IROM) address of each loaded lib's container, i.e. the value
// the lisp code passes to load-native-lib / unload-native-lib. For XIP libs
// this equals base_addr; for RAM-loaded (relocated) libs it differs.
static uint32_t lib_flash_addr[LIB_NUM_MAX] = {0};

// Heap allocation backing a RAM-loaded lib, NULL for XIP libs.
static void *lib_ram_alloc[LIB_NUM_MAX] = {0};

// Second allocation backing a RAM-loaded lib's data region (S3), NULL
// otherwise.
static void *lib_ram_data[LIB_NUM_MAX] = {0};

__attribute__((section(".libif"))) static volatile union {
	vesc_c_if cif;
	char pad[2048];
} cif;

// The .libif section is placed at a fixed, target-specific address by
// main/linker_libif_<target>.ld so that native libs can find the interface
// table through the VESC_IF macro. Keep the heap allocator away from it.
SOC_RESERVE_MEMORY_REGION((intptr_t)&cif, (intptr_t)&cif + sizeof(cif), vesc_libif);

static bool lib_init_done = false;

static bool lib_is_func_valid(void *func) {
	return esp_ptr_executable(func);
}

static void lib_sleep_ms(uint32_t ms) {
	vTaskDelay(pdMS_TO_TICKS(ms));
}

static void lib_sleep_us(uint32_t us) {
	if (us >= 1000) {
		vTaskDelay(pdMS_TO_TICKS(us / 1000));
		us %= 1000;
	}
	if (us)
		esp_rom_delay_us(us);
}

static float lib_system_time(void) {
	return UTILS_AGE_S(0);
}

static float lib_ts_to_age_s(TickType_t ts) {
	return UTILS_AGE_S(ts);
}

static void lib_thd(void *arg) {
	lib_thd_info *t = (lib_thd_info*)arg;

	// Set thread-local storage for should_terminate check
	vTaskSetThreadLocalStoragePointer(NULL, 0, t);

	t->func(t->arg);

	// Task finished, remove from global tracking
	for (size_t i = 0; i < lib_thread_infos_cnt; i++) {
		if (lib_thread_infos[i] == t) {
			// Shift down remaining entries
			for (size_t j = i; j < lib_thread_infos_cnt - 1; j++) {
				lib_thread_infos[j] = lib_thread_infos[j + 1];
			}
			lib_thread_infos[--lib_thread_infos_cnt] = NULL;
			break;
		}
	}

	lbm_free(t);
	vTaskDelete(NULL);  // clean self-termination
}


static bool lib_should_terminate(void) {
	lib_thd_info *info = (lib_thd_info*) pvTaskGetThreadLocalStoragePointer(NULL, 0);
	return info && info->should_terminate;
}
_Static_assert(
	configNUM_THREAD_LOCAL_STORAGE_POINTERS > 0,
	"Need ≥1 TLS pointer for lib thread bookkeeping"
);
lib_thread lispif_spawn(void (*func)(void*), size_t stack_size, const char *name, void *arg) {
	if (!lib_is_func_valid(func)) {
		commands_printf_lisp("Invalid function address. Must be static.");
		return 0;
	}

	if (lib_thread_infos_cnt >= LIB_MAX_THREADS) {
		commands_printf_lisp("Thread limit reached.");
		return 0;
	}

	lib_thd_info *info = lbm_malloc_reserve(sizeof(lib_thd_info));
	if (!info) {
		commands_printf_lisp("Failed to allocate thread info");
		return 0;
	}

	info->arg = arg;
	info->func = func;
	info->name = name;
	info->should_terminate = false;

	TaskHandle_t thd = NULL;
	BaseType_t result = xTaskCreatePinnedToCore(
		lib_thd,
		name ? name : "lib-thd",
		stack_size,
		info,
		tskIDLE_PRIORITY + 5,
		&thd,
		tskNO_AFFINITY
	);

	if (result == pdPASS && thd != NULL) {
		info->handle = thd;
		info->base_prio = uxTaskPriorityGet(thd);
		lib_thread_infos[lib_thread_infos_cnt++] = info;
		return (lib_thread)thd;
	} else {
		commands_printf_lisp("Thread creation failed");
		lbm_free(info);
		return 0;
	}
}


static void lib_request_terminate(lib_thread thd) {
	TaskHandle_t handle = (TaskHandle_t)thd;

	for (size_t i = 0; i < lib_thread_infos_cnt; i++) {
		if (lib_thread_infos[i]->handle == handle) {
			lib_thread_infos[i]->should_terminate = true;

			// Wait for task to self-terminate
			int timeout = 2000;
			while (eTaskGetState(handle) != eDeleted && timeout-- > 0) {
				vTaskDelay(pdMS_TO_TICKS(1));
			}

			if (timeout <= 0) {
				commands_printf_lisp("Thread did not exit. Crashing...");
				vTaskDelay(pdMS_TO_TICKS(20));
				abort();
			}

			return;
		}
	}

	commands_printf_lisp("Thread handle not found");
}


static inline UBaseType_t clamp_prio(int p) {
	// ESP-IDF sets FREERTOS_MAX_PRIORITIES via sdkconfig; typical is 25.
	const UBaseType_t maxp = configMAX_PRIORITIES - 1;
	const UBaseType_t minp = tskIDLE_PRIORITY + 1;
	if (p < (int)minp)
		return minp;
	if (p > (int)maxp)
		return maxp;
	return (UBaseType_t)p;
}

static void lib_thread_set_priority(int delta /* -5..5 */) {
	// Find our bookkeeping record for the CURRENT task
	lib_thd_info *info =
		(lib_thd_info *)pvTaskGetThreadLocalStoragePointer(NULL, 0);
	if (!info || info->handle != xTaskGetCurrentTaskHandle()) {
		lbm_set_error_reason(
			"thread_set_priority must be called from a lib thread"
		);
		return;
	}

	// Normalize the requested delta into absolute target priority
	// 0 => baseline, +1 => one level higher than baseline, etc.
	int target       = (int)info->base_prio + delta;
	UBaseType_t newp = clamp_prio(target);

	vTaskPrioritySet(info->handle, newp);
}

static void **lib_get_arg(uint32_t prog_addr) {
	uint32_t p = (uint32_t)utils_drom_to_irom((void *)prog_addr);

	for (int i = 0; i < LIB_NUM_MAX; i++) {
		uint32_t base = loaded_libs[i].base_addr;
		if (!base)
			continue;

		if (p == base + 4u) {
			return &loaded_libs[i].arg;
		}
	}
	return NULL;
}

static bool lib_create_byte_array(lbm_value *value, lbm_uint num_elt) {
	return lbm_heap_allocate_array(value, num_elt);
}

static bool lib_eval_is_paused(void) {
	return lbm_get_eval_state() == EVAL_CPS_STATE_PAUSED;
}

static lib_mutex lib_mutex_create(void) {
	SemaphoreHandle_t *m = lbm_malloc_reserve(sizeof(SemaphoreHandle_t));
	if (!m)
		return NULL;
	*m = xSemaphoreCreateMutex();
	if (!*m) {
		lbm_free(m);
		return NULL;
	}
	return (lib_mutex)m;
}

static void lib_mutex_lock(lib_mutex m) {
	xSemaphoreTake(*((SemaphoreHandle_t *)m), portMAX_DELAY);
}

static void lib_mutex_unlock(lib_mutex m) {
	xSemaphoreGive(*((SemaphoreHandle_t *)m));
}

static lib_semaphore lib_sem_create(void) {
	SemaphoreHandle_t *s = lbm_malloc_reserve(sizeof(SemaphoreHandle_t));
	if (!s)
		return NULL;
	*s = xSemaphoreCreateCounting(0xFFFF, 0);
	if (!*s) {
		lbm_free(s);
		return NULL;
	}
	return (lib_semaphore)s;
}

static void lib_sem_wait(lib_semaphore s) {
	xSemaphoreTake(*((SemaphoreHandle_t *)s), portMAX_DELAY);
}

static void lib_sem_signal(lib_semaphore s) {
	xSemaphoreGive(*((SemaphoreHandle_t *)s));
}

static bool lib_sem_wait_to(lib_semaphore s, TickType_t timeout_ticks) {
	return xSemaphoreTake(*((SemaphoreHandle_t *)s), timeout_ticks) == pdPASS;
}

static void lib_sem_reset(lib_semaphore s) {
	SemaphoreHandle_t h = *((SemaphoreHandle_t *)s);
	while (xSemaphoreTake(h, 0) == pdPASS) { /* drain */
	}
}

static bool lib_add_extension(char *sym_str, extension_fptr ext) {
	if (sym_str[0] != 'e' || sym_str[1] != 'x' || sym_str[2] != 't'
		|| sym_str[3] != '-') {
		lbm_set_error_reason("Error: Extensions must start with ext-");
		return false;
	}

	return lbm_add_extension(sym_str, ext);
}

static int lib_lbm_set_error_reason(char *str) {
	lbm_set_error_reason(str);
	return 1;
}

// High resolution timer for short busy-wait sleeps and time measurement
uint32_t lib_timer_time_now() {
	return (uint32_t)(esp_timer_get_time()); // microseconds
}

float lib_timer_seconds_elapsed_since(uint32_t time_us) {
	uint32_t now_us = lib_timer_time_now();
	return (now_us - time_us) / 1000000.0f;
}

void lib_timer_sleep(float seconds) {
	if (seconds <= 0)
		return;
	uint32_t us = (uint32_t)(seconds * 1000000.0f);
	while (us >= 2000) {
		vTaskDelay(pdMS_TO_TICKS(1));
		us -= 1000;
	}
	if (us)
		esp_rom_delay_us(us);
}

void lispif_stop_lib(void) {
	// 1) Call stop_fun for all loaded libs (mirrors STM32)
	for (int i = 0; i < LIB_NUM_MAX; i++) {
		if (loaded_libs[i].stop_fun) {
			if (lib_is_func_valid(loaded_libs[i].stop_fun)) {
				loaded_libs[i].stop_fun(loaded_libs[i].arg);
			}
			loaded_libs[i].stop_fun  = NULL;
			loaded_libs[i].base_addr = 0;
			loaded_libs[i].arg       = NULL;
			lib_flash_addr[i]        = 0;
		}
	}
	// 2) Terminate remaining lib threads. Snapshot the handles under the
	// lock, but request termination outside of it as lib_request_terminate
	// blocks and blocking is not allowed in a critical section.
	TaskHandle_t handles[LIB_MAX_THREADS];
	size_t handle_cnt = 0;

	LIB_THR_LOCK();
	for (size_t i = 0; i < lib_thread_infos_cnt; i++) {
		if (lib_thread_infos[i] && lib_thread_infos[i]->handle) {
			handles[handle_cnt++] = lib_thread_infos[i]->handle;
		}
	}
	LIB_THR_UNLOCK();

	for (size_t i = 0; i < handle_cnt; i++) {
		lib_request_terminate(handles[i]);
	}

	// 3) Free RAM-loaded lib images. Done last, after every lib thread is
	// gone, as their code lives in these allocations.
	for (int i = 0; i < LIB_NUM_MAX; i++) {
		if (lib_ram_alloc[i]) {
			heap_caps_free(lib_ram_alloc[i]);
			lib_ram_alloc[i] = NULL;
		}
		if (lib_ram_data[i]) {
			heap_caps_free(lib_ram_data[i]);
			lib_ram_data[i] = NULL;
		}
	}
}

// The RX callback / app-data handler pointers come from the native lib, whose
// code is reached through the flash instruction bus (IROM). Translate the
// DROM-side pointer the lib hands us to its IROM alias before the firmware
// stores and later calls it.
static void can_set_sid_rx_callback_wrapper(bool (*p_func)(uint32_t id, uint8_t *data, uint8_t len)) {
	bool (*p_func_irom)(uint32_t, uint8_t *, uint8_t) = utils_drom_to_irom(p_func);
	comm_can_set_sid_rx_callback(p_func_irom);
}

static void can_set_eid_rx_callback_wrapper(bool (*p_func)(uint32_t id, uint8_t *data, uint8_t len)) {
	bool (*p_func_irom)(uint32_t, uint8_t *, uint8_t) = utils_drom_to_irom(p_func);
	comm_can_set_eid_rx_callback(p_func_irom);
}

static bool set_app_data_handler_wrapper(void (*func)(unsigned char *data, unsigned int len)) {
	void (*func_irom)(unsigned char *, unsigned int) = utils_drom_to_irom(func);
	return commands_set_app_data_handler(func_irom);
}

// Standard BLE link connect/disconnect callback comes from the lib; translate
// to its IROM alias before comm_ble stores and later calls it.
static void ble_app_set_conn_callback_wrapper(void (*cb)(bool connected)) {
	void (*cb_irom)(bool) = utils_drom_to_irom((void *)cb);
	comm_ble_set_conn_callback(cb_irom);
}

// Device / firmware identification.
static int lib_fw_version_major(void) { return FW_VERSION_MAJOR; }
static int lib_fw_version_minor(void) { return FW_VERSION_MINOR; }
static int lib_fw_version_test(void)  { return FW_TEST_VERSION_NUMBER; }
static const char *lib_hw_name(void)  { return HW_NAME; }
static const char *lib_chip_name(void) { return CONFIG_IDF_TARGET; }

static void lib_get_mac(uint8_t *buf) {
	if (buf) {
		esp_read_mac(buf, ESP_MAC_WIFI_STA);
	}
}

static void lib_get_ble_mac(uint8_t *buf) {
	if (buf) {
		esp_read_mac(buf, ESP_MAC_BT);
	}
}

static void imu_set_read_callback_wrapper(void (*func)(float *acc, float *gyro, float *mag, float dt)) {
	void (*func_irom)(float *, float *, float *, float) = utils_drom_to_irom(func);
	imu_set_read_callback(func_irom);
}

// Custom config registration takes three lib callbacks; translate each to its
// IROM alias before the firmware stores and later calls them.
static void conf_custom_add_config_wrapper(
		int (*get_cfg)(uint8_t *data, bool is_default),
		bool (*set_cfg)(uint8_t *data), int (*get_cfg_xml)(uint8_t **data)) {
	int (*get_cfg_irom)(uint8_t *, bool) = utils_drom_to_irom(get_cfg);
	bool (*set_cfg_irom)(uint8_t *)      = utils_drom_to_irom(set_cfg);
	int (*get_cfg_xml_irom)(uint8_t **)  = utils_drom_to_irom(get_cfg_xml);
	conf_custom_add_config(get_cfg_irom, set_cfg_irom, get_cfg_xml_irom);
}

// Set remote nunchuk/joystick + button state on a motor controller over CAN.
// Encodes a COMM_SET_CHUCK_DATA packet and forwards it (send=0: process, no
// reply). js_x/js_y are 0..255 (128 = centre); acc_* are raw int16 counts.
static void can_set_chuck_data_wrapper(uint8_t controller_id, int js_x, int js_y,
		bool bt_c, bool bt_z, int acc_x, int acc_y, int acc_z) {
	uint8_t buf[11];
	int32_t ind = 0;
	buf[ind++] = COMM_SET_CHUCK_DATA;
	buf[ind++] = (uint8_t)js_x;
	buf[ind++] = (uint8_t)js_y;
	buf[ind++] = bt_c ? 1 : 0;
	buf[ind++] = bt_z ? 1 : 0;
	buffer_append_int16(buf, (int16_t)acc_x, &ind);
	buffer_append_int16(buf, (int16_t)acc_y, &ind);
	buffer_append_int16(buf, (int16_t)acc_z, &ind);
	comm_can_send_buffer(controller_id, buf, ind, 0);
}

// ---- MQTT ------------------------------------------------------------------
// Backed by comm_mqtt, which owns all esp-mqtt access. Only two adapters are
// needed here: config translation (mqtt_config -> comm_mqtt_cfg) and an event
// shim that repacks comm_mqtt_event into the lib-facing mqtt_event and calls
// the lib callback. The lib callback is carried as comm_mqtt's per-client user
// pointer, so no separate table is needed. The rest of the mqtt_* slots map
// straight onto comm_mqtt_* (identical signatures) and are wired directly.

// comm_mqtt_event and mqtt_event have the same fields; repack and forward to
// the lib callback (passed as user, already translated to its IROM alias).
static void mqtt_native_cb(void *client, const comm_mqtt_event *ev, void *user) {
	void (*lib_cb)(void *, const mqtt_event *) = user;
	if (!lib_cb) {
		return;
	}
	mqtt_event out = {0};
	out.event_id       = ev->event_id;
	out.topic          = ev->topic;
	out.topic_len      = ev->topic_len;
	out.data           = ev->data;
	out.data_len       = ev->data_len;
	out.data_offset    = ev->data_offset;
	out.total_data_len = ev->total_data_len;
	out.msg_id         = ev->msg_id;
	out.qos            = ev->qos;
	out.retain         = ev->retain;
	lib_cb(client, &out);
}

static void *mqtt_init_wrapper(const mqtt_config *cfg) {
	if (!cfg) {
		return NULL;
	}
	comm_mqtt_cfg c = {
		.uri             = cfg->uri,
		.host            = cfg->host,
		.port            = cfg->port,
		.client_id       = cfg->client_id,
		.username        = cfg->username,
		.password        = cfg->password,
		.keepalive       = cfg->keepalive,
		.lwt_topic       = cfg->lwt_topic,
		.lwt_msg         = cfg->lwt_msg,
		.lwt_qos         = cfg->lwt_qos,
		.lwt_retain      = cfg->lwt_retain,
		.server_cert_pem = cfg->server_cert_pem,
	};
	return comm_mqtt_create(&c);
}

static void mqtt_set_event_handler_wrapper(void *client,
		void (*cb)(void *client, const mqtt_event *ev)) {
	void *cb_irom = utils_drom_to_irom((void *)cb);
	comm_mqtt_set_cb(client, mqtt_native_cb, cb_irom);
}

// ---- ESP-NOW ---------------------------------------------------------------
// Backed by comm_espnow. A single native listener forwards received frames to
// the lib's rx callback (stored IROM-translated); start/add_peer/del_peer map
// straight onto comm_espnow_*, send needs an int->size_t adapter.
static void (*s_espnow_lib_rx)(const uint8_t *src, const uint8_t *data,
		int len, int rssi) = NULL;
static int s_espnow_listener = -1;

static void espnow_native_recv(const uint8_t *src, const uint8_t *des,
		const uint8_t *data, int len, int rssi, void *user) {
	(void)des;
	(void)user;
	void (*cb)(const uint8_t *, const uint8_t *, int, int) = s_espnow_lib_rx;
	if (cb) {
		cb(src, data, len, rssi);
	}
}

static void espnow_set_rx_callback_wrapper(
		void (*cb)(const uint8_t *src, const uint8_t *data, int len, int rssi)) {
	s_espnow_lib_rx = utils_drom_to_irom((void *)cb);
	if (s_espnow_listener < 0) {
		s_espnow_listener =
			comm_espnow_add_listener(espnow_native_recv, NULL, NULL);
	}
}

static int espnow_send_wrapper(const uint8_t *mac, const uint8_t *data, int len) {
	return comm_espnow_send(mac, data, (size_t)len);
}

// ---- BLE GATT server (via custom_ble) --------------------------------------
#if !CONFIG_IDF_TARGET_ESP32P4

// custom_ble_add_service reports the created handles through a callback that
// carries no user pointer, and blocks until it has fired. Serialize via this
// static capture (the underlying API is single-thread-only anyway).
typedef struct { uint16_t *out; int cap; int count; } ble_handles_cap_t;
static ble_handles_cap_t *s_ble_cap = NULL;

static void ble_handles_trampoline(uint16_t count, const uint16_t handles[]) {
	if (!s_ble_cap) {
		return;
	}
	int n = (int)count;
	if (n > s_ble_cap->cap) {
		n = s_ble_cap->cap;
	}
	for (int i = 0; i < n; i++) {
		s_ble_cap->out[i] = handles[i];
	}
	s_ble_cap->count = (int)count;
}

static esp_bt_uuid_t ble_to_esp_uuid(const ble_uuid *u) {
	esp_bt_uuid_t e;
	memset(&e, 0, sizeof(e));
	e.len = u->len;
	if (u->len == ESP_UUID_LEN_16) {
		memcpy(&e.uuid.uuid16, u->uuid, 2);
	} else if (u->len == ESP_UUID_LEN_32) {
		memcpy(&e.uuid.uuid32, u->uuid, 4);
	} else {
		memcpy(e.uuid.uuid128, u->uuid, 16);
	}
	return e;
}

static bool ble_start_wrapper(void) {
	custom_ble_result_t r = custom_ble_start();
	return r == CUSTOM_BLE_OK || r == CUSTOM_BLE_ALREADY_STARTED;
}

static bool ble_set_name_wrapper(const char *name) {
	return custom_ble_set_name(name) == CUSTOM_BLE_OK;
}

static bool ble_update_adv_wrapper(bool use_raw, const uint8_t *adv,
		int adv_len, const uint8_t *scan_rsp, int scan_rsp_len) {
	return custom_ble_update_adv(use_raw, (size_t)adv_len, adv,
			(size_t)scan_rsp_len, scan_rsp) == CUSTOM_BLE_OK;
}

static int ble_add_service_wrapper(const ble_uuid *uuid,
		const ble_chr_def *chrs, int chr_count, uint16_t *handles,
		int handles_cap) {
	if (!uuid || (chr_count > 0 && !chrs)) {
		return -1;
	}

	ble_chr_definition_t *ec = NULL;
	if (chr_count > 0) {
		ec = calloc(chr_count, sizeof(ble_chr_definition_t));
		if (!ec) {
			return -1;
		}
		for (int i = 0; i < chr_count; i++) {
			ec[i].uuid          = ble_to_esp_uuid(&chrs[i].uuid);
			ec[i].perm          = chrs[i].perm;
			ec[i].property      = chrs[i].prop;
			ec[i].value_max_len = chrs[i].max_len;
			ec[i].value_len     = chrs[i].init_len;
			ec[i].value         = (uint8_t *)chrs[i].init;
			ec[i].descr_count   = chrs[i].descr_count;
			ec[i].descriptors   = NULL;
			if (chrs[i].descr_count > 0 && chrs[i].descrs) {
				ble_desc_definition_t *dd =
					calloc(chrs[i].descr_count, sizeof(ble_desc_definition_t));
				if (!dd) {
					for (int k = 0; k < i; k++) {
						free((void *)ec[k].descriptors);
					}
					free(ec);
					return -1;
				}
				for (int j = 0; j < chrs[i].descr_count; j++) {
					dd[j].uuid          = ble_to_esp_uuid(&chrs[i].descrs[j].uuid);
					dd[j].perm          = chrs[i].descrs[j].perm;
					dd[j].value_max_len = chrs[i].descrs[j].max_len;
					dd[j].value_len     = chrs[i].descrs[j].init_len;
					dd[j].value         = (uint8_t *)chrs[i].descrs[j].init;
				}
				ec[i].descriptors = dd;
			}
		}
	}

	ble_handles_cap_t cap = { handles, handles_cap, 0 };
	s_ble_cap = &cap;
	custom_ble_result_t res = custom_ble_add_service(
			ble_to_esp_uuid(uuid), (uint16_t)chr_count, ec,
			ble_handles_trampoline);
	s_ble_cap = NULL;

	if (ec) {
		for (int i = 0; i < chr_count; i++) {
			free((void *)ec[i].descriptors);
		}
		free(ec);
	}

	return (res == CUSTOM_BLE_OK) ? cap.count : -1;
}

static bool ble_remove_service_wrapper(uint16_t service_handle) {
	return custom_ble_remove_service(service_handle) == CUSTOM_BLE_OK;
}

static int ble_attr_get_value_wrapper(uint16_t handle, uint8_t *out,
		int out_cap) {
	uint16_t len = 0;
	const uint8_t *val = NULL;
	if (custom_ble_get_attr_value(handle, &len, &val) != CUSTOM_BLE_OK || !val) {
		return -1;
	}
	int n = (int)len;
	if (n > out_cap) {
		n = out_cap;
	}
	if (out && n > 0) {
		memcpy(out, val, n);
	}
	return (int)len; // full length, even if truncated into out
}

static bool ble_attr_set_value_wrapper(uint16_t handle, const uint8_t *data,
		int len) {
	return custom_ble_set_attr_value(handle, (uint16_t)len, data)
			== CUSTOM_BLE_OK;
}

// The lib's write callback, carried IROM-translated. One custom_ble write
// listener forwards to it.
static void (*s_ble_lib_write)(uint16_t handle, const uint8_t *data, int len)
		= NULL;
static int s_ble_write_listener = -1;

static void ble_native_write(uint16_t attr_handle, uint16_t len,
		uint8_t *value, void *user) {
	(void)user;
	void (*cb)(uint16_t, const uint8_t *, int) = s_ble_lib_write;
	if (cb) {
		cb(attr_handle, value, (int)len);
	}
}

static void ble_set_write_callback_wrapper(
		void (*cb)(uint16_t handle, const uint8_t *data, int len)) {
	s_ble_lib_write = utils_drom_to_irom((void *)cb);
	if (s_ble_write_listener < 0) {
		s_ble_write_listener =
			custom_ble_add_write_listener(ble_native_write, NULL);
	}
}

#endif // !CONFIG_IDF_TARGET_ESP32P4

// GPIO helpers. mode: 0=input, 1=input/output, 2=open-drain. pull: 0=none,
// 1=up, 2=down. Guarded by utils_gpio_is_valid so a bad pin is a no-op.
static void gpio_configure_wrapper(int pin, int mode, int pull) {
	if (!utils_gpio_is_valid(pin)) {
		return;
	}
	gpio_config_t c = {0};
	c.pin_bit_mask = 1ULL << pin;
	c.intr_type    = GPIO_INTR_DISABLE;
	switch (mode) {
		case 0:  c.mode = GPIO_MODE_INPUT; break;
		case 2:  c.mode = GPIO_MODE_INPUT_OUTPUT_OD; break;
		case 1:  c.mode = GPIO_MODE_INPUT_OUTPUT; break;
		default: c.mode = GPIO_MODE_DISABLE; break;
	}
	c.pull_up_en   = (pull == 1) ? GPIO_PULLUP_ENABLE   : GPIO_PULLUP_DISABLE;
	c.pull_down_en = (pull == 2) ? GPIO_PULLDOWN_ENABLE : GPIO_PULLDOWN_DISABLE;
	gpio_reset_pin(pin);
	gpio_config(&c);
}

static void gpio_write_wrapper(int pin, bool state) {
	if (utils_gpio_is_valid(pin)) {
		gpio_set_level(pin, state ? 1 : 0);
	}
}

static bool gpio_read_wrapper(int pin) {
	if (!utils_gpio_is_valid(pin)) {
		return false;
	}
	return gpio_get_level(pin) != 0;
}

// I2C combined transaction. Returns the esp_err_t as an int (0 == ESP_OK).
static int i2c_tx_rx_wrapper(uint8_t addr, const uint8_t *write, size_t wlen,
		uint8_t *read, size_t rlen) {
	return (int)lispif_i2c_tx_rx(addr, write, wlen, read, rlen);
}

// BMS command handler is called by the firmware, so translate the lib's
// pointer to its IROM alias first. cmd is a COMM_PACKET_ID (== int on ABI).
static void bms_set_cmd_handler_wrapper(void (*handler)(int cmd, int param1, int param2)) {
	void (*handler_irom)(COMM_PACKET_ID, int, int) = utils_drom_to_irom(handler);
	bms_register_cmd_handler(handler_irom);
}

lbm_value ext_load_native_lib(lbm_value *args, lbm_uint argn) {
	lbm_value res = ENC_SYM_EERROR;

	if (argn != 1) {
		return ENC_SYM_TERROR;
	}

	if (!lbm_is_array_r(args[0])) {
		return ENC_SYM_TERROR;
	}

	if (lbm_is_array_rw(args[0])) {
		lbm_set_error_reason("Native library must be in flash");
		return ENC_SYM_TERROR;
	}

	uint32_t irom_base = 0;
	bool is_reloc = false;

	lbm_array_header_t *array = (lbm_array_header_t *)lbm_car(args[0]);
	if (array->size > 12) {
		uint32_t magic_be = 0;
		memcpy(
			&magic_be, (const uint8_t *)array->data,
			sizeof(magic_be)
		);

		if (magic_be == __builtin_bswap32(NATIVE_LIB_MAGIC) ||
			magic_be == __builtin_bswap32(NATIVE_LIB_RELOC_MAGIC)) {
			irom_base = (uint32_t)utils_drom_to_irom(array->data);
			is_reloc = magic_be == __builtin_bswap32(NATIVE_LIB_RELOC_MAGIC);
		} else {
			lbm_set_error_reason("Magic number not found at IROM address");
			return res;
		}
	}

	if (irom_base == 0 || (irom_base & 0x3) != 0) {
		lbm_set_error_reason("Invalid address");
		return res;
	}

	// The linker script and VESC_IF must agree on where the interface table
	// lives, otherwise libs would read garbage function pointers.
	if ((uintptr_t)&cif != (uintptr_t)VESC_IF) {
		lbm_set_error_reason("Native lib interface address mismatch (firmware bug)");
		return res;
	}

	if (!lib_init_done) {
		// Zero the padding beyond the struct; slots appended to the
		// interface after this firmware was built read as NULL.
		memset((char *)cif.pad, 0, 2048);

		cif.cif.if_version = VESC_C_IF_VERSION;

		// LBM
		cif.cif.lbm_add_extension            = lib_add_extension;
		cif.cif.lbm_block_ctx_from_extension = lbm_block_ctx_from_extension;
		cif.cif.lbm_unblock_ctx              = lbm_unblock_ctx;
		cif.cif.lbm_get_current_cid          = lbm_get_current_cid;
		cif.cif.lbm_set_error_reason         = lib_lbm_set_error_reason;
		cif.cif.lbm_pause_eval_with_gc       = lbm_pause_eval_with_gc;
		cif.cif.lbm_continue_eval            = lbm_continue_eval;
		cif.cif.lbm_send_message             = lbm_send_message;
		cif.cif.lbm_eval_is_paused           = lib_eval_is_paused;

		cif.cif.lbm_cons                     = lbm_cons;
		cif.cif.lbm_car                      = lbm_car;
		cif.cif.lbm_cdr                      = lbm_cdr;
		cif.cif.lbm_list_destructive_reverse = lbm_list_destructive_reverse;
		cif.cif.lbm_create_byte_array        = lib_create_byte_array;

		cif.cif.lbm_add_symbol_const   = lbm_add_symbol_const;
		cif.cif.lbm_get_symbol_by_name = lbm_get_symbol_by_name;

		cif.cif.lbm_enc_i     = lbm_enc_i;
		cif.cif.lbm_enc_u     = lbm_enc_u;
		cif.cif.lbm_enc_char  = lbm_enc_char;
		cif.cif.lbm_enc_float = lbm_enc_float;
		cif.cif.lbm_enc_u32   = lbm_enc_u32;
		cif.cif.lbm_enc_i32   = lbm_enc_i32;
		cif.cif.lbm_enc_sym   = lbm_enc_sym;

		cif.cif.lbm_dec_as_float = lbm_dec_as_float;
		cif.cif.lbm_dec_as_u32   = lbm_dec_as_u32;
		cif.cif.lbm_dec_as_i32   = lbm_dec_as_i32;
		cif.cif.lbm_dec_char     = lbm_dec_char;
		cif.cif.lbm_dec_str      = lbm_dec_str;
		cif.cif.lbm_dec_sym      = lbm_dec_sym;

		cif.cif.lbm_is_byte_array = lbm_is_array_r;
		cif.cif.lbm_is_cons       = lbm_is_cons;
		cif.cif.lbm_is_number     = lbm_is_number;
		cif.cif.lbm_is_char       = lbm_is_char;
		cif.cif.lbm_is_symbol     = lbm_is_symbol;

		cif.cif.lbm_enc_sym_nil    = ENC_SYM_NIL;
		cif.cif.lbm_enc_sym_true   = ENC_SYM_TRUE;
		cif.cif.lbm_enc_sym_terror = ENC_SYM_TERROR;
		cif.cif.lbm_enc_sym_eerror = ENC_SYM_EERROR;
		cif.cif.lbm_enc_sym_merror = ENC_SYM_MERROR;

		cif.cif.lbm_is_symbol_nil  = lbm_is_symbol_nil;
		cif.cif.lbm_is_symbol_true = lbm_is_symbol_true;

		// Os
		cif.cif.sleep_ms          = lib_sleep_ms;
		cif.cif.sleep_us          = lib_sleep_us;
		cif.cif.system_time       = lib_system_time;
		cif.cif.ts_to_age_s       = lib_ts_to_age_s;
		cif.cif.printf            = commands_printf_lisp;
		cif.cif.malloc            = lbm_malloc_reserve;
		cif.cif.free              = lbm_free;
		cif.cif.spawn             = lispif_spawn;
		cif.cif.request_terminate = lib_request_terminate;
		cif.cif.should_terminate  = lib_should_terminate;
		cif.cif.get_arg           = lib_get_arg;

		// Mutex
		cif.cif.mutex_create = lib_mutex_create;
		cif.cif.mutex_lock   = lib_mutex_lock;
		cif.cif.mutex_unlock = lib_mutex_unlock;

		// High resolution timer for short busy-wait sleeps and time measurement
		cif.cif.timer_time_now              = lib_timer_time_now;
		cif.cif.timer_seconds_elapsed_since = lib_timer_seconds_elapsed_since;
		cif.cif.timer_sleep                 = lib_timer_sleep;

		// Flat values
		cif.cif.lbm_start_flatten  = lbm_start_flatten;
		cif.cif.lbm_finish_flatten = lbm_finish_flatten;
		cif.cif.f_b                = f_b;
		cif.cif.f_cons             = f_cons;
		cif.cif.f_float            = f_float;
		cif.cif.f_i                = f_i;
		cif.cif.f_i32              = f_i32;
		cif.cif.f_i64              = f_i64;
		cif.cif.f_lbm_array        = f_lbm_array;
		cif.cif.f_sym              = f_sym;
		cif.cif.f_u32              = f_u32;
		cif.cif.f_u64              = f_u64;

		// Unblock unboxed
		cif.cif.lbm_unblock_ctx_unboxed = lbm_unblock_ctx_unboxed;

		// System time
		cif.cif.system_time_ticks = xTaskGetTickCount;
		cif.cif.sleep_ticks       = vTaskDelay;

		// Semaphores
		cif.cif.sem_create  = lib_sem_create;
		cif.cif.sem_wait    = lib_sem_wait;
		cif.cif.sem_signal  = lib_sem_signal;
		cif.cif.sem_wait_to = lib_sem_wait_to;
		cif.cif.sem_reset   = lib_sem_reset;

		cif.cif.thread_set_priority = lib_thread_set_priority;

		// Device / firmware identification
		cif.cif.fw_version_major = lib_fw_version_major;
		cif.cif.fw_version_minor = lib_fw_version_minor;
		cif.cif.fw_version_test  = lib_fw_version_test;
		cif.cif.hw_name          = lib_hw_name;
		cif.cif.chip_name        = lib_chip_name;
		cif.cif.get_mac          = lib_get_mac;

		// CAN bus
		cif.cif.can_transmit_sid = comm_can_transmit_sid;
		cif.cif.can_transmit_eid = comm_can_transmit_eid;
		cif.cif.can_send_buffer  = comm_can_send_buffer;
		cif.cif.can_set_sid_rx_callback = can_set_sid_rx_callback_wrapper;
		cif.cif.can_set_eid_rx_callback = can_set_eid_rx_callback_wrapper;
		cif.cif.can_set_duty     = comm_can_set_duty;
		cif.cif.can_set_current  = comm_can_set_current;
		cif.cif.can_set_current_off_delay = comm_can_set_current_off_delay;
		cif.cif.can_set_current_brake     = comm_can_set_current_brake;
		cif.cif.can_set_rpm      = comm_can_set_rpm;
		cif.cif.can_set_pos      = comm_can_set_pos;
		cif.cif.can_set_current_rel = comm_can_set_current_rel;
		cif.cif.can_set_current_rel_off_delay = comm_can_set_current_rel_off_delay;
		cif.cif.can_set_current_brake_rel = comm_can_set_current_brake_rel;
		cif.cif.can_ping         = comm_can_ping;
		cif.cif.can_get_status_msg_index   = comm_can_get_status_msg_index;
		cif.cif.can_get_status_msg_id      = comm_can_get_status_msg_id;
		cif.cif.can_get_status_msg_2_index = comm_can_get_status_msg_2_index;
		cif.cif.can_get_status_msg_2_id    = comm_can_get_status_msg_2_id;
		cif.cif.can_get_status_msg_3_index = comm_can_get_status_msg_3_index;
		cif.cif.can_get_status_msg_3_id    = comm_can_get_status_msg_3_id;
		cif.cif.can_get_status_msg_4_index = comm_can_get_status_msg_4_index;
		cif.cif.can_get_status_msg_4_id    = comm_can_get_status_msg_4_id;
		cif.cif.can_get_status_msg_5_index = comm_can_get_status_msg_5_index;
		cif.cif.can_get_status_msg_5_id    = comm_can_get_status_msg_5_id;
		cif.cif.can_get_status_msg_6_index = comm_can_get_status_msg_6_index;
		cif.cif.can_get_status_msg_6_id    = comm_can_get_status_msg_6_id;

		// App comms
		cif.cif.send_app_data        = commands_send_app_data;
		cif.cif.set_app_data_handler = set_app_data_handler_wrapper;

		// IMU
		cif.cif.imu_startup_done       = imu_startup_done;
		cif.cif.imu_get_roll           = imu_get_roll;
		cif.cif.imu_get_pitch          = imu_get_pitch;
		cif.cif.imu_get_yaw            = imu_get_yaw;
		cif.cif.imu_get_rpy            = imu_get_rpy;
		cif.cif.imu_get_accel          = imu_get_accel;
		cif.cif.imu_get_gyro           = imu_get_gyro;
		cif.cif.imu_get_mag            = imu_get_mag;
		cif.cif.imu_derotate           = imu_derotate;
		cif.cif.imu_get_accel_derotated = imu_get_accel_derotated;
		cif.cif.imu_get_gyro_derotated  = imu_get_gyro_derotated;
		cif.cif.imu_get_quaternions    = imu_get_quaternions;
		cif.cif.imu_get_calibration    = imu_get_calibration;
		cif.cif.imu_set_read_callback  = imu_set_read_callback_wrapper;

		// AHRS
		cif.cif.ahrs_init_attitude_info      = ahrs_init_attitude_info;
		cif.cif.ahrs_update_all_parameters   = ahrs_update_all_parameters;
		cif.cif.ahrs_update_initial_orientation = ahrs_update_initial_orientation;
		cif.cif.ahrs_update_mahony_imu       = ahrs_update_mahony_imu;
		cif.cif.ahrs_update_madgwick_imu     = ahrs_update_madgwick_imu;
		cif.cif.ahrs_get_roll                = ahrs_get_roll;
		cif.cif.ahrs_get_pitch               = ahrs_get_pitch;
		cif.cif.ahrs_get_yaw                 = ahrs_get_yaw;
		cif.cif.ahrs_get_roll_pitch_yaw      = ahrs_get_roll_pitch_yaw;

		// Persistent storage
		cif.cif.store_eeprom_var = store_eeprom_var;
		cif.cif.read_eeprom_var  = read_eeprom_var;
		cif.cif.store_eeprom_var_batch = store_eeprom_var_batch;
		cif.cif.read_eeprom_var_batch  = read_eeprom_var_batch;

		// Custom config (VESC Tool settings page)
		cif.cif.conf_custom_add_config    = conf_custom_add_config_wrapper;
		cif.cif.conf_custom_clear_configs = conf_custom_clear_configs;

		// Wi-Fi
		cif.cif.wifi_is_connected        = comm_wifi_is_connected;
		cif.cif.wifi_is_client_connected = comm_wifi_is_client_connected;
		cif.cif.wifi_is_connecting       = comm_wifi_is_connecting;
		cif.cif.wifi_disconnect          = comm_wifi_disconnect;
		cif.cif.wifi_change_network      = comm_wifi_change_network;
		cif.cif.wifi_reconnect_network   = comm_wifi_reconnect_network;
		cif.cif.wifi_disconnect_network  = comm_wifi_disconnect_network;
		cif.cif.wifi_set_auto_reconnect  = comm_wifi_set_auto_reconnect;
		cif.cif.wifi_get_auto_reconnect  = comm_wifi_get_auto_reconnect;

		// CAN: more remote-device commands
		cif.cif.can_set_handbrake     = comm_can_set_handbrake;
		cif.cif.can_set_handbrake_rel = comm_can_set_handbrake_rel;
		cif.cif.can_io_board_set_output_digital = comm_can_io_board_set_output_digital;
		cif.cif.can_io_board_set_output_pwm     = comm_can_io_board_set_output_pwm;
		cif.cif.can_psw_switch            = comm_can_psw_switch;
		cif.cif.can_update_pid_pos_offset = comm_can_update_pid_pos_offset;
		cif.cif.can_set_chuck_data        = can_set_chuck_data_wrapper;

		// GPIO
		cif.cif.gpio_configure = gpio_configure_wrapper;
		cif.cif.gpio_write     = gpio_write_wrapper;
		cif.cif.gpio_read      = gpio_read_wrapper;

		// I2C
		cif.cif.i2c_tx_rx = i2c_tx_rx_wrapper;

		// GNSS
		cif.cif.gnss_get_state = nmea_get_state;
		cif.cif.gnss_fix_type  = nmea_fix_type;

		// BMS
		cif.cif.bms_get_values      = bms_get_values;
		cif.cif.bms_send_status_can = bms_send_status_can;
		cif.cif.bms_set_cmd_handler = bms_set_cmd_handler_wrapper;

		// Wi-Fi raw send
		cif.cif.wifi_send_raw_local = comm_wifi_send_raw_local;
		cif.cif.wifi_send_raw_hub   = comm_wifi_send_raw_hub;

		// MQTT (via comm_mqtt). Direct-signature slots map straight onto
		// comm_mqtt_*; init/set_event_handler need small adapters.
		cif.cif.mqtt_init              = mqtt_init_wrapper;
		cif.cif.mqtt_start             = comm_mqtt_start;
		cif.cif.mqtt_stop              = comm_mqtt_stop;
		cif.cif.mqtt_publish           = comm_mqtt_publish;
		cif.cif.mqtt_subscribe         = comm_mqtt_subscribe;
		cif.cif.mqtt_unsubscribe       = comm_mqtt_unsubscribe;
		cif.cif.mqtt_destroy           = comm_mqtt_destroy;
		cif.cif.mqtt_set_event_handler = mqtt_set_event_handler_wrapper;

#if !CONFIG_IDF_TARGET_ESP32P4
		// ESP-NOW (via comm_espnow)
		cif.cif.espnow_start          = comm_espnow_start;
		cif.cif.espnow_add_peer       = comm_espnow_add_peer;
		cif.cif.espnow_del_peer       = comm_espnow_del_peer;
		cif.cif.espnow_send           = espnow_send_wrapper;
		cif.cif.espnow_set_rx_callback = espnow_set_rx_callback_wrapper;

		// BLE GATT server (via custom_ble)
		cif.cif.ble_start             = ble_start_wrapper;
		cif.cif.ble_started           = custom_ble_started;
		cif.cif.ble_set_name          = ble_set_name_wrapper;
		cif.cif.ble_update_adv        = ble_update_adv_wrapper;
		cif.cif.ble_add_service       = ble_add_service_wrapper;
		cif.cif.ble_remove_service    = ble_remove_service_wrapper;
		cif.cif.ble_attr_get_value    = ble_attr_get_value_wrapper;
		cif.cif.ble_attr_set_value    = ble_attr_set_value_wrapper;
		cif.cif.ble_set_write_callback = ble_set_write_callback_wrapper;

		// Standard BLE app link (comm_ble has P4 stubs, so no guard needed).
		cif.cif.ble_app_connected         = comm_ble_is_connected;
		cif.cif.ble_app_mtu               = comm_ble_mtu_now;
		cif.cif.ble_app_set_conn_callback = ble_app_set_conn_callback_wrapper;
		cif.cif.get_ble_mac      = lib_get_ble_mac;
#endif

		// RGB LED strip
		cif.cif.rgbled_init   = rgbled_init;
		cif.cif.rgbled_deinit = rgbled_deinit;
		cif.cif.rgbled_update = rgbled_update;

		lib_init_done = true;
	}

	// Duplicate check by container flash address
	for (int i = 0; i < LIB_NUM_MAX; i++) {
		if (loaded_libs[i].stop_fun != NULL
			&& lib_flash_addr[i] == irom_base) {
			lbm_set_error_reason("Library already loaded");
			return res;
		}
	}

	int slot = -1;
	for (int i = 0; i < LIB_NUM_MAX; i++) {
		if (loaded_libs[i].stop_fun == NULL) {
			slot = i;
			break;
		}
	}
	if (slot < 0) {
		lbm_set_error_reason("Library table full");
		return res;
	}

	// base_addr is where the lib image lives at runtime (prog_ptr at +4),
	// entry_addr is the init function.
	uint32_t base_addr;
	uint32_t entry_addr;
	void *ram_alloc = NULL;
	void *ram_data = NULL;

	if (is_reloc) {
#if CONFIG_IDF_TARGET_ESP32S3 && CONFIG_ESP_SYSTEM_MEMPROT_FEATURE
		// The exec-heap allocation below can never succeed with memory
		// protection enabled, so fail with a message that says exactly
		// what is wrong with this firmware build.
		lbm_set_error_reason("This firmware was built with "
			"CONFIG_ESP_SYSTEM_MEMPROT_FEATURE=y - native libs on the "
			"ESP32-S3 need a build with it disabled");
		return res;
#elif CONFIG_IDF_TARGET_ESP32S3
		// Xtensa cannot run position-independent code in place, so the
		// container carries region-relative relocations and the image is
		// copied to RAM in two parts: the code region goes to executable
		// memory (any exec block works - including pure-IRAM without a
		// data alias, since code is only word-accessed) and the data
		// region to any byte-accessible internal RAM (including the
		// DRAM-only spare). This keeps native libs out of the contested
		// D/IRAM that LispBM needs. Container layout after the magic:
		// version, code_size, data_size, entry_offset, reloc_count (all
		// LE u32), relocs[], code[], data[].
		uint32_t version, code_size, data_size, entry_offset, reloc_count;
		const uint8_t *container_drom = (const uint8_t *)array->data;
		memcpy(&version, container_drom + 4, 4);
		memcpy(&code_size, container_drom + 8, 4);
		memcpy(&data_size, container_drom + 12, 4);
		memcpy(&entry_offset, container_drom + 16, 4);
		memcpy(&reloc_count, container_drom + 20, 4);

		if (version != 2) {
			lbm_set_error_reason("Native lib container version mismatch - "
				"rebuild the lib with the current vesc_pkg c_libs");
			return res;
		}

		if (code_size < 4 || code_size > 0x40000 || (code_size & 3)
			|| data_size < 8 || data_size > 0x40000 || (data_size & 3)
			|| entry_offset >= code_size || (entry_offset & 3)
			|| reloc_count > (code_size + data_size) / 4) {
			lbm_set_error_reason("Invalid native lib container");
			return res;
		}

		uint32_t *code_ram = heap_caps_malloc(
			code_size, MALLOC_CAP_EXEC | MALLOC_CAP_INTERNAL);
		uint8_t *data_ram = heap_caps_malloc(
			data_size, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
		if (!code_ram || !data_ram) {
			static char err_buf[96];
			snprintf(err_buf, sizeof(err_buf),
				"Out of memory for lib: code %u (largest %u), data %u (largest %u)",
				(unsigned)code_size,
				(unsigned)heap_caps_get_largest_free_block(
					MALLOC_CAP_EXEC | MALLOC_CAP_INTERNAL),
				(unsigned)data_size,
				(unsigned)heap_caps_get_largest_free_block(
					MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
			if (code_ram) heap_caps_free(code_ram);
			if (data_ram) heap_caps_free(data_ram);
			lbm_set_error_reason(err_buf);
			return res;
		}

		const uint8_t *relocs = container_drom + 24;
		const uint8_t *code_src = relocs + reloc_count * 4;
		const uint8_t *data_src = code_src + code_size;

		// The code block may be pure IRAM, which only allows aligned
		// 32-bit accesses - copy and patch it word-wise.
		for (uint32_t i = 0; i < code_size / 4; i++) {
			uint32_t w;
			memcpy(&w, code_src + i * 4, 4);
			code_ram[i] = w;
		}
		memcpy(data_ram, data_src, data_size);

		// Relocation entries: bit31 = target is code, bit30 = the word
		// sits in the data region, low bits = region-relative offset of
		// the word. Stored words are region-relative target offsets.
		bool patch_ok = true;
		for (uint32_t r = 0; r < reloc_count; r++) {
			uint32_t e;
			memcpy(&e, relocs + r * 4, 4);
			uint32_t off = e & 0x3FFFFFFF;
			uint32_t add = (e & 0x80000000)
				? (uint32_t)code_ram : (uint32_t)data_ram;

			if (e & 0x40000000) {
				if ((off & 3) || off + 4 > data_size) {
					patch_ok = false;
					break;
				}
				uint32_t word;
				memcpy(&word, data_ram + off, 4);
				word += add;
				memcpy(data_ram + off, &word, 4);
			} else {
				if ((off & 3) || off + 4 > code_size) {
					patch_ok = false;
					break;
				}
				code_ram[off / 4] += add;
			}
		}

		uint32_t inner_magic = 0;
		memcpy(&inner_magic, data_ram, 4);
		if (!patch_ok || inner_magic != __builtin_bswap32(NATIVE_LIB_MAGIC)) {
			heap_caps_free(code_ram);
			heap_caps_free(data_ram);
			lbm_set_error_reason("Invalid relocation table in native lib");
			return res;
		}

		// Make the copied and patched code visible to instruction fetch.
		__asm__ __volatile__("memw\n\tisync\n\t" ::: "memory");

		base_addr  = (uint32_t)data_ram;
		entry_addr = (uint32_t)code_ram + entry_offset;
		ram_alloc  = code_ram;
		ram_data   = data_ram;
#else
		lbm_set_error_reason(
			"Relocatable libs are only supported on the ESP32-S3");
		return res;
#endif
	} else {
		// XIP: runs in place from flash. Entry is after the header:
		// magic(4) + prog_addr(4) = 8 bytes.
		base_addr  = irom_base;
		entry_addr = irom_base + 8;
	}

	loaded_libs[slot].base_addr = base_addr;
	lib_flash_addr[slot]        = irom_base;
	lib_ram_alloc[slot]         = ram_alloc;
	lib_ram_data[slot]          = ram_data;

	bool ok = ((bool (*)(lib_info *))entry_addr)(&loaded_libs[slot]);

	if (ok && loaded_libs[slot].stop_fun != NULL) {
		void *stop_fun_irom = utils_drom_to_irom(loaded_libs[slot].stop_fun);
		if (lib_is_func_valid(stop_fun_irom)) {
			loaded_libs[slot].stop_fun = stop_fun_irom;
			return lbm_enc_sym(SYM_TRUE);
		}
		lbm_set_error_reason("Invalid stop function. Must be static.");
	} else if (ok) {
		lbm_set_error_reason("Library init failed - no stop function set");
	} else {
		lbm_set_error_reason("Library init failed");
	}

	// Rollback
	loaded_libs[slot].stop_fun  = NULL;
	loaded_libs[slot].base_addr = 0;
	loaded_libs[slot].arg       = NULL;
	lib_flash_addr[slot]        = 0;
	if (lib_ram_alloc[slot]) {
		heap_caps_free(lib_ram_alloc[slot]);
		lib_ram_alloc[slot] = NULL;
	}
	if (lib_ram_data[slot]) {
		heap_caps_free(lib_ram_data[slot]);
		lib_ram_data[slot] = NULL;
	}

	return res;
}

lbm_value ext_unload_native_lib(lbm_value *args, lbm_uint argn) {
	lbm_value res = lbm_enc_sym(SYM_EERROR);

	if (argn != 1 || !lbm_is_number(args[0])) {
		return res;
	}

	uint32_t irom_base = lbm_dec_as_u32(args[0]);

	for (int i = 0; i < LIB_NUM_MAX; i++) {
		if (loaded_libs[i].stop_fun != NULL
			&& lib_flash_addr[i] == irom_base) {
			// The stop function must stop everything the lib started,
			// including its threads, before returning.
			if (lib_is_func_valid(loaded_libs[i].stop_fun)) {
				loaded_libs[i].stop_fun(loaded_libs[i].arg);
			}
			loaded_libs[i].stop_fun  = NULL;
			loaded_libs[i].base_addr = 0;
			loaded_libs[i].arg       = NULL;
			lib_flash_addr[i]        = 0;
			if (lib_ram_alloc[i]) {
				heap_caps_free(lib_ram_alloc[i]);
				lib_ram_alloc[i] = NULL;
			}
			if (lib_ram_data[i]) {
				heap_caps_free(lib_ram_data[i]);
				lib_ram_data[i] = NULL;
			}

			return lbm_enc_sym(SYM_TRUE);
		}
	}

	lbm_set_error_reason("Library not loaded");
	return res;
}
