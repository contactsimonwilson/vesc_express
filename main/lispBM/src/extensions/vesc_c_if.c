/*
    Copyright 2022‑2024 Benjamin Vedder     <benjamin@vedder.se>
    Copyright 2022      Joel Svensson      <svenssonjoel@yahoo.se>

    This file is part of the VESC firmware and is released under the
    terms of the GNU General Public License, version 3 (or any later).

    -----------------------------------------------------------------
    FreeRTOS / ESP‑IDF PORT
    -----------------------------------------------------------------
    All ChibiOS threading primitives have been replaced with their
    FreeRTOS counterparts so that `(spawn …)` and related Lisp helpers
    work unmodified on ESP32 targets.  No other functionality has been
    changed.
*/

#pragma GCC optimize("Os")
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "esp_timer.h"
#include "esp_rom_sys.h"
#include "esp_heap_caps.h"
#include "esp_memory_utils.h"
#include "heap_memory_layout.h"
#include "ahrs.h"
#include "commands.h"
#include "comm_can.h"
#include "crypto.h"
#include "extensions.h"
#include "packet.h"
#include "lbm_flat_value.h"
#include "flash_helper.h"
#include "lispif.h"
#include "lispbm.h"
#include "mempools.h"
#include "terminal.h"
#include "utils.h"
#include "imu.h"
#include "main.h"
#include "extensions/vesc_c_if.h"
#include "conf_custom.h"
#include "freertos/task.h"
#include "lispif_rgbled_extensions.h"
#include "lbm_color_extensions.h"

_Static_assert(sizeof(vesc_c_if) <= 2048, "cif pad too small");

// Function prototypes otherwise missing
void packet_init(
	void (*s_func)(unsigned char *data, unsigned int len),
	void (*p_func)(unsigned char *data, unsigned int len), PACKET_STATE_t *state
);
void packet_reset(PACKET_STATE_t *state);
void packet_process_byte(uint8_t rx_data, PACKET_STATE_t *state);
void packet_send_packet(
	unsigned char *data, unsigned int len, PACKET_STATE_t *state
);

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

#if CONFIG_IDF_TARGET_ESP32S3
// Static D/IRAM pool for RAM-loaded libs. Static .bss lives in the
// D/IRAM address range and is executable through the IRAM alias with
// memory protection off, and unlike a heap reservation it cannot
// fragment the heap: the heap region simply starts a little higher, and
// the linker fails the build if it does not fit. A simple bump
// allocator serves lib loads and resets when every lib is unloaded
// (lisp restart), so restarts always reuse the same bytes.
#define LIB_POOL_SIZE (20 * 1024)
static uint8_t lib_pool[LIB_POOL_SIZE] __attribute__((aligned(8)));
static uint32_t lib_pool_used = 0;
static int lib_pool_allocs = 0;

static bool ptr_in_lib_pool(const void *p) {
	return (const uint8_t *)p >= lib_pool
		&& (const uint8_t *)p < lib_pool + LIB_POOL_SIZE;
}

// Allocate a D/IRAM block from the exec heap: reject pure-IRAM blocks
// (no data alias) until the allocator falls through to real D/IRAM.
// Fallback for when the pool is full.
static void *diram_exec_malloc(uint32_t size) {
	void *ptr = NULL;
	void *rejects[8];
	int n_rej = 0;
	while (n_rej < 8) {
		ptr = heap_caps_malloc(size, MALLOC_CAP_EXEC | MALLOC_CAP_INTERNAL);
		if (!ptr || esp_ptr_in_diram_iram(ptr)) {
			break;
		}
		rejects[n_rej++] = ptr;
		ptr = NULL;
	}
	for (int i = 0; i < n_rej; i++) {
		heap_caps_free(rejects[i]);
	}
	return ptr;
}

static void lib_ram_free_one(void *p) {
	if (ptr_in_lib_pool(p)) {
		if (lib_pool_allocs > 0 && --lib_pool_allocs == 0) {
			lib_pool_used = 0;
		}
	} else {
		heap_caps_free(p);
	}
}
#else
static void lib_ram_free_one(void *p) {
	heap_caps_free(p);
}
#endif

__attribute__((section(".libif"))) static volatile union {
	vesc_c_if cif;
	char pad[2048];
} cif;

// The .libif section is placed at a fixed, target-specific address by
// main/linker_libif_<target>.ld so that native libs can find the interface
// table through the VESC_IF macro. Keep the heap allocator away from it.
SOC_RESERVE_MEMORY_REGION((intptr_t)&cif, (intptr_t)&cif + sizeof(cif), vesc_libif);

static bool lib_init_done = false;

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
	if (!utils_is_func_valid(func)) {
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

static void comm_can_transmit_sid_wrapper(
	uint32_t id, const uint8_t *data, uint8_t len
) {
	comm_can_transmit_sid(id, data, len);
}

static void comm_can_transmit_eid_wrapper(
	uint32_t id, const uint8_t *data, uint8_t len
) {
	comm_can_transmit_eid(id, data, len);
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

// App data handler wrapper
bool commands_set_app_data_handler_wrapper(
	void (*func)(unsigned char *data, unsigned int len)
) {
	void (*func_irom)(unsigned char *, unsigned int) = utils_drom_to_irom(func);
	return commands_set_app_data_handler(func_irom);
}
// CAN standard ID callback wrapper
void comm_can_set_sid_rx_callback_wrapper(
	bool (*p_func)(uint32_t id, uint8_t *data, uint8_t len)
) {
	bool (*p_func_irom)(uint32_t, uint8_t *, uint8_t) =
		utils_drom_to_irom(p_func);
	comm_can_set_sid_rx_callback(p_func_irom);
}

// CAN extended ID callback wrapper
void comm_can_set_eid_rx_callback_wrapper(
	bool (*p_func)(uint32_t id, uint8_t *data, uint8_t len)
) {
	bool (*p_func_irom)(uint32_t, uint8_t *, uint8_t) =
		utils_drom_to_irom(p_func);
	comm_can_set_eid_rx_callback(p_func_irom);
}

// Terminal callback removal wrapper
void terminal_unregister_callback_wrapper(
	void (*cbf)(int argc, const char **argv)
) {
	void (*cbf_irom)(int, const char **) = utils_drom_to_irom(cbf);
	terminal_unregister_callback(cbf_irom);
}

// IMU read callback wrapper
void imu_set_read_callback_wrapper(
	void (*func)(float *acc, float *gyro, float *mag, float dt)
) {
	void (*func_irom)(float *, float *, float *, float) =
		utils_drom_to_irom(func);
	imu_set_read_callback(func_irom);
}

// Reply function removal wrapper
void commands_unregister_reply_func_wrapper(
	void (*reply_func)(unsigned char *data, unsigned int len)
) {
	void (*reply_func_irom)(unsigned char *, unsigned int) =
		utils_drom_to_irom(reply_func);
	commands_unregister_reply_func(reply_func_irom);
}

// Terminal command registration wrapper (1 function pointer)
void terminal_register_command_callback_wrapper(
	const char *command, const char *help, const char *arg_names,
	void (*cbf)(int argc, const char **argv)
) {
	void (*cbf_irom)(int, const char **) = utils_drom_to_irom(cbf);
	terminal_register_command_callback(command, help, arg_names, cbf_irom);
}

// Packet init wrapper (2 function pointers)
void packet_init_wrapper(
	void (*s_func)(unsigned char *, unsigned int),
	void (*p_func)(unsigned char *, unsigned int), PACKET_STATE_t *state
) {
	void (*s_func_irom)(unsigned char *, unsigned int) =
		utils_drom_to_irom(s_func);
	void (*p_func_irom)(unsigned char *, unsigned int) =
		utils_drom_to_irom(p_func);
	packet_init(s_func_irom, p_func_irom, state);
}

// Custom config wrapper (3 function pointers)
void conf_custom_add_config_wrapper(
	int (*get_cfg)(uint8_t *data, bool is_default),
	bool (*set_cfg)(uint8_t *data), int (*get_cfg_xml)(uint8_t **data)
) {
	int (*get_cfg_irom)(uint8_t *, bool) = utils_drom_to_irom(get_cfg);
	bool (*set_cfg_irom)(uint8_t *)      = utils_drom_to_irom(set_cfg);
	int (*get_cfg_xml_irom)(uint8_t **)  = utils_drom_to_irom(get_cfg_xml);
	conf_custom_add_config(get_cfg_irom, set_cfg_irom, get_cfg_xml_irom);
}
void lispif_stop_lib(void) {
	// 1) Call stop_fun for all loaded libs (mirrors STM32)
	for (int i = 0; i < LIB_NUM_MAX; i++) {
		if (loaded_libs[i].stop_fun) {
			if (utils_is_func_valid(loaded_libs[i].stop_fun)) {
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
			lib_ram_free_one(lib_ram_alloc[i]);
			lib_ram_alloc[i] = NULL;
		}
	}
}

void commands_process_packet_wrapper(
	unsigned char *data, unsigned int len,
	void (*reply_func)(unsigned char *data, unsigned int len)
) {
	void (*reply_func_irom)(unsigned char *, unsigned int) =
		reply_func ? utils_drom_to_irom(reply_func) : NULL;
	commands_process_packet(data, len, reply_func_irom);
}

lbm_value ext_load_native_lib(lbm_value *args, lbm_uint argn) {
	lbm_value res = lbm_enc_sym(SYM_EERROR);

	// Expect a single numeric argument containing the IROM base address
	if (argn != 1 || !lbm_is_number(args[0])) {
		return res;
	}

	// The linker script and VESC_IF must agree on where the interface table
	// lives, otherwise libs would read garbage function pointers.
	if ((uintptr_t)&cif != (uintptr_t)VESC_IF) {
		lbm_set_error_reason("Native lib interface address mismatch (firmware bug)");
		return res;
	}

	if (!lib_init_done) {
		memset((char *)cif.pad, 0, 2048);
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

		// CAN
		cif.cif.can_set_sid_cb   = comm_can_set_sid_rx_callback_wrapper;
		cif.cif.can_set_eid_cb   = comm_can_set_eid_rx_callback_wrapper;
		cif.cif.can_transmit_sid = comm_can_transmit_sid_wrapper;
		cif.cif.can_transmit_eid = comm_can_transmit_eid_wrapper;
		cif.cif.can_send_buffer  = comm_can_send_buffer;
		cif.cif.can_set_duty     = comm_can_set_duty;
		cif.cif.can_set_current  = comm_can_set_current;
		cif.cif.can_set_current_off_delay = comm_can_set_current_off_delay;
		cif.cif.can_set_current_brake     = comm_can_set_current_brake;
		cif.cif.can_set_rpm               = comm_can_set_rpm;
		cif.cif.can_set_pos               = comm_can_set_pos;
		cif.cif.can_set_current_rel       = comm_can_set_current_rel;
		cif.cif.can_set_current_rel_off_delay =
			comm_can_set_current_rel_off_delay;
		cif.cif.can_set_current_brake_rel  = comm_can_set_current_brake_rel;
		cif.cif.can_ping                   = comm_can_ping;
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

		// Comm
		cif.cif.commands_process_packet = commands_process_packet_wrapper;
		cif.cif.send_app_data           = commands_send_app_data;
		cif.cif.set_app_data_handler    = commands_set_app_data_handler_wrapper;

		// Packets
		cif.cif.packet_init         = packet_init_wrapper;
		cif.cif.packet_reset        = packet_reset;
		cif.cif.packet_process_byte = packet_process_byte;
		cif.cif.packet_send_packet  = packet_send_packet;

		// IMU
		cif.cif.imu_startup_done        = imu_startup_done;
		cif.cif.imu_get_roll            = imu_get_roll;
		cif.cif.imu_get_pitch           = imu_get_pitch;
		cif.cif.imu_get_yaw             = imu_get_yaw;
		cif.cif.imu_get_rpy             = imu_get_rpy;
		cif.cif.imu_get_accel           = imu_get_accel;
		cif.cif.imu_get_gyro            = imu_get_gyro;
		cif.cif.imu_get_mag             = imu_get_mag;
		cif.cif.imu_derotate            = imu_derotate;
		cif.cif.imu_get_accel_derotated = imu_get_accel_derotated;
		cif.cif.imu_get_gyro_derotated  = imu_get_gyro_derotated;
		cif.cif.imu_get_quaternions     = imu_get_quaternions;
		cif.cif.imu_get_calibration     = imu_get_calibration;

		// EEPROM
		cif.cif.read_eeprom_var  = read_eeprom_var;
		cif.cif.store_eeprom_var = store_eeprom_var;

		// Terminal
		cif.cif.terminal_register_command_callback =
			terminal_register_command_callback_wrapper;
		cif.cif.terminal_unregister_callback =
			terminal_unregister_callback_wrapper;

		// Plot
		cif.cif.plot_init        = commands_init_plot;
		cif.cif.plot_add_graph   = commands_plot_add_graph;
		cif.cif.plot_set_graph   = commands_plot_set_graph;
		cif.cif.plot_send_points = commands_send_plot_points;

		// Custom config
		cif.cif.conf_custom_add_config    = conf_custom_add_config_wrapper;
		cif.cif.conf_custom_clear_configs = conf_custom_clear_configs;

		// Mutex
		cif.cif.mutex_create = lib_mutex_create;
		cif.cif.mutex_lock   = lib_mutex_lock;
		cif.cif.mutex_unlock = lib_mutex_unlock;

		// High resolution timer for short busy-wait sleeps and time measurement TODO
		cif.cif.timer_time_now              = lib_timer_time_now;
		cif.cif.timer_seconds_elapsed_since = lib_timer_seconds_elapsed_since;
		cif.cif.timer_sleep                 = lib_timer_sleep;

		// System lock (with counting)
		cif.cif.sys_lock   = utils_sys_lock_cnt;
		cif.cif.sys_unlock = utils_sys_unlock_cnt;

		cif.cif.commands_unregister_reply_func =
			commands_unregister_reply_func_wrapper;

		// IMU AHRS functions and read callback
		cif.cif.imu_set_read_callback   = imu_set_read_callback_wrapper;
		cif.cif.ahrs_init_attitude_info = ahrs_init_attitude_info;
		cif.cif.ahrs_update_initial_orientation =
			ahrs_update_initial_orientation;
		cif.cif.ahrs_update_mahony_imu   = ahrs_update_mahony_imu;
		cif.cif.ahrs_update_madgwick_imu = ahrs_update_madgwick_imu;
		cif.cif.ahrs_get_roll            = ahrs_get_roll;
		cif.cif.ahrs_get_pitch           = ahrs_get_pitch;
		cif.cif.ahrs_get_yaw             = ahrs_get_yaw;

		// Store backup data
		cif.cif.store_backup_data = main_store_backup_data;

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

		cif.cif.rgbled_init   = rgbled_init;
		cif.cif.rgbled_deinit = rgbled_deinit;
		cif.cif.rgbled_update = rgbled_update;
		cif.cif.aes_ctr_crypt = aes_ctr_crypt_inplace;

		lib_init_done = true;
	}

	// Read IROM header base directly
	uint32_t irom_base = lbm_dec_as_u32(args[0]);
	//commands_printf_lisp("=== LOAD ATTEMPT: irom=0x%08X ===", irom_base);

	// Basic pointer/alignment sanity
	if (irom_base == 0 || (irom_base & 0x3) != 0) {
		lbm_set_error_reason("Invalid IROM base pointer");
		return res;
	}

	// Validate native header magic. Read through the DROM alias, as data
	// reads through the instruction bus fault on some targets.
	const uint8_t *container_drom = utils_irom_to_drom((void *)irom_base);
	uint32_t magic_be = 0;
	memcpy(&magic_be, container_drom, sizeof(magic_be));

	bool is_reloc = magic_be == __builtin_bswap32(NATIVE_LIB_RELOC_MAGIC);
	if (!is_reloc && magic_be != __builtin_bswap32(NATIVE_LIB_MAGIC)) {
		lbm_set_error_reason("Magic number not found at IROM address");
		return res;
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
		// container carries a relocation table and the image is copied to
		// executable RAM and patched with its load address. Container
		// layout after the magic: image_size, entry_offset, reloc_count
		// (all LE u32), relocs[], image[].
		uint32_t image_size, entry_offset, reloc_count;
		memcpy(&image_size, container_drom + 4, 4);
		memcpy(&entry_offset, container_drom + 8, 4);
		memcpy(&reloc_count, container_drom + 12, 4);

		if (image_size < 12 || image_size > 0x80000 || (image_size & 3)
			|| entry_offset < 8 || entry_offset >= image_size
			|| (entry_offset & 3) || reloc_count > image_size / 4) {
			lbm_set_error_reason("Invalid native lib container");
			return res;
		}

		// The image needs RAM that is byte-accessible through the DRAM
		// alias (strings/data reads) AND executable through the IRAM
		// alias, i.e. real D/IRAM. Serve it from the boot-time pool when
		// possible - the internal heap is usually too fragmented for a
		// contiguous block by the time lisp code runs - and fall back to
		// a D/IRAM heap allocation otherwise.
		uint8_t *img_dram = NULL;
		void *alloc_ptr = NULL;

		uint32_t aligned_size = (image_size + 7) & ~7u;
		if (lib_pool && lib_pool_used + aligned_size <= LIB_POOL_SIZE) {
			img_dram = lib_pool + lib_pool_used;
			lib_pool_used += aligned_size;
			lib_pool_allocs++;
			alloc_ptr = img_dram;
		} else {
			void *heap_iram = diram_exec_malloc(image_size);
			if (heap_iram) {
				img_dram = (uint8_t *)MAP_IRAM_TO_DRAM((uint32_t)heap_iram);
				alloc_ptr = heap_iram;
			}
		}

		if (!img_dram) {
			static char err_buf[96];
			snprintf(err_buf, sizeof(err_buf),
				"Out of D/IRAM for lib: need %u, pool free %u, largest exec free %u",
				(unsigned)image_size,
				(unsigned)(lib_pool ? LIB_POOL_SIZE - lib_pool_used : 0),
				(unsigned)heap_caps_get_largest_free_block(
					MALLOC_CAP_EXEC | MALLOC_CAP_INTERNAL));
			lbm_set_error_reason(err_buf);
			return res;
		}

		uint8_t *img_iram =
			(uint8_t *)MAP_DRAM_TO_IRAM((uint32_t)img_dram);

		const uint8_t *relocs = container_drom + 16;
		memcpy(img_dram, relocs + reloc_count * 4, image_size);

		// Patch every absolute word with the load address: pointers into
		// executable sections get the IRAM alias, everything else the
		// DRAM alias. The image starts with the magic word, so ELF
		// address 0 lives at image offset 4.
		bool patch_ok = true;
		for (uint32_t r = 0; r < reloc_count; r++) {
			uint32_t e;
			memcpy(&e, relocs + r * 4, 4);
			uint32_t off = e & 0x7FFFFFFF;
			if (off < 4 || (off & 3) || off + 4 > image_size) {
				patch_ok = false;
				break;
			}
			uint32_t word;
			memcpy(&word, img_dram + off, 4);
			word += (uint32_t)((e & 0x80000000) ? img_iram : img_dram) + 4;
			memcpy(img_dram + off, &word, 4);
		}

		uint32_t inner_magic = 0;
		memcpy(&inner_magic, img_dram, 4);
		if (!patch_ok || inner_magic != __builtin_bswap32(NATIVE_LIB_MAGIC)) {
			lib_ram_free_one(alloc_ptr);
			lbm_set_error_reason("Invalid relocation table in native lib");
			return res;
		}

		// Make the copied and patched code visible to instruction fetch.
		__asm__ __volatile__("memw\n\tisync\n\t" ::: "memory");

		base_addr  = (uint32_t)img_dram;
		entry_addr = (uint32_t)img_iram + entry_offset;
		ram_alloc  = alloc_ptr;
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

	//commands_printf_lisp("Calling init function at 0x%08X", entry_addr);
	bool ok = ((bool (*)(lib_info *))entry_addr)(&loaded_libs[slot]);

	if (ok && loaded_libs[slot].stop_fun != NULL) {
		void *stop_fun_irom = utils_drom_to_irom(loaded_libs[slot].stop_fun);
		if (utils_is_func_valid(stop_fun_irom)) {
			loaded_libs[slot].stop_fun = stop_fun_irom;
			//commands_printf_lisp("=== LOAD SUCCESS ===");
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
		lib_ram_free_one(lib_ram_alloc[slot]);
		lib_ram_alloc[slot] = NULL;
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
			if (utils_is_func_valid(loaded_libs[i].stop_fun)) {
				loaded_libs[i].stop_fun(loaded_libs[i].arg);
			}
			loaded_libs[i].stop_fun  = NULL;
			loaded_libs[i].base_addr = 0;
			loaded_libs[i].arg       = NULL;
			lib_flash_addr[i]        = 0;
			if (lib_ram_alloc[i]) {
				lib_ram_free_one(lib_ram_alloc[i]);
				lib_ram_alloc[i] = NULL;
			}

			//commands_printf_lisp("Library at 0x%08X unloaded", irom_base);
			return lbm_enc_sym(SYM_TRUE);
		}
	}

	lbm_set_error_reason("Library not loaded");
	return res;
}