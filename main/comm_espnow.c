/*
	Copyright 2026 The VESC Express contributors

	This file is part of the VESC firmware.

	The VESC firmware is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	The VESC firmware is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with this program.  If not, see <http://www.gnu.org/licenses/>.
	*/

#include "comm_espnow.h"

#if !CONFIG_IDF_TARGET_ESP32P4

#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "esp_idf_version.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_wifi.h"
#include "esp_now.h"

#include "main.h"
#include "rb.h"
#include "comm_wifi.h"

#define ESPNOW_MAX_LISTENERS 4

typedef struct {
	comm_espnow_recv_cb_t recv;
	comm_espnow_sent_cb_t sent;
	void *user;
	bool used;
} espnow_listener;

// One received frame, queued from the Wi-Fi recv callback to the dispatch task.
typedef struct {
	uint8_t *data;
	int len;
	uint8_t src[COMM_ESPNOW_ADDR_LEN];
	uint8_t des[COMM_ESPNOW_ADDR_LEN];
	int rssi;
} espnow_rx_item;

#define ESPNOW_RX_BUFFER_ELEMENTS 10

static bool s_initialized = false;
static rb_t s_rx_rb;
static espnow_rx_item s_rx_data[ESPNOW_RX_BUFFER_ELEMENTS];
static SemaphoreHandle_t s_rx_sem;

static espnow_listener s_listeners[ESPNOW_MAX_LISTENERS];
static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;

// Snapshot the listener table under the lock, then invoke callbacks outside it
// (they may flatten lisp values, allocate, etc.).
static void dispatch_recv(const espnow_rx_item *item) {
	espnow_listener snap[ESPNOW_MAX_LISTENERS];
	portENTER_CRITICAL(&s_mux);
	memcpy(snap, s_listeners, sizeof(snap));
	portEXIT_CRITICAL(&s_mux);

	for (int i = 0; i < ESPNOW_MAX_LISTENERS; i++) {
		if (snap[i].used && snap[i].recv) {
			snap[i].recv(item->src, item->des, item->data, item->len,
					item->rssi, snap[i].user);
		}
	}
}

static void dispatch_sent(const uint8_t *des, bool success) {
	espnow_listener snap[ESPNOW_MAX_LISTENERS];
	portENTER_CRITICAL(&s_mux);
	memcpy(snap, s_listeners, sizeof(snap));
	portEXIT_CRITICAL(&s_mux);

	for (int i = 0; i < ESPNOW_MAX_LISTENERS; i++) {
		if (snap[i].used && snap[i].sent) {
			snap[i].sent(des, success, snap[i].user);
		}
	}
}

static void espnow_rx_task(void *arg) {
	(void)arg;
	for (;;) {
		xSemaphoreTake(s_rx_sem, 10 / portTICK_PERIOD_MS);

		espnow_rx_item item;
		if (!rb_pop(&s_rx_rb, &item)) {
			continue;
		}

		dispatch_recv(&item);
		free(item.data);
	}
}

#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 5, 0)
static void espnow_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status) {
	dispatch_sent(mac_addr, status == ESP_NOW_SEND_SUCCESS);
}
#else
static void espnow_send_cb(const esp_now_send_info_t *tx_info, esp_now_send_status_t status) {
	dispatch_sent(tx_info ? tx_info->des_addr : NULL,
			status == ESP_NOW_SEND_SUCCESS);
}
#endif

static void espnow_recv_cb(const esp_now_recv_info_t *info, const uint8_t *data,
		int data_len) {
	espnow_rx_item item;
	item.data = malloc(data_len);
	if (!item.data) {
		return;
	}
	item.len = data_len;
	memcpy(item.data, data, data_len);
	memcpy(item.src, info->src_addr, COMM_ESPNOW_ADDR_LEN);
	memcpy(item.des, info->des_addr, COMM_ESPNOW_ADDR_LEN);
	item.rssi = info->rx_ctrl->rssi;

	if (rb_insert(&s_rx_rb, &item)) {
		xSemaphoreGive(s_rx_sem);
	} else {
		free(item.data);
	}
}

bool comm_espnow_start(void) {
	main_wait_until_init_done();

	if (backup.config.wifi_mode == WIFI_MODE_DISABLED && !s_initialized) {
		esp_netif_init();
		esp_event_loop_create_default();
		wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
		esp_wifi_init(&cfg);
		esp_wifi_set_storage(WIFI_STORAGE_RAM);
		esp_wifi_set_mode(WIFI_MODE_APSTA);

		if (backup.config.ble_mode == BLE_MODE_DISABLED) {
			esp_wifi_set_ps(WIFI_PS_NONE);
		}

		// The event handler allows some of the wifi-extensions to work.
		esp_event_handler_instance_t instance_any_id;
		esp_event_handler_instance_register(
				WIFI_EVENT,
				ESP_EVENT_ANY_ID,
				&comm_wifi_event_handler,
				NULL,
				&instance_any_id);

		// Enable FTM responder
		wifi_config_t wifi_config;
		memset(&wifi_config, 0, sizeof(wifi_config));
		esp_wifi_get_config(WIFI_IF_AP, &wifi_config);
		wifi_config.ap.ftm_responder = true;
		esp_wifi_set_config(WIFI_IF_AP, &wifi_config);

		esp_wifi_start();
	}

	if (!s_initialized) {
		if (esp_now_init() != ESP_OK) {
			return false;
		}

		s_rx_sem = xSemaphoreCreateBinary();
		rb_init(&s_rx_rb, s_rx_data, sizeof(espnow_rx_item),
				ESPNOW_RX_BUFFER_ELEMENTS);
		xTaskCreate(espnow_rx_task, "espnow_rx", 2048, NULL, 3, NULL);

		esp_now_register_send_cb(espnow_send_cb);
		esp_now_register_recv_cb(espnow_recv_cb);
		s_initialized = true;
	}

	return true;
}

bool comm_espnow_is_initialized(void) {
	return s_initialized;
}

bool comm_espnow_add_peer(const uint8_t *mac, int rate) {
	if (!s_initialized || !mac) {
		return false;
	}

	esp_now_peer_info_t peer;
	memset(&peer, 0, sizeof(peer));
	peer.channel = 0; // 0 = current channel (must match wifi channel when used)
	peer.ifidx   = ESP_IF_WIFI_AP;
	peer.encrypt = false;
	memcpy(peer.peer_addr, mac, COMM_ESPNOW_ADDR_LEN);

	esp_err_t res = esp_now_add_peer(&peer);

	if (rate >= 0) {
		esp_now_rate_config_t rate_cfg;
		rate_cfg.phymode = WIFI_PHY_MODE_HT20;
		rate_cfg.dcm     = false;
		rate_cfg.ersu    = false;
		rate_cfg.rate    = rate;
		esp_now_set_peer_rate_config(mac, &rate_cfg);
	}

	return res == ESP_OK || res == ESP_ERR_ESPNOW_EXIST;
}

bool comm_espnow_del_peer(const uint8_t *mac) {
	if (!s_initialized || !mac) {
		return false;
	}
	esp_err_t res = esp_now_del_peer(mac);
	return res == ESP_OK || res == ESP_ERR_ESPNOW_NOT_FOUND;
}

int comm_espnow_send(const uint8_t *mac, const uint8_t *data, size_t len) {
	if (!s_initialized) {
		return ESP_ERR_ESPNOW_NOT_INIT;
	}
	return esp_now_send(mac, data, len);
}

int comm_espnow_add_listener(comm_espnow_recv_cb_t recv,
		comm_espnow_sent_cb_t sent, void *user) {
	int id = -1;
	portENTER_CRITICAL(&s_mux);
	for (int i = 0; i < ESPNOW_MAX_LISTENERS; i++) {
		if (!s_listeners[i].used) {
			s_listeners[i].recv = recv;
			s_listeners[i].sent = sent;
			s_listeners[i].user = user;
			s_listeners[i].used = true;
			id = i;
			break;
		}
	}
	portEXIT_CRITICAL(&s_mux);
	return id;
}

void comm_espnow_remove_listener(int id) {
	if (id < 0 || id >= ESPNOW_MAX_LISTENERS) {
		return;
	}
	portENTER_CRITICAL(&s_mux);
	s_listeners[id].used = false;
	s_listeners[id].recv = NULL;
	s_listeners[id].sent = NULL;
	s_listeners[id].user = NULL;
	portEXIT_CRITICAL(&s_mux);
}

#else

bool comm_espnow_start(void) { return false; }
bool comm_espnow_is_initialized(void) { return false; }
bool comm_espnow_add_peer(const uint8_t *mac, int rate) { (void)mac; (void)rate; return false; }
bool comm_espnow_del_peer(const uint8_t *mac) { (void)mac; return false; }
int comm_espnow_send(const uint8_t *mac, const uint8_t *data, size_t len) { (void)mac; (void)data; (void)len; return -1; }
int comm_espnow_add_listener(comm_espnow_recv_cb_t recv, comm_espnow_sent_cb_t sent, void *user) { (void)recv; (void)sent; (void)user; return -1; }
void comm_espnow_remove_listener(int id) { (void)id; }

#endif /* !CONFIG_IDF_TARGET_ESP32P4 */
