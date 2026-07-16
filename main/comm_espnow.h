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

#ifndef MAIN_COMM_ESPNOW_H_
#define MAIN_COMM_ESPNOW_H_

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/*
 * Shared ESP-NOW wrapper. ESP-NOW allows exactly one global receive and one
 * global send callback, so this module owns them and fans out to any number of
 * registered listeners. Both the LispBM esp-now-* extensions and the native-
 * lib interface (VESC_IF->espnow_*) go through here so they can coexist.
 *
 * Received frames are dispatched from a dedicated task (not the Wi-Fi callback
 * context) so listeners may do heavier work such as flattening lisp values.
 */

#define COMM_ESPNOW_ADDR_LEN 6

// Called for each received frame, from the dispatch task. Pointers are valid
// only for the duration of the call.
typedef void (*comm_espnow_recv_cb_t)(const uint8_t *src, const uint8_t *des,
		const uint8_t *data, int len, int rssi, void *user);

// Called when a transmission completes, from the Wi-Fi callback context - keep
// it light. des may be NULL on IDF versions that don't report it.
typedef void (*comm_espnow_sent_cb_t)(const uint8_t *des, bool success,
		void *user);

// Bring up ESP-NOW (and Wi-Fi, if disabled). Idempotent; returns true if
// initialized (now or already).
bool comm_espnow_start(void);
bool comm_espnow_is_initialized(void);

// Add/remove a peer by 6-byte MAC. rate < 0 uses the default rate; otherwise
// the ESP-NOW rate index (see esp_now_rate_config_t). Returns success.
bool comm_espnow_add_peer(const uint8_t *mac, int rate);
bool comm_espnow_del_peer(const uint8_t *mac);

// Queue a frame to a peer MAC. Returns the esp_err_t (0 == ESP_OK). Delivery
// status is reported via the sent callback, not the return value.
int comm_espnow_send(const uint8_t *mac, const uint8_t *data, size_t len);

// Register a listener. Either callback may be NULL. Returns a listener id
// (>= 0) or -1 if the table is full.
int comm_espnow_add_listener(comm_espnow_recv_cb_t recv,
		comm_espnow_sent_cb_t sent, void *user);
void comm_espnow_remove_listener(int id);

#endif /* MAIN_COMM_ESPNOW_H_ */
