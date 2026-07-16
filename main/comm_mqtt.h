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

#ifndef MAIN_COMM_MQTT_H_
#define MAIN_COMM_MQTT_H_

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/*
 * Thin wrapper around esp-mqtt shared by the native-lib interface
 * (vesc_c_if mqtt_* slots) and the LispBM mqtt-* extensions. This is the
 * single place that touches esp-mqtt: config mapping, the event handler and
 * the client operations all live here so the two front-ends stay thin.
 */

// Compact MQTT client configuration. Set uri OR host+port. All strings are
// copied by the client, so they need not outlive the create call.
typedef struct {
	const char *uri;             // takes precedence over host/port when set
	const char *host;            // used when uri is NULL
	int port;
	const char *client_id;       // NULL = auto-generated
	const char *username;        // NULL = no authentication
	const char *password;
	int keepalive;               // seconds, 0 = default (120)
	const char *lwt_topic;       // last will and testament, NULL = none
	const char *lwt_msg;
	int lwt_qos;
	int lwt_retain;
	const char *server_cert_pem; // CA cert (PEM) for mqtts://, NULL = none
} comm_mqtt_cfg;

// MQTT event delivered to the registered callback. Pointers are valid only for
// the duration of the callback. Large payloads may arrive as several DATA
// events; use data_offset/total_data_len to reassemble.
typedef struct {
	int event_id;        // 0=CONNECTED 1=DISCONNECTED 2=DATA 3=SUBSCRIBED
	                     // 4=UNSUBSCRIBED 5=PUBLISHED 6=ERROR (-1=other)
	const char *topic;
	int topic_len;
	const char *data;
	int data_len;
	int data_offset;     // offset of this chunk within the full payload
	int total_data_len;  // full payload length
	int msg_id;
	int qos;
	bool retain;
} comm_mqtt_event;

// Callback invoked on the mqtt task for each event. user is the pointer passed
// to comm_mqtt_set_cb. client is the same handle comm_mqtt_create returned.
typedef void (*comm_mqtt_cb_t)(void *client, const comm_mqtt_event *ev, void *user);

// Create (but do not start) a client. Returns an opaque handle or NULL.
void *comm_mqtt_create(const comm_mqtt_cfg *cfg);

// Register the event callback for a client (replaces any previous one).
void comm_mqtt_set_cb(void *client, comm_mqtt_cb_t cb, void *user);

bool comm_mqtt_start(void *client);
bool comm_mqtt_stop(void *client);
void comm_mqtt_destroy(void *client); // stops, frees and forgets the client

// publish/subscribe/unsubscribe return the message id (>= 0) or -1 on failure.
int comm_mqtt_publish(void *client, const char *topic, const uint8_t *data,
		int len, int qos, bool retain);
int comm_mqtt_subscribe(void *client, const char *topic, int qos);
int comm_mqtt_unsubscribe(void *client, const char *topic);

#endif /* MAIN_COMM_MQTT_H_ */
