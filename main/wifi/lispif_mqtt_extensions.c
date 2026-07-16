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

#include "lispif_mqtt_extensions.h"

#if !CONFIG_IDF_TARGET_ESP32P4

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "comm_mqtt.h"

#include "eval_cps.h"
#include "heap.h"
#include "lbm_defines.h"
#include "lbm_types.h"
#include "lbm_memory.h"
#include "lbm_flat_value.h"
#include "extensions.h"
#include "lbm_vesc_utils.h"

#include "lispif_events.h"

// The lisp MQTT extensions manage a single global esp-mqtt client. Incoming
// messages and connection state are delivered through the standard lisp event
// system (enable with event-enable): event-mqtt-connected,
// event-mqtt-disconnected and (event-mqtt-rx topic data).
static void *s_client = NULL;
static volatile bool s_connected = false;

// Deliver a payload-less event, e.g. (event-mqtt-connected).
static void send_event_sym(lbm_uint sym) {
	lbm_flat_value_t v;
	if (!lbm_start_flatten(&v, 30)) {
		return;
	}
	f_cons(&v);
	f_sym(&v, sym);
	f_sym(&v, SYM_NIL);
	lbm_finish_flatten(&v);
	if (!lbm_event(&v)) {
		lbm_free(v.buf);
	}
}

// Deliver (event-mqtt-rx topic data). topic/data are exact-length byte arrays
// (not null terminated), matching event-ble-rx and event-data-rx.
static void send_event_rx(const char *topic, int topic_len,
		const char *data, int data_len) {
	if (topic_len < 0) {
		topic_len = 0;
	}
	if (data_len < 0) {
		data_len = 0;
	}
	lbm_flat_value_t v;
	if (!lbm_start_flatten(&v, 60 + topic_len + data_len)) {
		return;
	}
	f_cons(&v);
	f_sym(&v, sym_event_mqtt_rx);
	f_cons(&v);
	f_lbm_array(&v, topic_len, (uint8_t *)(topic ? topic : ""));
	f_cons(&v);
	f_lbm_array(&v, data_len, (uint8_t *)(data ? data : ""));
	f_sym(&v, SYM_NIL);
	lbm_finish_flatten(&v);
	if (!lbm_event(&v)) {
		lbm_free(v.buf);
	}
}

// comm_mqtt callback. Runs on the mqtt task; lbm_event is cross-task safe.
// event_id: 0=CONNECTED 1=DISCONNECTED 2=DATA (see comm_mqtt.h).
static void mqtt_cb(void *client, const comm_mqtt_event *ev, void *user) {
	(void)client;
	(void)user;
	switch (ev->event_id) {
		case 0: // CONNECTED
			s_connected = true;
			if (event_mqtt_connected_en) {
				send_event_sym(sym_event_mqtt_connected);
			}
			break;

		case 1: // DISCONNECTED
			s_connected = false;
			if (event_mqtt_disconnected_en) {
				send_event_sym(sym_event_mqtt_disconnected);
			}
			break;

		case 2: // DATA
			if (event_mqtt_rx_en) {
				send_event_rx(ev->topic, ev->topic_len, ev->data, ev->data_len);
			}
			break;

		default:
			break;
	}
}

// (mqtt-connect uri [client-id] [user] [password] [keepalive])
// uri: "mqtt://host:1883" or "mqtts://host:8883" (mqtts uses the built-in CA
// bundle - use the native-lib interface for a custom server certificate).
// Returns t on success, or eerror. Replaces any existing client.
static lbm_value ext_mqtt_connect(lbm_value *args, lbm_uint argn) {
	if (argn < 1 || argn > 5) {
		return ENC_SYM_EERROR;
	}

	char *uri = lbm_dec_str(args[0]);
	if (!uri) {
		return ENC_SYM_TERROR;
	}

	char *client_id = (argn > 1) ? lbm_dec_str(args[1]) : NULL;
	char *user      = (argn > 2) ? lbm_dec_str(args[2]) : NULL;
	char *password  = (argn > 3) ? lbm_dec_str(args[3]) : NULL;
	int keepalive   = (argn > 4 && lbm_is_number(args[4]))
		? lbm_dec_as_i32(args[4]) : 0;

	// Tear down a previous client first so a reconnect is clean.
	if (s_client) {
		comm_mqtt_stop(s_client);
		comm_mqtt_destroy(s_client);
		s_client = NULL;
		s_connected = false;
	}

	comm_mqtt_cfg cfg = {
		.uri       = uri,
		.client_id = client_id,
		.username  = user,
		.password  = password,
		.keepalive = (keepalive > 0) ? keepalive : 0,
	};

	s_client = comm_mqtt_create(&cfg);
	if (!s_client) {
		lbm_set_error_reason("Failed to create MQTT client");
		return ENC_SYM_EERROR;
	}

	comm_mqtt_set_cb(s_client, mqtt_cb, NULL);

	if (!comm_mqtt_start(s_client)) {
		comm_mqtt_destroy(s_client);
		s_client = NULL;
		lbm_set_error_reason("Failed to start MQTT client");
		return ENC_SYM_EERROR;
	}

	return ENC_SYM_TRUE;
}

// (mqtt-disconnect) - stop and destroy the client. Returns t.
static lbm_value ext_mqtt_disconnect(lbm_value *args, lbm_uint argn) {
	(void)args;
	(void)argn;
	if (s_client) {
		comm_mqtt_stop(s_client);
		comm_mqtt_destroy(s_client);
		s_client = NULL;
	}
	s_connected = false;
	return ENC_SYM_TRUE;
}

// (mqtt-connected) - t if currently connected to the broker, else nil.
static lbm_value ext_mqtt_connected(lbm_value *args, lbm_uint argn) {
	(void)args;
	(void)argn;
	return (s_client && s_connected) ? ENC_SYM_TRUE : ENC_SYM_NIL;
}

// (mqtt-publish topic data [qos] [retain])
// data may be a string or byte array; a single trailing null (from a string
// literal) is dropped. Returns the message id (>= 0) or nil on failure.
static lbm_value ext_mqtt_publish(lbm_value *args, lbm_uint argn) {
	if (argn < 2 || argn > 4) {
		return ENC_SYM_EERROR;
	}
	if (!s_client) {
		lbm_set_error_reason("MQTT not connected");
		return ENC_SYM_EERROR;
	}

	char *topic = lbm_dec_str(args[0]);
	if (!topic || !lbm_is_array_r(args[1])) {
		return ENC_SYM_TERROR;
	}

	const lbm_array_header_t *array = lbm_dec_array_header(args[1]);
	if (!array || !array->data) {
		return ENC_SYM_TERROR;
	}
	const char *data = (const char *)array->data;
	int len = (int)array->size;
	if (len > 0 && data[len - 1] == '\0') {
		len--; // drop the string terminator
	}

	int qos    = (argn > 2 && lbm_is_number(args[2])) ? lbm_dec_as_i32(args[2]) : 0;
	int retain = (argn > 3 && lbm_is_number(args[3])) ? lbm_dec_as_i32(args[3]) : 0;

	int msg_id = comm_mqtt_publish(s_client, topic, (const uint8_t *)data, len,
			qos, retain != 0);
	return (msg_id < 0) ? ENC_SYM_NIL : lbm_enc_i(msg_id);
}

// (mqtt-subscribe topic [qos]) - returns the message id (>= 0) or nil.
static lbm_value ext_mqtt_subscribe(lbm_value *args, lbm_uint argn) {
	if (argn < 1 || argn > 2) {
		return ENC_SYM_EERROR;
	}
	if (!s_client) {
		lbm_set_error_reason("MQTT not connected");
		return ENC_SYM_EERROR;
	}

	char *topic = lbm_dec_str(args[0]);
	if (!topic) {
		return ENC_SYM_TERROR;
	}
	int qos = (argn > 1 && lbm_is_number(args[1])) ? lbm_dec_as_i32(args[1]) : 0;

	int msg_id = comm_mqtt_subscribe(s_client, topic, qos);
	return (msg_id < 0) ? ENC_SYM_NIL : lbm_enc_i(msg_id);
}

// (mqtt-unsubscribe topic) - returns the message id (>= 0) or nil.
static lbm_value ext_mqtt_unsubscribe(lbm_value *args, lbm_uint argn) {
	if (argn != 1) {
		return ENC_SYM_EERROR;
	}
	if (!s_client) {
		lbm_set_error_reason("MQTT not connected");
		return ENC_SYM_EERROR;
	}

	char *topic = lbm_dec_str(args[0]);
	if (!topic) {
		return ENC_SYM_TERROR;
	}

	int msg_id = comm_mqtt_unsubscribe(s_client, topic);
	return (msg_id < 0) ? ENC_SYM_NIL : lbm_enc_i(msg_id);
}

void lispif_load_mqtt_extensions(void) {
	lbm_add_extension("mqtt-connect", ext_mqtt_connect);
	lbm_add_extension("mqtt-disconnect", ext_mqtt_disconnect);
	lbm_add_extension("mqtt-connected", ext_mqtt_connected);
	lbm_add_extension("mqtt-publish", ext_mqtt_publish);
	lbm_add_extension("mqtt-subscribe", ext_mqtt_subscribe);
	lbm_add_extension("mqtt-unsubscribe", ext_mqtt_unsubscribe);
}

#else

void lispif_load_mqtt_extensions(void) {
}

#endif /* !CONFIG_IDF_TARGET_ESP32P4 */
