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

#include "comm_mqtt.h"

#include "freertos/FreeRTOS.h"
#include "esp_err.h"
#include "esp_event_base.h"
#include "mqtt_client.h"

// Callback registry, keyed by client handle. A single esp-mqtt event handler
// (mqtt_evt) is shared by all clients and dispatches to the matching callback.
#define MQTT_MAX_CLIENTS 8

typedef struct {
	esp_mqtt_client_handle_t client;
	comm_mqtt_cb_t cb;
	void *user;
} mqtt_binding;

static mqtt_binding s_bindings[MQTT_MAX_CLIENTS];
static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;

static int mqtt_map_event_id(int32_t id) {
	switch (id) {
		case MQTT_EVENT_CONNECTED:    return 0;
		case MQTT_EVENT_DISCONNECTED: return 1;
		case MQTT_EVENT_DATA:         return 2;
		case MQTT_EVENT_SUBSCRIBED:   return 3;
		case MQTT_EVENT_UNSUBSCRIBED: return 4;
		case MQTT_EVENT_PUBLISHED:    return 5;
		case MQTT_EVENT_ERROR:        return 6;
		default:                      return -1;
	}
}

// Shared esp-mqtt event handler. Looks up the callback for the client that
// fired the event, copies out cb/user under the lock, then invokes the
// callback outside the lock (it may do arbitrary work, e.g. fire lisp events).
static void mqtt_evt(void *args, esp_event_base_t base, int32_t event_id,
		void *event_data) {
	(void)args;
	(void)base;
	esp_mqtt_event_handle_t e = (esp_mqtt_event_handle_t)event_data;
	if (!e) {
		return;
	}

	comm_mqtt_cb_t cb = NULL;
	void *user = NULL;
	portENTER_CRITICAL(&s_mux);
	for (int i = 0; i < MQTT_MAX_CLIENTS; i++) {
		if (s_bindings[i].client == e->client) {
			cb   = s_bindings[i].cb;
			user = s_bindings[i].user;
			break;
		}
	}
	portEXIT_CRITICAL(&s_mux);

	if (!cb) {
		return;
	}

	comm_mqtt_event ev = {0};
	ev.event_id       = mqtt_map_event_id(e->event_id);
	ev.topic          = e->topic;
	ev.topic_len      = e->topic_len;
	ev.data           = e->data;
	ev.data_len       = e->data_len;
	ev.data_offset    = e->current_data_offset;
	ev.total_data_len = e->total_data_len;
	ev.msg_id         = e->msg_id;
	ev.qos            = e->qos;
	ev.retain         = e->retain;
	cb((void *)e->client, &ev, user);
}

static bool binding_add(esp_mqtt_client_handle_t client) {
	bool ok = false;
	portENTER_CRITICAL(&s_mux);
	for (int i = 0; i < MQTT_MAX_CLIENTS; i++) {
		if (s_bindings[i].client == NULL) {
			s_bindings[i].client = client;
			s_bindings[i].cb     = NULL;
			s_bindings[i].user   = NULL;
			ok = true;
			break;
		}
	}
	portEXIT_CRITICAL(&s_mux);
	return ok;
}

static void binding_remove(esp_mqtt_client_handle_t client) {
	portENTER_CRITICAL(&s_mux);
	for (int i = 0; i < MQTT_MAX_CLIENTS; i++) {
		if (s_bindings[i].client == client) {
			s_bindings[i].client = NULL;
			s_bindings[i].cb     = NULL;
			s_bindings[i].user   = NULL;
			break;
		}
	}
	portEXIT_CRITICAL(&s_mux);
}

void *comm_mqtt_create(const comm_mqtt_cfg *cfg) {
	if (!cfg) {
		return NULL;
	}

	esp_mqtt_client_config_t c = {0};
	if (cfg->uri) {
		c.broker.address.uri = cfg->uri;
	} else {
		c.broker.address.hostname  = cfg->host;
		c.broker.address.port      = (uint32_t)cfg->port;
		c.broker.address.transport = cfg->server_cert_pem
			? MQTT_TRANSPORT_OVER_SSL : MQTT_TRANSPORT_OVER_TCP;
	}
	if (cfg->server_cert_pem) {
		c.broker.verification.certificate = cfg->server_cert_pem;
	}
	c.credentials.username                = cfg->username;
	c.credentials.client_id               = cfg->client_id;
	c.credentials.authentication.password = cfg->password;
	if (cfg->keepalive) {
		c.session.keepalive = cfg->keepalive;
	}
	if (cfg->lwt_topic) {
		c.session.last_will.topic  = cfg->lwt_topic;
		c.session.last_will.msg    = cfg->lwt_msg;
		c.session.last_will.qos    = cfg->lwt_qos;
		c.session.last_will.retain = cfg->lwt_retain;
	}

	esp_mqtt_client_handle_t client = esp_mqtt_client_init(&c);
	if (!client) {
		return NULL;
	}

	// Register the binding before the handler so an early event still resolves
	// (to a NULL callback, i.e. a no-op) rather than being missed.
	if (!binding_add(client)) {
		esp_mqtt_client_destroy(client);
		return NULL;
	}
	esp_mqtt_client_register_event(client, MQTT_EVENT_ANY, mqtt_evt, NULL);

	return (void *)client;
}

void comm_mqtt_set_cb(void *client, comm_mqtt_cb_t cb, void *user) {
	if (!client) {
		return;
	}
	portENTER_CRITICAL(&s_mux);
	for (int i = 0; i < MQTT_MAX_CLIENTS; i++) {
		if (s_bindings[i].client == (esp_mqtt_client_handle_t)client) {
			s_bindings[i].cb   = cb;
			s_bindings[i].user = user;
			break;
		}
	}
	portEXIT_CRITICAL(&s_mux);
}

bool comm_mqtt_start(void *client) {
	return client
		&& esp_mqtt_client_start((esp_mqtt_client_handle_t)client) == ESP_OK;
}

bool comm_mqtt_stop(void *client) {
	return client
		&& esp_mqtt_client_stop((esp_mqtt_client_handle_t)client) == ESP_OK;
}

void comm_mqtt_destroy(void *client) {
	if (!client) {
		return;
	}
	esp_mqtt_client_destroy((esp_mqtt_client_handle_t)client);
	binding_remove((esp_mqtt_client_handle_t)client);
}

int comm_mqtt_publish(void *client, const char *topic, const uint8_t *data,
		int len, int qos, bool retain) {
	if (!client) {
		return -1;
	}
	return esp_mqtt_client_publish((esp_mqtt_client_handle_t)client, topic,
			(const char *)data, len, qos, retain ? 1 : 0);
}

int comm_mqtt_subscribe(void *client, const char *topic, int qos) {
	if (!client) {
		return -1;
	}
	return esp_mqtt_client_subscribe_single((esp_mqtt_client_handle_t)client,
			topic, qos);
}

int comm_mqtt_unsubscribe(void *client, const char *topic) {
	if (!client) {
		return -1;
	}
	return esp_mqtt_client_unsubscribe((esp_mqtt_client_handle_t)client, topic);
}
