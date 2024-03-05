/*
	Copyright 2023 Syler Clayton    syler.clayton@gmail.com

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

#include "pubremote.h"
#include "buffer.h"
#include "utils.h"
#include "datatypes.h"
#include "comm_can.h"
#include "commands.h"
#include "main.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "commands.h"

#include <string.h>
#include <math.h>

// Private variables
static bool has_started;
static send_func_t fw_send_func_old;
static volatile bool fw_reply_rx = false;
static volatile bool fw_reply_ok = false;


static void fw_reply_func(unsigned char *data, unsigned int len) {
	if (len < 2) {
		return;
	}
    commands_printf("wut");

	fw_reply_rx = true;

	COMM_PACKET_ID packet_id;

	packet_id = data[0];
	data++;
	len--;

	switch (packet_id) {
	case COMM_CUSTOM_APP_DATA:
		fw_reply_ok = data[0];
		break;

	default:
		return;
	}

	commands_set_send_func(fw_send_func_old);
}


static void pubremote_task(void *arg) {
	for (;;) {
        commands_printf("lolpub");
        //TODO: use comm_can to GET_CUSTOM_APP_DATA from float package. Figure out how to use ESP-Now and make sure it doesn't conflict with Lisp functions for it. Implement a json parser for bi-directional communication over esp-now.
     int can_id = -1;
	can_id = 39;
	if (can_id > 254) {
			//return ENC_SYM_TERROR;
	}
    uint8_t buf[6];
	int32_t ind = 0;

	//if (can_id >= 0) {
	//	buf[ind++] = COMM_FORWARD_CAN;
	//	buf[ind++] = can_id;
	//}

	buf[ind++] = COMM_CUSTOM_APP_DATA;
    buf[ind++]= 101;
    buf[ind++]= 0;

	//fw_reply_rx = false;
	//fw_reply_ok = false;
	//fw_send_func_old = commands_get_send_func();
    //this does not work??? maybe if semephore is used? idk cuz then you could be missing packets intended for other thread??
    //commands_process_packet(buf, ind, fw_reply_func);

    //this shows up on the thor
    comm_can_send_buffer(can_id,buf,ind,0);

        //How fast to run the loop??
		vTaskDelay(5000);
	}
}

void pubremote_init(void) {
    //initalize vars
    has_started = false;

    // //spawn new task
    // //TODO: Figure out what affinity and such to configure
    //xTaskCreatePinnedToCore(pubremote_task, "pubremote_task", 1024, NULL, configMAX_PRIORITIES - 1, NULL, tskNO_AFFINITY);
    xTaskCreatePinnedToCore(pubremote_task, "pubremote_task", 1024, NULL, 6, NULL, tskNO_AFFINITY);
    commands_printf("PubRemote Initalized");
    has_started = true;
}

bool pubremote_started()
{
    return has_started;
}