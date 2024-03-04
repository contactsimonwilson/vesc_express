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

#include <string.h>
#include <math.h>

// Private variables
static bool has_started;
static PUBREMOTE_MODE pubremote_mode;

static void pubremote_task(void *arg) {
	for (;;) {
        commands_printf("Hi from PubRemote loop");
        //TODO: use comm_can to GET_CUSTOM_APP_DATA from float package. Figure out how to use ESP-Now and make sure it doesn't conflict with Lisp functions for it. Implement a json parser for bi-directional communication over esp-now.


        //How fast to run the loop
		vTaskDelay(1000);
	}
}

void pubremote_init(void) {
    //initalize vars
    has_started = false;
    //spawn new task
    //TODO: Figure out what affinity and such to configure
    xTaskCreatePinnedToCore(pubremote_task, "pubremote_task", 1024, NULL, 6, NULL, tskNO_AFFINITY);
    commands_printf("PubRemote Initalized");
    has_started = true;
}

bool pubremote_started()
{
    return has_started;
}