/*
 * DebugPrint.cpp
 *
 *  Created on: 29 Sep 2020
 *      Author: visa
 */

#include "DebugPrint.h"


QueueHandle_t xDebugQueue;

static LpcUart * debuguart;
static bool debugon = false;

static void vDebug(void *pvParameters) {
	char buffer[64];
	debugEvent msg;

	while (1) {
		xQueueReceive(xDebugQueue, &msg, portMAX_DELAY);
		snprintf(buffer, 64, msg.format, msg.data[0], msg.data[1], msg.data[2]);
		if ( debuguart != nullptr && debugon )
			debuguart->write(buffer);
//		uart->write("\r\n");

	}
}

void debug(const char *format, uint32_t d1, uint32_t d2, uint32_t d3)
{
	debugEvent msg;

	msg.format = format;
	msg.data[0] = d1;
	msg.data[1] = d2;
	msg.data[2] = d3;
	xQueueSendToBack(xDebugQueue, &msg, 0);
	// don't block for debug message, could alter program flow otherwise.
	// Ensure queue is big enough instead to try and not miss messages.
}

bool SetupDebugPrint( LpcUart * uartin ) {

	xDebugQueue = xQueueCreate( 50, sizeof( debugEvent ) );
    vQueueAddToRegistry( xDebugQueue, "Debug Queue" );

	debugon = true;
	if ( uartin == nullptr )
		debuguart = nullptr;
	else
		debuguart = uartin;

	xTaskCreate(vDebug, "Debug Output",
				200, (void *) xDebugQueue, (tskIDLE_PRIORITY + 1UL),
				(TaskHandle_t *) NULL);
	return true;
}

