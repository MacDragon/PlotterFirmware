/*
===============================================================================
 Name        : main.c
 Author      : $(author)
 Version     :
 Copyright   : $(copyright)
 Description : main definition
===============================================================================
*/

#if defined (__USE_LPCOPEN)
#if defined(NO_BOARD_LIB)
#include "chip.h"
#else
#include "board.h"
#endif
#endif


#include "FreeRTOS.h"
#include "semphr.h"
#include <cr_section_macros.h>
#include "LpcUart.h"
#include "gparse.h"

// TODO: insert other include files here

// TODO: insert other definitions and declarations here

SemaphoreHandle_t xUARTMutex;

LpcUart * uart;

static void vUARTInTask(void *pvParameters)
{
	uint8_t charcount = 0;

	char inputstr[81] = { 0 }; // shouldn't ever be a line longer than this from mdraw

	int inputpos = 0;

	while (1) {
		// just to be on safe side then.
		char ch = 0;
		xSemaphoreTake(xUARTMutex, portMAX_DELAY);
		int read = uart->read(ch);
		xSemaphoreGive(xUARTMutex);

		// didn't receive a char, skip.
		if ( ch == 0 )
		{
			taskYIELD();
			continue; // nothing to do this loop, return to start.
		}

		inputpos += read;

		bool endline = false;

		// not received end of line string, add character to input buffer.
		if ( !( ch == '\n' || ch == '\r') )
		{
			inputstr[charcount] = ch;
			inputstr[charcount+1] = 0;
			++charcount;
		} else endline = true;

		if ( charcount == 80 || endline )
		{
		    command parsed = GCodeParser(inputstr);

		    if ( parsed.cmd != bad)
		    {
				xSemaphoreTake(xUARTMutex, portMAX_DELAY);
				uart->write("OK\r\n");
				xSemaphoreGive(xUARTMutex);

				// throw command into relevant queue here.
		    }

			charcount = 0;
			inputstr[0] = 0;
		}
	}
}


int main(void) {

#if defined (__USE_LPCOPEN)
    // Read clock settings and update SystemCoreClock variable
    SystemCoreClockUpdate();
#if !defined(NO_BOARD_LIB)
    // Set up and initialize all required blocks and
    // functions related to the board hardware
    Board_Init();
    // Set the LED to the state of "On"
    Board_LED_Set(0, true);
#endif
#endif

	LpcPinMap none = {-1, -1}; // unused pin has negative values in it
	LpcPinMap txpin = { 0, 18 }; // transmit pin that goes to debugger's UART->USB converter
	LpcPinMap rxpin = { 0, 13 }; // receive pin that goes to debugger's UART->USB converter
	LpcUartConfig cfg = { LPC_USART0, 115200, UART_CFG_DATALEN_8 | UART_CFG_PARITY_NONE | UART_CFG_STOPLEN_1, false, txpin, rxpin, none, none };
	uart = new LpcUart(cfg);

    // TODO: insert code here

	xUARTMutex = xSemaphoreCreateMutex();

	xTaskCreate(vUARTInTask, "vUARTTask",
				configMINIMAL_STACK_SIZE*3, NULL, (tskIDLE_PRIORITY + 8UL),
				(TaskHandle_t *) NULL);

	vTaskStartScheduler();


    // Enter an infinite loop, just incrementing a counter
    while(1) {
    }
    return 0 ;
}
