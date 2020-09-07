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

#include <cr_section_macros.h>
#include "FreeRTOS.h"
#include "semphr.h"
#include <cr_section_macros.h>
#include "LpcUart.h"
#include "gparse.h"
#include "string.h"
#include "ITM_write.h"
#include "heap_lock_monitor.h"
#include "user_vcom.h"
#include "EEPROM.h"

// defines whether to use debuguart or CDC for the uart connection.
//#define USECDC
// TODO: insert other include files here

// TODO: insert other definitions and declarations here

extern "C" {

void vConfigureTimerForRunTimeStats( void ) {
	Chip_SCT_Init(LPC_SCTSMALL1);
	LPC_SCTSMALL1->CONFIG = SCT_CONFIG_32BIT_COUNTER;
	LPC_SCTSMALL1->CTRL_U = SCT_CTRL_PRE_L(255) | SCT_CTRL_CLRCTR_L; // set prescaler to 256 (255 + 1), and start timer
}

}

static void vGCode( const char * gcode);

SemaphoreHandle_t xUARTMutex;

LpcUart * uart;

EEPROM * EEProm;

// more useful for parse output than input..
//QueueHandle_t xParseQueue;


// just a function to putput the OK message for MDRAW as it's used for multiple responses.
void printOK()
{
	char okmsg[]="OK\r\n";
#ifdef USECDC
	USB_send((uint8_t *)okmsg, strlen(okmsg));
#else
	xSemaphoreTake(xUARTMutex, portMAX_DELAY);
	uart->write(okmsg);
	xSemaphoreGive(xUARTMutex);
#endif
	ITM_write(okmsg);
}

#define INPUTMAXLEN (80)

#ifdef USECDC
static void vUSBInput(void *pvParameters)
{
	vTaskDelay(100);

	char inputstr[INPUTMAXLEN+1] = "";
	uint8_t inputpos = 0;

	char rcv[RCV_BUFSIZE+1] = "";
	uint32_t rcvlen = 0;
	uint32_t rcvpos = 0;

	while (1) {
		// just to be on safe side then.

		if ( rcvpos == 0 ) // run out of existing input to process, get more.
		{
		    rcvlen = USB_receive((uint8_t *)rcv, RCV_BUFSIZE); // this already blocks.
			rcv[rcvlen] = '\0';

			USB_send((uint8_t *)rcv, rcvlen);
		}

		bool endline = false;

		uint8_t copylen=0;

		for (;rcvpos+copylen < rcvlen;copylen++)
		{
			if (rcv[rcvpos+copylen] == '\n' || rcv[rcvpos+copylen] == '\r')
			{
				endline = true;
				copylen++; // we've reached end of string, add one more so we copy the end too.
				break;
			}
		};

		if ( rcvpos+copylen < rcvlen && copylen > 0)
		{
			endline = true;// there was an EOL received
		}

		uint8_t curlen = strlen(inputstr);

		if ( curlen + copylen > INPUTMAXLEN ) copylen = INPUTMAXLEN-curlen; // ensure input buffer doesn't go over max specified 60.

		strncat(inputstr, &rcv[rcvpos], copylen);

		rcvpos+=copylen;

		if ( copylen != 0 )
		{
			inputpos+=copylen;
		}

		if ( rcvpos >= rcvlen )
		{
			rcvpos = 0;
		}

		if( inputpos >= INPUTMAXLEN || endline)
		{

			// check valid end of line, if so process through.

			// echo back
			USB_send((uint8_t *)inputstr, strlen(inputstr));
			USB_send((uint8_t *)"\r\n", 2);

//			if ( inputstr[strlen(inputstr)-1] == '\n' || inputstr[strlen(inputstr)-1] == '\r' )
//			{
//				inputstr[strlen(inputstr)-1] = 0; // remove EOL terminator.
//			}

			if ( strlen(inputstr) == 0) // if we now have empty string don't process further.sa=:
			{
				inputpos=0;
				inputstr[0] = 0;
				continue;
			}

			vGCode(inputstr);

//			xQueueSendToBack( xParseQueue, inputstr, portMAX_DELAY );

			inputpos=0;
			inputstr[0] = 0;

		}
	}
}
#else

static void vUARTInTask(void *pvParameters)
{
	uint8_t charcount = 0;

	char inputstr[81] = { 0 }; // shouldn't ever be a line longer than this from mdraw

	int inputpos = 0;

	ITM_write("Starting UART\r\n");

	while (1) {
		// just to be on safe side then.
		char ch = 0;
		int read = uart->readblock(ch); // block was hanging unexpectedly.

		// didn't receive a char, skip.
		if ( read == 0 )
		{
			taskYIELD();
			continue; // nothing to do this loop, return to start.
		}

		inputpos += read;

		bool endline = false;

		// not received end of line string, add character to input buffer.
		if ( !( ch == '\n' || ch == '\r') )
		{
#ifndef FULLECHO
			xSemaphoreTake(xUARTMutex, portMAX_DELAY);
			uart->write(ch);
			xSemaphoreGive(xUARTMutex);
#endif
			inputstr[charcount] = ch;
			inputstr[charcount+1] = 0;
			++charcount;
		} else endline = true;

		if ( charcount == 80 || endline )
		{

			xSemaphoreTake(xUARTMutex, portMAX_DELAY);
#ifdef FULLECHO
			uart->write(inputstr);
#endif
			uart->write("\r\n");
			xSemaphoreGive(xUARTMutex);

//			ITM_write(inputstr);
//			ITM_write("\r\n");

			// only bother passing input if we didn't get an empty line.
			if ( inputstr[0] != 0 )
				vGCode(inputstr);
//				xQueueSendToBack( xParseQueue, inputstr, portMAX_DELAY );

			charcount = 0;
			inputstr[0] = 0;
		}
	}
}
#endif


void printGCode( const char * output )
{
#ifdef USECDC
	USB_send((uint8_t *)output, strlen(output));
#else
	xSemaphoreTake(xUARTMutex, portMAX_DELAY);
	uart->write(output);
	xSemaphoreGive(xUARTMutex);
#endif
	ITM_write(output);
}


char numstrint[11];

char * numstr( uint32_t num )
{
		uint8_t i = 0;
		uint32_t temp=num;
		  for(i=1; i<=11; i++)
		  {
			  numstrint[11-i] = (uint8_t) ((temp % 10UL) + '0');
		    temp/=10;
		  }

		  numstrint[i-1] = '\0';

		for ( i=0;i<10&&numstrint[i]=='0';i++);

		return &numstrint[i];
}

// was previously setup as a seperate task, but doesn't seem much need for that
static void vGCode( const char * input) {

	char gcode[INPUTMAXLEN+1] = "";

//	while (1)
	{
//		xQueueReceive(xParseQueue, gcode, portMAX_DELAY);

		ITM_write(input);
		uint32_t starttick = DWT->CYCCNT;
	    command parsed = GCodeParser(input);
	    uint32_t ticktime = DWT->CYCCNT - starttick;
#ifdef USESPRINT
	    snprintf(gcode, 79, "Gcode parse took %ld cycles\r\n", ticktime);
#else
	    strcpy(gcode, "Gcode parse took ");
	    strcat(gcode, numstr( ticktime ));
	    strcat(gcode, " cycles\n\r");
#endif
		ITM_write(gcode);

	    switch ( parsed.cmd )
	    {
// messages expecting explicit reply rather than OK, or data saving.
			case init:

// saves a bit of stack space not using full sprintf formatting function.
//	Harder work to write output though, but as this is only complex output needed...
#ifndef USESPRINT
			    strcpy(gcode, "M10 XY ");
			    strcat(gcode, numstr( EEProm->getXSize() ));
			    strcat(gcode, " ");
			    strcat(gcode, numstr( EEProm->getYSize() ));
			    strcat(gcode, " 0.00 0.00 A");
			    strcat(gcode, numstr( EEProm->getXDir() ));
			    strcat(gcode, " B");
			    strcat(gcode, numstr( EEProm->getYDir() ));
			    strcat(gcode, " H0 S");
			    strcat(gcode, numstr( EEProm->getSpeed() ));
			    strcat(gcode, " U");
			    strcat(gcode, numstr( EEProm->getPUp() ));
			    strcat(gcode, " D");
				strcat(gcode, numstr( EEProm->getPDown() ));
				strcat(gcode, " \r\n");
#else
				snprintf(gcode, 79,  "M10 XY %ld %ld 0.00 0.00 A%d B%d H0 S%d U%d D%d\r\n",
						EEProm->getXSize(),
						EEProm->getYSize(),
						EEProm->getXDir(),
						EEProm->getYDir(),
						EEProm->getSpeed(),
						EEProm->getPUp(),
						EEProm->getPDown()
						);
#endif
				printGCode(gcode);
				printOK();
				break;
			case limit:
				printGCode("M11 1 1 1 1\r\n"); // fake limit switch reporting for now..
				printOK();
				break;
			case savepen: // request to save new pen up/down positions.
				EEProm->setPen(parsed.penstore.up, parsed.penstore.down);
				ITM_write("Pen data saved.\n");
				printOK();
				break;

			case savestepper: // request to save new drawing area sizes/speed/etc.
				EEProm->setStepper(parsed.stepper.A,
						parsed.stepper.B,
						parsed.stepper.width,
						parsed.stepper.height,
						parsed.stepper.speed);
				ITM_write("Stepper data saved.\n");
				printOK();
				break;

				// the OK messages.

			case setpen:
			case setlaser:
			case origin:
			case goxy: printOK(); break;

			case bad:
			case none:
			default:
				break;

	    };
	    vTaskDelay(10); // add in a small artificial delay to pretend doing something;
	}
}


int main(void) {

#if defined (__USE_LPCOPEN)
    // Read clock settings and update SystemCoreClock variable
    SystemCoreClockUpdate();
#if !defined(NO_BOARD_LIB)
    // Set up and initialize all required blocks and
    // functions related to the board hardware

	heap_monitor_setup();

    Board_Init();
    // Set the LED to the state of "On"
    Board_LED_Set(0, true);
#endif
#endif

    // TODO: insert code here

    // Force the counter to be placed into memory
    volatile static int i = 0 ;

	ITM_init();

	ITM_write("Boot\r\n");

	LpcPinMap none = {-1, -1}; // unused pin has negative values in it
	LpcPinMap txpin = { 0, 18 }; // transmit pin that goes to debugger's UART->USB converter
	LpcPinMap rxpin = { 0, 13 }; // receive pin that goes to debugger's UART->USB converter
	LpcUartConfig cfg = { LPC_USART0, 115200, UART_CFG_DATALEN_8 | UART_CFG_PARITY_NONE | UART_CFG_STOPLEN_1, false, txpin, rxpin, none, none };
	uart = new LpcUart(cfg);

	EEProm = new EEPROM();


    // TODO: insert code here

	xUARTMutex = xSemaphoreCreateMutex();

 //   xParseQueue = xQueueCreate( 5, sizeof( char[INPUTMAXLEN+1] ) );
 //   vQueueAddToRegistry( xParseQueue, "Parser input Queue" );

#ifdef USECDC
	/* low level USB communicationthread */
	xTaskCreate(cdc_task, "CDC",
				100, NULL, (tskIDLE_PRIORITY + 2UL), // 87
				(TaskHandle_t *) NULL);
	// higher level usb uart input
	xTaskCreate(vUSBInput, "USB In",
				200, NULL, (tskIDLE_PRIORITY + 3UL),
				(TaskHandle_t *) NULL);
#else
	xTaskCreate(vUARTInTask, "vUARTTask",
				200, NULL, (tskIDLE_PRIORITY + 3UL),
				(TaskHandle_t *) NULL);
#endif

/*
	xTaskCreate(vGCode, "GCode Parser",
				200, NULL, (tskIDLE_PRIORITY + 3UL),
				(TaskHandle_t *) NULL);
*/

	vTaskStartScheduler();

    // Enter an infinite loop, just incrementing a counter
    while(1) {
        i++ ;
        // "Dummy" NOP to allow source level single
        // stepping of tight while() loop
        __asm volatile ("nop");
    }
    return 0 ;
}
