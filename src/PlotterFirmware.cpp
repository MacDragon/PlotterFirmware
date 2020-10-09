/*
===============================================================================
 Name        : main.c
 Author      : $(author)
 Version     :
 Copyright   : $(copyright)
 Description : main definition
===============================================================================
*/

// define movement setup.
#define SPEEDSLOW  		  (1000)
#define SPEEDFAST  		  (3800) // faster was breaking sim on 1/8 step long moves..
#define SPEEDACCEL		  (2000) // acceleration in rpm/s^2
#define STEPS_PER_REV     (400)

#define COMMANDQ

#if defined (__USE_LPCOPEN)
#if defined(NO_BOARD_LIB)
#include "chip.h"
#else
#include "board.h"
#endif
#endif

#include "FreeRTOS.h"
#include "semphr.h"
#include "event_groups.h"
#include <cr_section_macros.h>
#include "LpcUart.h"
#include "string.h"
#include "ITM_write.h"
#include "heap_lock_monitor.h"
#include "user_vcom.h"
#include <string>

#include "gparse.h"
#include "EEPROM.h"
#include "helper.h"
#include "DrawControl.h"
#include "MotorXY.h"

#define ECHO

// defines whether to use debuguart or CDC for the uart connection.
#define CDC
// TODO: insert other include files here

// TODO: insert other definitions and declarations here

static void vGCode( const char * input);
void printGCode( const char * output );

extern "C" {

void vConfigureTimerForRunTimeStats( void ) {
	Chip_SCT_Init(LPC_SCTSMALL1);
	LPC_SCTSMALL1->CONFIG = SCT_CONFIG_32BIT_COUNTER;
	LPC_SCTSMALL1->CTRL_U = SCT_CTRL_PRE_L(255) | SCT_CTRL_CLRCTR_L; // set prescaler to 256 (255 + 1), and start timer
}

}

SemaphoreHandle_t xUARTMutex;
EventGroupHandle_t xInit;
SemaphoreHandle_t xAllowMotor;

LpcUart * uart;

EEPROM * EEProm;

MotorXY * XY;
DrawControl * Draw;


DigitalIoPin * Limit1; // 0_9
DigitalIoPin * Limit2; // 0_29
DigitalIoPin * Limit3; // 0_9
DigitalIoPin * Limit4; // 0_29

OutputPin * Laser; // 0_12
OutputPin * Pen; // 0_10

OutputPin * Red; // 0_25
OutputPin * Green; // 0_3
OutputPin * Blue; // 1_1

SemaphoreHandle_t xLimit;



// more useful for parse output than input..
QueueHandle_t xParseQueue;
QueueHandle_t xCommandQueue;
QueueHandle_t xLEDQ;

// just a function to putput the OK message for MDRAW as it's used for multiple responses.
void printOK()
{
	char okmsg[]="OK\r\n";
#ifdef CDC
	USB_send((uint8_t *)okmsg, strlen(okmsg));
#else
	uart->write(okmsg);
#endif
	ITM_write(okmsg);
}

#define INPUTMAXLEN (80)


void vInwrite(const char *str) {
#ifdef CDC

#else
	if ( uart != nullptr )
	{
		uart->write(str);
	}
#endif
}

static void vInput(void *pvParameters)
{
	vTaskDelay(100);

	xEventGroupWaitBits(xInit, 1, pdFALSE, pdTRUE, portMAX_DELAY);
	vInwrite("\r\nWaiting for Limit switch opening...\r\n");
	xSemaphoreTake(xAllowMotor, portMAX_DELAY);

	vInwrite("\r\nRunning plotter initilisation...\r\n");

	if ( !XY->trackinit() )
	{
		vInwrite("Failed to calibrate, halting stepper.");
		while ( 1 ) vTaskSuspend(NULL);
	}

	vInwrite("Track width detected: 0-");
	vInwrite(getnumstr(XY->getwidth()));
	vInwrite("steps\r\n");
	vInwrite("Track height detected: 0-");
	vInwrite(getnumstr(XY->getheight()));
	vInwrite("steps\r\n");
	vInwrite("Waiting for commands\r\n");

	xEventGroupSetBits(xInit,2);

	char inputstr[INPUTMAXLEN+1] = "";
	uint8_t inputpos = 0;

	char rcv[RCV_BUFSIZE+1] = "";
	uint32_t rcvlen = 0;
	uint32_t rcvpos = 0;

#ifndef CDC
	uart->read(rcv, RCV_BUFSIZE, 0); // clear buffer.
	rcv[0] = '\0';
#endif

	while (1) {
		// just to be on safe side then.

		if ( rcvpos == 0 ) // run out of existing input to process, get more.
		{
#ifdef CDC
		    rcvlen = USB_receive((uint8_t *)rcv, RCV_BUFSIZE); // this already blocks.
//			uart->write("In:");
//			uart->write(rcv);

#ifdef ECHO
			USB_send((uint8_t *)rcv, rcvlen);
			uart->write("Out:");
			uart->write(rcv);
			uart->write("\r\n");
#endif
#else
		    rcvlen =  uart->read(rcv, RCV_BUFSIZE, 20);
			rcv[rcvlen] = '\0';
			vInwrite(rcv);
#endif
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
			int32_t length = strlen(inputstr);

			for( int i=length; isspace(inputstr[i-1]);i--)
				inputstr[i-1]='\0';

			// check valid end of line, if so process through.

			// echo back
#ifdef CDC
#ifdef ECHO
			USB_send((uint8_t *)inputstr, strlen(inputstr));
			USB_send((uint8_t *)"\r\n", 2);
			uart->write("Out:");
			uart->write(inputstr);
			uart->write("\r\n");
#endif
#else
			uart->write(inputstr);
			uart->write("\r\n");
#endif

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

void printGCode( const char * output )
{
#ifdef CDC
	USB_send((uint8_t *)output, strlen(output));
#else
	xSemaphoreTake(xUARTMutex, portMAX_DELAY);
	uart->write(output);
	xSemaphoreGive(xUARTMutex);
#endif
	ITM_write(output);
}


static void vGCode( const char * input) {
//static void vGCode(void *pvParameters ){
	char gcode[INPUTMAXLEN+1] = "";

	xEventGroupWaitBits(xInit, 2, pdFALSE, pdTRUE, portMAX_DELAY);

	// scaling values by extra 100 allows integer division without losing accuracy to needed precision.
	int32_t xfact = ((EEProm->getXSize()*10000) / XY->getwidth());
	int32_t yfact = ((EEProm->getYSize()*10000) / XY->getheight());

	static int32xy_t mdrawpos={0,0};

	uint32_t starttime;
	uint32_t runtime;

//	while (1)
	{
//		xQueueReceive(xParseQueue, gcode, portMAX_DELAY);
		uart->write(gcode);
		uint32_t starttick = DWT->CYCCNT;
	    command parsed = GCodeParser(input);
	    uint32_t ticktime = DWT->CYCCNT - starttick;
	    snprintf(gcode, 79, " : parse took %ld cycles\r\n", ticktime);
	    uart->write(gcode);
	    switch ( parsed.cmd )
	    {
// messages expecting explicit reply rather than OK, or data saving.
			case init:
				snprintf(gcode, 79,  "M10 XY %ld %ld 0.00 0.00 A%d B%d H0 S%d U%d D%d\r\n",
						EEProm->getXSize(),
						EEProm->getYSize(),
						EEProm->getXDir(),
						EEProm->getYDir(),
						EEProm->getSpeed(),
						EEProm->getPUp(),
						EEProm->getPDown()
						);

				printGCode(gcode);
	//			printOK(); // no OK on M10?
				break;
			case limit:
				snprintf(gcode, 79,  "M11 %d %d %d %d\r\n",
						!Limit1->read(),
						!Limit2->read(),
						!Limit3->read(),
						!Limit4->read());
				printGCode(gcode);
	//			printOK(); // no OK on M11?
				break;
			case savepen: // request to save new pen up/down positions.
				EEProm->setPen(parsed.penstore.up, parsed.penstore.down);
				Draw->setPenUpDown(parsed.penstore.up, parsed.penstore.down);
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
				xfact = ((EEProm->getXSize()*10000) / XY->getwidth());
				yfact = ((EEProm->getYSize()*10000) / XY->getheight());

				 // work around that mdraw speed scale is 0-99% 1-100% makes more sense, and limiting values to 100% range.
				if ( parsed.stepper.speed <  0 ) parsed.stepper.speed = 1;
				else if ( parsed.stepper.speed >= 100 ) parsed.stepper.speed = 100;
				else if ( parsed.stepper.speed < 100 ) parsed.stepper.speed++;

				XY->setPPS(((SPEEDSLOW*100)*(parsed.stepper.speed*100)/100)/10000, SPEEDFAST );
				XY->setInvert((parsed.stepper.A==0)?false:true, (parsed.stepper.B==0)?false:true);

				printOK();
				break;
			case setpen:
#ifdef COMMANDQ
				xQueueSendToBack( xCommandQueue, &parsed, portMAX_DELAY );
#else
				Draw->setPen( parsed.pen.pos );
#endif
				printOK();
				break;
			case setlaser:
#ifdef COMMANDQ
				xQueueSendToBack( xCommandQueue, &parsed, portMAX_DELAY );
#else
				Draw->setLaser(parsed.laser.power );
#endif
				printOK();
				break;
			case origin:
				mdrawpos = {0,0};
#ifdef COMMANDQ
				parsed.cmd = goxy;
				parsed.pos.x = 0;
				parsed.pos.y = 0;
				parsed.pos.abs = 1;
				xQueueSendToBack( xCommandQueue, &parsed, portMAX_DELAY );
#else
				XY->gotoxy({0, 0}, true, 0, SPEEDACCEL);
#endif
				printOK();
				break;
			case goxy:

				// TODO multiple XY could be batched together for faster movement without decelerating between
				// if movement direction similar enough. -- would need quite a long queue and big changes.

				// likely not for this project

				if ( (parsed.pos.abs==0) )
				{
					mdrawpos = {(parsed.pos.x), (parsed.pos.y)};
				} else
				{
					mdrawpos += {(parsed.pos.x), (parsed.pos.y)};
				}
				// scale factor?

#ifdef COMMANDQ
				parsed.pos.x = (parsed.pos.x*100)/xfact;
				parsed.pos.y = (parsed.pos.y*100)/yfact;
				xQueueSendToBack( xCommandQueue, &parsed, portMAX_DELAY );
#else
				starttime = xTaskGetTickCount();
				XY->gotoxy({(parsed.pos.x*100)/xfact, (parsed.pos.y*100)/yfact}, true, 0, SPEEDACCEL); // always absolute value.
				runtime = xTaskGetTickCount()-starttime;
				snprintf(gcode, 79,  "Move took %ld ms\r\n",runtime );
				uart->write(gcode);
#endif
				printOK();
				break;
			case bad:
			case none:
			default:
				break;
	    };
	    vTaskDelay(1); // stop mdraw crashing on receiving OK too fast..
	}
}


#ifdef COMMANDQ
static void vPlot(void *pvParameters ){

	// scaling values by extra 100 allows integer division without losing accuracy to needed precision.

	uint32_t starttime;
	uint32_t runtime;

	command parsed;

	while (1)
	{
		xQueueReceive(xCommandQueue, &parsed, portMAX_DELAY);
	    switch ( parsed.cmd )
	    {
// messages expecting explicit reply rather than OK, or data saving.
			case init:
				break;
			case limit:
				break;
			case savepen: // request to save new pen up/down positions.
				break;

			case savestepper: // request to save new drawing area sizes/speed/etc.
				break;
			case setpen:
				Draw->setPen( parsed.pen.pos );
				break;
			case setlaser:
				Draw->setLaser(parsed.laser.power );
				break;
			case origin:
				XY->gotoxy({0, 0}, true, 0, SPEEDACCEL);
				break;
			case goxy:
				starttime = xTaskGetTickCount();
				XY->gotoxy({parsed.pos.x, parsed.pos.y}, true, 0, SPEEDACCEL); // always absolute value.
				runtime = xTaskGetTickCount()-starttime;
				char str[80];
				snprintf(str, 79,  "Move took %ld ms\r\n",runtime );
				uart->write(str);
				break;
			case bad:
			case none:
			default:
				break;

	    };
	}
}
#endif


static void vLED(void *pvParameters)
{
	vTaskDelay(100);

	uint32_t msg;

	bool holdingmutex = true;
	xSemaphoreTake(xAllowMotor, portMAX_DELAY);

//	setuppinint( );
	// ensure the semaphore isn't taken before limit task has it.
	xEventGroupSetBits(xInit,1);
	while ( holdingmutex )
	{
		if ( ( Limit1->read() || Limit2->read() || Limit3->read() || Limit4->read() ) )
		{
			if ( !holdingmutex)
			{
				xSemaphoreTake(xAllowMotor, portMAX_DELAY);
				holdingmutex=true;
			}
		} else // no limit switches active, hand along motor control mutex.
		{
			if ( holdingmutex )
			{
				holdingmutex=false;
				xSemaphoreGive(xAllowMotor);
			}
		}

		Blue->toggle();
		vTaskDelay(200);
	}

	Blue->write(false);
	Red->write(true); // indicate powered on.

	while (1) {

		if ( xQueueReceive(xLEDQ, &msg, 0)) // we've received led update.
		{
			if ( msg == 0)
			{
				Green->write(true);
				Red->write(false);
			}

			if ( msg == 1) // error state
			{
				Green->write(false);

				bool error = true;

				while ( error )
				{
					if ( xQueueReceive(xLEDQ, &msg, 0) )
					{
						if ( msg == 0 )
						{
							error = false;
							Red->write(false);
							Green->write(true);
							break;
						}
					}
					vTaskDelay(100);
					Red->toggle(); // init fail or other error.
				}
			}
		}

		if ( xSemaphoreTake(xLimit, 0) || Limit1->read() || Limit2->read() || Limit3->read()|| Limit4->read() )
		{
			Blue->setcounter(500);
		}
		else
		{
			Blue->checkcounter();
		}

		vTaskDelay(10);
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

	PinMap LaserP = { 0, 12 };
	PinMap PenP = { 0, 10 };

    Laser = new OutputPin(LaserP.port,LaserP.pin,false);
	Laser->write(false);

    // TODO: insert code here

    // Force the counter to be placed into memory
//    volatile static int i = 0 ;

	ITM_init();

	ITM_write("Boot\r\n");

	// initialize RIT (= enable clocking etc.)
	Chip_RIT_Init(LPC_RITIMER);
	// set the priority level of the interrupt
	// The level must be equal or lower than the maximum priority specified in FreeRTOS config
	// Note that in a Cortex-M3 a higher number indicates lower interrupt priority
	NVIC_SetPriority( RITIMER_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 1 );

	EEProm = new EEPROM();

	Red = new OutputPin(0,25,true);
	Green = new OutputPin(0,3,true);
	Blue = new OutputPin(1,1,true);

	xLimit = xSemaphoreCreateBinary();
	xAllowMotor = xSemaphoreCreateMutex();
	xInit = xEventGroupCreate();

	LpcPinMap none = {-1, -1}; // unused pin has negative values in it
	LpcPinMap txpin = { 0, 18 }; // transmit pin that goes to debugger's UART->USB converter
	LpcPinMap rxpin = { 0, 13 }; // receive pin that goes to debugger's UART->USB converter
	LpcUartConfig cfg = { LPC_USART0, 115200, UART_CFG_DATALEN_8 | UART_CFG_PARITY_NONE | UART_CFG_STOPLEN_1, false, txpin, rxpin, none, none };
	uart = new LpcUart(cfg);

	PinMap Lim1P = { 1, 3 };
	PinMap Lim2P = { 0, 0 };
	PinMap Lim3P = { 0, 9 };
	PinMap Lim4P = { 0, 29 };

	PinMap MotorX = { 0, 24 };
	PinMap DirX = { 1, 0 };

	PinMap MotorY = { 0, 27 };
	PinMap DirY = { 0, 28 };

	Limit1 = new DigitalIoPin(Lim1P.port,Lim1P.pin,true, true, true);
	Limit2 = new DigitalIoPin(Lim2P.port,Lim2P.pin,true, true, true);
	Limit3 = new DigitalIoPin(Lim3P.port,Lim3P.pin,true, true, true);
	Limit4 = new DigitalIoPin(Lim4P.port,Lim4P.pin,true, true, true);


    xLEDQ = xQueueCreate( 10, sizeof( uint32_t ) );
    vQueueAddToRegistry( xLEDQ, "LED update Queue" );

	MotorConfig mcfg = { MotorX, MotorY, DirX, DirY, Lim1P, Lim2P, Lim3P, Lim4P,
			STEPS_PER_REV, (uint32_t)((SPEEDSLOW*100)*(EEProm->getSpeed()*100)/100)/10000,
			SPEEDFAST, SPEEDACCEL, EEProm->getXDir(), EEProm->getYDir(), xLEDQ };

	Draw = new DrawControl( PenP, EEProm->getPUp(),EEProm->getPDown(), LaserP );

	XY = new MotorXY(mcfg, Draw, uart );

    // TODO: insert code here	xUARTMutex = xSemaphoreCreateMutex();
#ifdef PARSETASK
    xParseQueue = xQueueCreate( 10, sizeof( char[INPUTMAXLEN+1] ) );
    vQueueAddToRegistry( xParseQueue, "Parser input Queue" );
#endif
#ifdef CDC
	/* low level USB communicationthread */
	xTaskCreate(cdc_task, "CDC",
				100, NULL, (tskIDLE_PRIORITY + 3UL), // 87
				(TaskHandle_t *) NULL);
#endif
	// higher level usb uart input
	xTaskCreate(vInput, "MDraw Input",
				300, NULL, (tskIDLE_PRIORITY + 3UL),
				(TaskHandle_t *) NULL);
#ifdef PARSETASK
	xTaskCreate(vGCode, "GCode Parser",
				250, NULL, (tskIDLE_PRIORITY + 4UL),
				(TaskHandle_t *) NULL);
#endif

#ifdef COMMANDQ
    xCommandQueue = xQueueCreate( 40, sizeof( command ) );
    vQueueAddToRegistry( xCommandQueue, "Plotting commands Queue" );

	xTaskCreate(vPlot, "Plotter mover task.",
				250, NULL, (tskIDLE_PRIORITY + 4UL),
				(TaskHandle_t *) NULL);
#endif

	xTaskCreate(vLED, "LED Handling", // lowest priority task, only visual indication.
			60, NULL, (tskIDLE_PRIORITY + 2UL),
			(TaskHandle_t *) NULL);

	vTaskStartScheduler();

    // Enter an infinite loop, just incrementing a counter
    while(1) {
    }
    return 0 ;
}
