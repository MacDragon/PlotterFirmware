#include "DigIO.h"
#include "FreeRTOS.h"
#include "task.h"
//#include "systick.h"


DigitalIoPin::DigitalIoPin(uint8_t const port, uint8_t const pin, bool const input, bool const pullup, bool const invert) :
port { port }, pin { pin }, input { input }, invert { invert }
{
	LPC_IOCON->PIO[port][pin] = (1U + pullup) << 3 | 1U << 7 | invert << 6;

	LPC_GPIO->DIR[port] = input ? LPC_GPIO->DIR[port] & ~(1UL << pin) : LPC_GPIO->DIR[port] | 1UL << pin;
}

DigitalIoPin::~DigitalIoPin() {};

/* Remaining DigIO functions are inlined in the header file */

/* Button child class of DigIO 8 */

ButtonPin::ButtonPin(int port, int pin, bool invert) :
		DigitalIoPin(port, pin, true, true, invert), press(0), pressed(false), release(0), returnstate(false), lastcall() { };


uint32_t ButtonPin::ReadWithBounce() {
	if ( xTaskGetTickCount()-lastcall < 10 )
		return returnstate; // don't check status unless it's been at least 9ms since last check;

	lastcall = xTaskGetTickCount();

	if(read()) {
		press++;
		release = 0;
	}
	else {
		release++;
		press = 0;
	}

	if ( press > 3 ) {
		timepressed = xTaskGetTickCount();
		pressed = true;
		returnstate = true;
	}

	if ( release > 3 )
	{
		returnstate = false;
	}

	if ( returnstate ) {
		uint32_t returnval = xTaskGetTickCount() - timepressed;
		if ( returnval == 0 ) returnval = 1;
		return returnval;
	}
	else return 0; // return the current internal button state
}

bool ButtonPin::ReadOnceWithBounce() {
	if ( !pressed ) return ReadWithBounce(); else if ( !ReadWithBounce() ) ResetPress();
	return 0;
}

void ButtonPin::ResetPress() { pressed = false; returnstate = false; };

bool ButtonPin::Released() { if ( !read() && pressed ) { ResetPress(); return true; } else return false; };

OutputPin::OutputPin(int port, int pin, bool invert) :
	DigitalIoPin( port,  pin, false, false, invert ), offms(0) { };

void OutputPin::setcounter(uint32_t time) {
	if ( !read() ) write(true);
	starttime = xTaskGetTickCount();
	offms = time;
};

void OutputPin::checkcounter() {
	if ( xTaskGetTickCount() - starttime > offms && read() ) write(false);
};


LEDPin::LEDPin(int port, int pin, bool invert) :
	OutputPin( port,  pin, invert ) { };



