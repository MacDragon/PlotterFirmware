/*
 * DrawControl.cpp
 *
 *  Created on: 29 Sep 2020
 *      Author: visa
 */

#include "DrawControl.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "helper.h"
#include <string.h>
#include "I2C.h"

#define STOPLASER

void DrawControl::setPenUpDown(uint8_t penup, uint8_t pendown) {

	this->penup = penup;
	this->pendown = pendown;
}

void DrawControl::SetOutofbounds(bool outside ) {
	if ( outside )
	{
		outofbounds = true;
		if ( curlaser > 0 )
		{
			intsetLaser(0);
		}

		if ( curpenpos < penup )
		{
			intsetPen(penup);
		}

	} else
	{
		if ( outofbounds )
		{
			outofbounds = false;
			if ( curlaser > 0 )
			{
				intsetLaser(curlaser);
			}

			if ( curpenpos < penup )
			{
				intsetPen(curpenpos);
			}
		}
	}


}

void DrawControl::sctInit( PinMap PenPin, uint8_t position, PinMap LaserPin )
{
	// Enable clock for timer

	Chip_SCT_Init(LPC_SCT0);
	Chip_SCT_Init(LPC_SCT1);

	// Table 202
	LPC_SCT0 -> CONFIG =
		SCT_CONFIG_16BIT_COUNTER+
		SCT_CONFIG_AUTOLIMIT_L; // Clear timer on match

	LPC_SCT1 -> CONFIG =
		SCT_CONFIG_16BIT_COUNTER+
		SCT_CONFIG_AUTOLIMIT_L; // Clear timer on match

	// Table 203
	LPC_SCT0 -> CTRL_L=
		(1 << 2) | // Halt timer for now
		(1 << 3) | // Clear timer
		(71 << 5); // max prescaler to keep the match setting as simple as possible

	LPC_SCT1 -> CTRL_L =
		(1 << 2) | // Halt timer for now
		(1 << 3) |
		(71 << 5);

	// Table 222 - compare/match value
	LPC_SCT0 -> MATCHREL[0].L = 20000-1; // 5% of 1000

	LPC_SCT1 -> MATCHREL[0].L = 1000-1;

	uint16_t match = (((1000*100)/(255)*(position))/100); // use large numbers for better scaling.
	if ( match > 996 )
		match = 1000; // ensure we actually get a 100% position.

	LPC_SCT0 -> MATCHREL[1].L = 1000+match;

	LPC_SCT1 -> MATCHREL[1].L = 1;

	// Table 229 - Match event
	LPC_SCT0 -> EVENT[0].STATE = 0xFFFFFFFF;
	LPC_SCT0 -> EVENT[0].CTRL = (0 << 0) | (1 << 12); // enables match  0 on low
	LPC_SCT0 -> EVENT[1].STATE = 0xFFFFFFFF;
	LPC_SCT0 -> EVENT[1].CTRL = (1 << 0) | (1 << 12); // enables match  1 on low

	LPC_SCT1 -> EVENT[2].STATE = 0xFFFFFFFF;
	LPC_SCT1 -> EVENT[2].CTRL = (0 << 0) | (1 << 12); // enables match 1 on low
	LPC_SCT1 -> EVENT[3].STATE = 0xFFFFFFFF;
	LPC_SCT1 -> EVENT[3].CTRL = (1 << 0) | (1 << 12); // enables match 0 on low

	LPC_SCT0 -> OUT[0].SET = (1 << 0);
	LPC_SCT0 -> OUT[0].CLR = (1 << 1);

    LPC_SCT1 -> OUT[1].SET = 0; // start laser disabled
//	LPC_SCT1 -> OUT[1].SET = (1 << 2);
	LPC_SCT1 -> OUT[1].CLR = (1 << 3);

	// Connect PWM-signal to P0_24
	Chip_SWM_MovablePinAssign(SWM_SCT0_OUT0_O,PenPin.port *32 + PenPin.pin);
	Chip_SWM_MovablePinAssign(SWM_SCT1_OUT1_O,LaserPin.port *32 + LaserPin.pin);

	// Start sct1 for pwm to set pen to initial position.
	LPC_SCT0 -> CTRL_L &= ~(1 << 2);
	LPC_SCT1 -> CTRL_L &= ~(1 << 2);
	// don't start laser at init.
}

void DrawControl::setPen( uint8_t position )
{
	curpenpos = position;
	if ( ! outofbounds )
	{
		intsetPen(position);
	}
}

void DrawControl::intsetPen( uint8_t position )
{
	uint16_t match = (((1000*100)/(255)*(position))/100); // use large numbers for better scaling.
	if ( match > 996 )
		match = 1000; // ensure we actually get a 100% position.

	LPC_SCT0 -> MATCHREL[1].L = 1000 + match;

	vTaskDelay(100); // allow time for pen to move.
}

DrawControl::DrawControl( PinMap PenPin, uint8_t penup, uint8_t pendown, PinMap LaserPin ) : penup(penup), pendown(pendown)
{

	I2C_config cfg;
	I2C i2c(cfg);
	char data[32] = { 0 }; // simulator has 32 byte storage
	char tx[1] = { 0 }; // specify start address in simulator (eeprom type addressing, 8-bit address)
	i2c.transaction(0x42, (uint8_t *) tx, 1, (uint8_t *) data, 32);
	if(strcmp(data, "Signal capture")==0) {
		simulator = true;
	} else simulator = false;

	outofbounds = false;
	inmotion = false; // setup so that we're prepared to draw && turn on laser, but keep it off when power is set.
	curpenpos = penup;
	curlaser = 0;
	sctInit( PenPin, penup, LaserPin ); // set the initial pen position ( should be up )
}

DrawControl::~DrawControl() {
	// TODO Auto-generated destructor stub
}


void DrawControl::setLaser( uint8_t power )
{
	curlaser = power;
#ifdef STOPLASER
	if ( ! outofbounds && inmotion )
	{
#endif
		intsetLaser(power);
#ifdef STOPLASER
	}
#endif
}

void DrawControl::intsetLaser( uint8_t power )
{
	LPC_SCT1 -> CTRL_L |= (1 << 2);
	if ( power > 1 )
	{
		uint16_t match = (((1000*100)/(255)*(power))/100); // use large numbers for better scaling.
		if ( match > 996 )
			match = 1000; // ensure we actually get a 100% position.

	    LPC_SCT1->OUT[1].SET = (1 << 2); // event 2 will set SCTx_OUT1
		LPC_SCT1->MATCHREL[1].L = match;
		LPC_SCT1 -> CTRL_L &= ~((1 << 2) + (1 << 3)); // restart timer
	} else
	{
		if ( simulator ) // set power to 2% on simulator so that detects faster, rather than true off.
		{
			uint16_t match = 20;
		    LPC_SCT1->OUT[1].SET = (1 << 2); // event 2 will set SCTx_OUT1
			LPC_SCT1->MATCHREL[1].L = match;
			LPC_SCT1 -> CTRL_L &= ~((1 << 2) + (1 << 3)); // restart timer so sim still sees a %
		} else
		{
		    LPC_SCT1->OUT[1].SET = 0; // event 3 has no effect on  SCTx_OUT1 --> laser is always off
			LPC_SCT1->MATCHREL[1].L = 1;
			LPC_SCT1->OUTPUT = 0; // set output to 0 and don't restart timer, ensures there's no small glitches.
		}
	}
	// clear and restart timer

}

uint8_t DrawControl::drawSpeed()
{
	if ( curlaser >= 1 ) // laser is enabled, return draw slow.  // curpenpos < penup removed for pen.
		return 1;
	else if ( curpenpos <= pendown )
		return 2;
	else
		return 0;
}

void DrawControl::ismoving( bool moving ) // ensure laser is only on when actually moving.
{
#ifdef STOPLASER
	if ( moving )
	{
		inmotion = true;
		if ( curlaser > 1 )
			intsetLaser( curlaser );
		else
			intsetLaser( 0 );

	} else
	{
		inmotion = false;
		intsetLaser( 0 );
	}
#endif
}

