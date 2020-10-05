/*
 * DrawControl.h
 *
 *  Created on: 29 Sep 2020
 *      Author: visa
 */

#ifndef DRAWCONTROL_H_
#define DRAWCONTROL_H_

#if defined (__USE_LPCOPEN)
#if defined(NO_BOARD_LIB)
#include "chip.h"
#else
#include "board.h"
#endif
#endif

#include "helper.h"

class DrawControl {
public:
	DrawControl( PinMap PenPin, uint8_t penup, uint8_t pendown, PinMap LaserPin );
	virtual ~DrawControl();

	void setPen( uint8_t position );
	void setLaser( uint16_t power );

	void setPenUpDown( uint8_t penup, uint8_t pendown );

	void SetOutofbounds( bool outside ); // move pen up temporarily.

	bool slowDraw(); // query whether to draw slow ( laser engaged )

	void ismoving( bool moving );

private:
	void intsetLaser( uint16_t power );
	void intsetPen( uint8_t position );
	void sctInit( PinMap PenPin, uint16_t position, PinMap LaserPin );
	uint8_t penup;
	uint8_t pendown;
	uint8_t curlaser;
	uint8_t curpenpos;
	bool outofbounds;
	bool inmotion;
	bool simulator;
};

#endif /* DRAWCONTROL_H_ */
