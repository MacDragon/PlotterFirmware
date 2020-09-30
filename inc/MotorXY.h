/*
 * MotorXY.h
 *
 *  Created on: 21.9.2020
 *      Author: Visa
 */

#ifndef MOTORXY_H_
#define MOTORXY_H_

#include "chip.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "Fmutex.h"
#include "DigIO.h"
#include "LpcUart.h"
#include <cmath>
#include "int32xy.h"
#include "helper.h"
#include "DrawControl.h"

#define STEPPERTEST
#define MINDELAY (144000)
#define INITRUNS (3)

struct MotorConfig {
	PinMap motorx;
	PinMap motory;
	PinMap dirx;
	PinMap diry;
	PinMap limit1;
	PinMap limit2;
	PinMap limit3;
	PinMap limit4;
	uint32_t steps;
	uint32_t ppsslow;
	uint32_t ppsfast;
	bool   invertx = false;
	bool   inverty = false;
};


class MotorXY {
public:
	enum XYdir { left, right, none, up=right, down=left };
	MotorXY(const MotorConfig &cfg, DrawControl *Draw = nullptr, LpcUart *UART = nullptr);
	MotorXY(const MotorXY &) = delete; // don't allow copy
	virtual ~MotorXY();
	int32xy_t gotoxy( int32xy_t move, bool absolute, uint32_t speed = 0, bool limit = false, uint32_t rpm = 0, bool skipaccel = true );
	bool trackinit();
	int32xy_t gotohome( XYdir xdir, XYdir ydir, bool forcelimit = false );
	int32xy_t gotomid();
	void setPPS(int32_t ppsslow, int32_t ppsfast );
	void setaccel( int32_t accel );
	void setInvert( bool xinv, bool yinv );

	int  getxpos();
	int  getypos();
	int  getabsxpos();
	int  getabsypos();

	int  getwidth();
	int  getmaxwidth();
	int  getheight();

	void toggledir(XYdir &dir);

	void isr(portBASE_TYPE *hpw); /* ISR handler. This will be called by the HW ISR handler. Do not call from application */

	// recovery no longer needed.
	int32xy_t dorecovery( XYdir xdirection, XYdir ydirection ); // recovery from direction in movement direction.

private:
	uint32_t drawslow();

	typedef bool (MotorXY::*LimitFn)();

	bool YLimitsSet;
	bool XLimitsSet;

	bool xinverted;
	bool yinverted;

	DigitalIoPin * getActiveLimit( void );
	void write(const char * str);
	int32xy_t RIT_start(int count1, int count2, int usstart, int usmax, uint32_t accel, bool skipaccel = true );
	int32xy_t domove(int32xy_t move, uint32_t initialpps,
					 uint32_t maxpps, XYdir xdirection, XYdir ydirection,
					 bool supress = false, uint32_t rpm = 0, bool skipaccel = true);

	int32xy_t plotLine(int32xy_t posstart, int32xy_t posend, bool inside = false );

	bool inbounds(int32xy_t pos);

	bool StepXLimit();

	bool StepYLimit();

	bool StepAnyLimit();

	LimitFn 	   Step1Limit;
	LimitFn		   Step2Limit;
	LpcUart 	 * uart;
	DrawControl  * draw;

	DigitalIoPin * Step1;
	DigitalIoPin * Step2;

	XYdir		   Dir1;
	XYdir		   Dir2;

	DigitalIoPin * StepX;
	DigitalIoPin * DirX;

	DigitalIoPin * StepY;
	DigitalIoPin * DirY;

	DigitalIoPin * LimitXL;
	DigitalIoPin * LimitXR;
	DigitalIoPin * LimitYU;
	DigitalIoPin * LimitYD;

	DigitalIoPin * Limit1;
	DigitalIoPin * Limit2;
	DigitalIoPin * Limit3;
	DigitalIoPin * Limit4;

	volatile uint32_t RIT_HalfCount;
	volatile uint32_t RIT_count1;
	volatile uint32_t RIT_count2;
	volatile uint32_t RIT_totalcount2;
	volatile uint32_t RIT_begin;
	volatile uint32_t RIT_end;

	volatile uint64_t RIT_cur;					// current delay amount.
	volatile uint32_t RIT_curacc;				  // 24.8 fixed point delay count
	volatile uint32_t RIT_cur32;				  // 24.8 fixed point delay count
	volatile uint32_t RIT_step;					// step size for fixed 'linear' ramp
	volatile uint32_t RIT_totalsteps;
	volatile uint32_t RIT_accelsteps;			// how long steps take
	volatile uint32_t RIT_decelstep;			// when to start deceleration
	volatile int32_t RIT_denom;                    // 4 * n + 1 in ramp algo (signed)
	volatile int32_t RIT_skippedaccel;
    volatile int32_t RIT_D;

	volatile uint32_t RIT_step_down;	     // start of ramp-down
	bool RIT_accelrate=false;

	SemaphoreHandle_t sbRIT;

	// ramp state-machine states
	enum accstate { RAMP_UP, RAMP_MAX, RAMP_DOWN };
	int32xy_t abspos;
	int32xy_t virtpos;

	volatile int32_t width=0;
	int32_t widest=0;
	volatile int32_t height=0;
	bool xfoundaxis=false;
	bool yfoundaxis=false;

	bool ymotor;

	uint32_t ppsslow;

	uint32_t ppsfast;

	accstate RIT_accstate;

	uint32_t stepsperrev;
};

#endif /* MOTORXY_H_ */
