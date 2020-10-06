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


// config structure defining all the pins and initial direction and speeds
// and steps per revolution of stepper for acceleration calculations.
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
	uint32_t accelrpm;
	bool   invertx = false;
	bool   inverty = false;
};


class MotorXY {
public:
	enum XYdir { left, right, none, up=right, down=left };
	MotorXY(const MotorConfig &cfg, DrawControl *Draw = nullptr, LpcUart *UART = nullptr);
	MotorXY(const MotorXY &) = delete; // don't allow copy
	virtual ~MotorXY();

	// run initial track initialisation to detect limit switches and size of stepper tracks.
	bool trackinit();

	// move to position xy in either relative or absolute terms, with optional faster speed/acceleration settings, and limit switch stop.
	int32xy_t gotoxy( int32xy_t move, bool absolute, uint32_t speed = 0, bool limit = false, uint32_t rpm = 0, bool skipaccel = true );

	// rehome steppers to either end of track. Forcelimit uses limit switches to stop rather than relative position.
	int32xy_t gotohome( XYdir xdir, XYdir ydir, bool forcelimit = false );

	// goto middle of plotter area.
	int32xy_t gotomid();

	// set speed for plotting and moving
	void setPPS(int32_t ppsslow, int32_t ppsfast);

	// set stepper direction for M5
	void setInvert( bool xinv, bool yinv );

	// getters for calling program to know details about plotter area/position.

	// these functions return virtual position that can be in or outside plottable area.
	int  getxpos();
	int  getypos();
	// abs position only gives actual plotter head position within real travelable area.
	int  getabsxpos();
	int  getabsypos();

	// returns track sizes. Max width designed for speed test, in case there was any variation during init.
	int  getwidth();
	int  getmaxwidth();
	int  getheight();

	/* ISR handler. This will be called by the HW ISR handler. Do not call from application, treat as private */
	void isr(portBASE_TYPE *hpw);

private:
	uint32_t drawslow();

	// recovery from limit switch no longer needed except for internal initialisation routine, now private.
	int32xy_t dorecovery( XYdir xdirection, XYdir ydirection ); // recovery from direction in movement direction.
	void toggledir(XYdir &dir);

	typedef bool (MotorXY::*LimitFn)();

	// internal variables to track whether track has been initialised on an axis.
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

	// checks whether requested co ordinate is within plottable area.
	bool inbounds(int32xy_t pos);

	// check if limit switch is active on axis in direction of movement.
	bool StepXLimit();
	bool StepYLimit();

	// check if any limit switch is active. Used initially before calibration for all movement.
	bool StepAnyLimit();

	// pointers to limit switches and axis controls so that
	// interrupt / limit switch checkers don't need to be altered for changes
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

	// internal variables for stepper interrupt operation...
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
	volatile accstate RIT_accstate;

	int32xy_t abspos;
	int32xy_t virtpos;

	volatile int32_t width=0;
	int32_t widest=0;
	volatile int32_t height=0;

	bool ymotor; // is y motor present, or are we using a single stepper track for testing.

	uint32_t ppsslow;
	uint32_t ppsfast;
	uint32_t accelrpm;

	uint32_t stepsperrev;
};

#endif /* MOTORXY_H_ */
