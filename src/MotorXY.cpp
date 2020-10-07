/*
 * MotorXY.cpp
 *
 *  Created on: 21 Sep 2020
 *      Author: visa
 */

#define SQUAREWAVE

#if defined (__USE_LPCOPEN)
#if defined(NO_BOARD_LIB)
#include "chip.h"
#else
#include "board.h"
#endif
#endif

#include "MotorXY.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "event_groups.h"

#include "ITM_write.h"

#include "helper.h"

static MotorXY *MXY;

extern "C" void RIT_IRQHandler(void) {
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

	if(MXY != nullptr ) {
		MXY->isr(&xHigherPriorityTaskWoken);
	}
	portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}


bool MotorXY::StepXLimit()
{
	if ( XLimitsSet )
	{
		if ( DirX->read() == left )
			return LimitXL->read();
		else
			return LimitXR->read();
	} else return StepAnyLimit();
}

bool MotorXY::StepYLimit()
{
	if ( YLimitsSet )
	{
		if ( DirY->read() == up )
			return LimitYU->read();
		else
			return LimitYD->read();
	} else return StepAnyLimit();
}

bool MotorXY::inbounds( int32xy_t pos) {

	if ( width > 0 && height > 0 )
		if ( pos.x < 0 || pos.x > width || pos.y < 0 || pos.y > height )
			return false;
		else
			return true;
	else return true; // we don't know bounds, so must calculate move.
}

bool MotorXY::StepAnyLimit()
{
	uint32_t ylimits = 0;
	if ( ymotor )
		ylimits =  Limit3->read() || Limit4->read();

	if ( ( Limit1->read() || Limit2->read() ) + ylimits )
		return true;
	else
		return false;
}

void MotorXY::isr(portBASE_TYPE *xHigherPriorityWoken){
	// Tell timer that we have processed the interrupt.
	// Timer then removes the IRQ until next match occurs

	Chip_RIT_ClearIntStatus(LPC_RITIMER); // clear IRQ flag
	if(RIT_count1 < RIT_totalsteps) {

		if ( ! Step1->read() )
		{
			if  ( ! (this->*Step1Limit)()) // only allow sending pulse if not limited on primary axis.
			{
				if ( RIT_totalcount2 != 0 && ! (this->*Step2Limit)() )
				{
					if (  RIT_D > 0 ) // Bresentham algo says we want to move on secondary axis.
					{
						   Step2->write(true);
						   RIT_count2++;
						   RIT_D = RIT_D + (2 * (RIT_totalcount2 - RIT_totalsteps)); // calculate next Bresentham step.
					} else
						   RIT_D = RIT_D + 2*RIT_totalcount2;
				} else if ( RIT_totalcount2 != 0 && (this->*Step2Limit)() ) // ensure we stop if we hit axis 2
				{
					if (  RIT_D > 0 ) // if we actually want to move another step on axis 2 whilst at limit, then stop.
					{
						Chip_RIT_Disable(LPC_RITIMER); // disable timer
						xSemaphoreGiveFromISR(sbRIT, xHigherPriorityWoken);
						return;
					} else
						   RIT_D = RIT_D + 2*RIT_totalcount2;
				}

				// not being limited in movement by either axis limit switch, proceed

				Step1->write(true); // only start a step cycle if limit switches not hit.
				RIT_count1++;
				if ( RIT_accelrate == false )
				{
					if ( RIT_count1 < RIT_HalfCount ) // still in acceleration phase.
					{
						if ( RIT_cur > RIT_end )
						{
							RIT_cur -=RIT_step;
							if ( RIT_cur < RIT_end )
								RIT_cur = RIT_end;
						}
					} else 	if ( RIT_totalsteps - RIT_count1 <= RIT_accelsteps) // deceleration phase...
					{
						if ( RIT_cur < RIT_begin )
						{
							RIT_cur +=RIT_step;
							if ( RIT_cur > RIT_begin )
								RIT_cur = RIT_begin;
						}
					}
				} else // non linear accel
				{
					// acceleration code modified from https://www.avrfreaks.net/comment/931537#comment-931537
					// which originally based on article https://www.embedded.com/generate-stepper-motor-speed-profiles-in-real-time/

					switch (RIT_accstate) {
					case RAMP_UP:                       // we are accelerating
						if (RIT_count1 >= RIT_HalfCount ) {           // midpoint: decel
							RIT_accstate = RAMP_DOWN;
							RIT_denom = ((RIT_count1 - RIT_totalsteps - RIT_skippedaccel ) << 2) - 3;
							if (!(RIT_totalsteps & 1)) {             // even # of moves: repeat last step
								RIT_denom += 4;
								break;
							}
						}
						// no break: share code for ramp algo

					case RAMP_DOWN:                     // we are decelerating
						// calculate the step value for the next step
						RIT_denom += 4;
						RIT_cur32 -= ((long) (RIT_cur32 << 1)) / RIT_denom;    // ramp algorithm
						RIT_curacc = (RIT_cur32 + 128) >> 8;             // round 24.8 format -> int16 and div 2

						if (RIT_curacc <= RIT_end) {                 // go to constant speed?

//							if ( RIT_count1 < ( RIT_HalfCount >> 2 ) )
//								xSemaphoreGiveFromISR(xMaxSpeed, &xHigherPriorityWoken); // inform task that we hit full speed during run.
							RIT_accstate = RAMP_MAX;            // yup
							RIT_decelstep = RIT_totalsteps - RIT_skippedaccel - RIT_count1 - 1;
							RIT_curacc = RIT_end;                      // next step is this
							RIT_cur32 = ((long) RIT_cur) << 8;          // and put it in the 24.8 format
							break;
						}
						break;

					case RAMP_MAX:                      // we're holding a constant speed
						if (RIT_count1 == RIT_decelstep) {       // are we done yet?
							RIT_accstate = RAMP_DOWN;           // yes - start decelerating next time
							RIT_denom = ((RIT_count1 - RIT_totalsteps - RIT_skippedaccel) << 2) + 1;
						}
						break;

					default:                            // end of last step - cleanup

						break;
					}

					if ( RIT_curacc > MINDELAY /2 && RIT_skippedaccel != 0)
						RIT_cur = MINDELAY / 2;
					else
						RIT_cur = RIT_curacc;
				}
			} else
			{
				// not yet set pin high, so can leave state as is.
				Chip_RIT_Disable(LPC_RITIMER); // disable timer
				// Give semaphore and set context switch flag if a higher priority task was woken up
				xSemaphoreGiveFromISR(sbRIT, xHigherPriorityWoken);
				return;
			}
		}
		else
		{  // we're in down phase of square wave, set pins back to false, adjust timer to next step if in acceleration phase.
			Step1->write(false);
			Step2->write(false);
			// adjust timer after a full cycle.
			if ( LPC_RITIMER->COMPVAL != RIT_cur )
			{
				Chip_RIT_Disable(LPC_RITIMER);
				LPC_RITIMER->COMPVAL = (uint32_t) RIT_cur; // only using 32 bit part so save a write maybe.
				Chip_RIT_Enable(LPC_RITIMER);
			}
			return;
		}
	}
	else { // reached end of wanted steps, make sure everything is set to known low state.
		Step1->write(false); // BUG, was not setting low on last step, leaving pin in wrong state!
		Step2->write(false);
		Chip_RIT_Disable(LPC_RITIMER); // disable timer
		// Give semaphore and set context switch flag if a higher priority task was woken up

		xSemaphoreGiveFromISR(sbRIT, xHigherPriorityWoken);
		return;
	}

	// End the ISR and (possibly) do a context switch
}

void MotorXY::toggledir(XYdir &dir) {
	if ( dir == left ) dir = right;
	else if ( dir == right ) dir = left;
}

// write debug messages to uart if uart defined.
void MotorXY::write(const char *str) {
	if ( uart != nullptr )
	{
		uart->write(str);
	}
}

// accel in rpm/s, default to linear ramp if not given. skip accel starts curve from start speed rather than 0.
// timer does not know which axis is which so that it can be kept simpler, uses pointers to axis objects
int32xy_t MotorXY::RIT_start(int count1, int count2, int usstart, int usmax, uint32_t accel, bool skipaccel ) {
	// Determine approximate compare value based on clock rate and passed interval

	RIT_begin = (uint64_t) Chip_Clock_GetSystemClockRate() * (uint64_t) ( usstart /2 ) / 1000000;
	RIT_end = (uint64_t) Chip_Clock_GetSystemClockRate() * (uint64_t) ( usmax / 2 ) / 1000000;

	RIT_cur = RIT_begin; // set rit timer initial period whether accelerating or not

	// linear curve acceleration based on algorhythm from article https://www.embedded.com/generate-stepper-motor-speed-profiles-in-real-time/

	if ( accel == 0 ) // if accel not specified then use simple linear ramp on timer. Not efficient, but simple.
	{
		RIT_accelrate = false;
				//  10% travel
		if ( count1 > 265 )
			RIT_accelsteps = ( count1 / 10 );
		else RIT_accelsteps = count1 / 2;
		RIT_step = ( RIT_begin-RIT_end ) / RIT_accelsteps;
	} else
	{
		RIT_accelrate = true; // using a true curve to achieve actual linear acceleration.

		RIT_cur = RIT_begin; 											// constant to go from rpm/s to rad/s^2
		RIT_curacc = ( 0.676*Chip_Clock_GetSystemClockRate()*sqrt(( 2*degToRad(360.0/stepsperrev) )/( accel * 0.1047197551143) ) ) / 2;
		RIT_cur32 = RIT_curacc << 8;
		RIT_denom = 1;
		RIT_accstate = RAMP_UP;

		RIT_skippedaccel=0;

		// if we want to start movement at defined slow move speed
		// calculate how many acceleration steps to skip later on deceleration so we don't decelerate too early.
		if ( skipaccel )
		{
			// skip initial part of acceleration curve to point we know is OK to start from.
			while ( RIT_curacc > RIT_begin && RIT_skippedaccel < count1/2 )
			{
				RIT_skippedaccel++;
				RIT_denom += 4;
				RIT_cur32 -= ((long) (RIT_cur32 << 1)) / RIT_denom;    // ramp algorithm
				RIT_curacc = (RIT_cur32 + 128) >> 8;             // round 24.8 format -> int16
			}
		}

	}

	// disable timer during configuration
	Chip_RIT_Disable(LPC_RITIMER);

	// setup how many steps to move for ISR.
	RIT_count1 = 0;
	RIT_totalsteps = count1;
	RIT_count2 = 0;
	RIT_totalcount2 = count2;

	RIT_HalfCount = (count1 + 1) >> 1; // midpoint, rounded up, to ensure acceleration stops by here so we can still decelerate.

    RIT_D = (2 * count2) - count1;

	// enable automatic clear on when compare value==timer value
	// this makes interrupts trigger periodically
	Chip_RIT_EnableCompClear(LPC_RITIMER);
	// reset the counter
	Chip_RIT_SetCounter(LPC_RITIMER, 0);
	Chip_RIT_SetCompareValue(LPC_RITIMER, RIT_cur);
	// start counting

	uint32_t starttime = xTaskGetTickCount();

	// if there are actual steps to move then start timer and wait for it to complete steps
	if ( RIT_totalsteps > 0 )
	{
		draw->ismoving( true ); // tell drawing object that we are about to move, so that laser can be switched on if needed.

		Chip_RIT_Enable(LPC_RITIMER);
		// Enable the interrupt signal in NVIC (the interrupt controller)
		NVIC_EnableIRQ(RITIMER_IRQn);
		// wait for ISR to tell that we're done

		if( xSemaphoreTake(sbRIT, portMAX_DELAY) == pdTRUE) {
			// Disable the interrupt signal in NVIC (the interrupt controller)
			NVIC_DisableIRQ(RITIMER_IRQn);

			draw->ismoving( false ); // done moving so inform laser can be turned off if needed.

			uint32_t runtime = xTaskGetTickCount()-starttime;
			write("interrupt  took "); // debug output of how long move took
			write(getnumstr(runtime));
			write("ms to process ");
			write(getnumstr(RIT_count1));
			write(" steps\r\n");

			return {(int32_t)RIT_count1, (int32_t)RIT_count2}; // return how many steps moved.
		}
		else {
			draw->ismoving( false ); // done moving so inform laser can be turned off if needed.
			// unexpected error. Shouldn't ever be here... not currently handled gracefully.
			return {-1,-1};
		}
	}
	return {0,0}; // if we got here no movement happened.
}


// internal move command to setup axes for interrupt, assigning correct axes objects to timer used pointers.
// and after move assigns movement performed move to correct x/y axis.
int32xy_t MotorXY::domove(int32xy_t move, uint32_t initialpps,
				 uint32_t maxpps, XYdir xdirection, XYdir ydirection,
				 bool supress, uint32_t rpm, bool skipaccel)
{
	int32xy_t moved = {0,0};

	if ( !ymotor ) // only perform y movement if we have a defined y axis.
	{
		move.y = 0;
	}

	if ( maxpps == 0 || maxpps < initialpps ) maxpps = initialpps; // ensure that we are moving at initial pps as minimum.

	if ( move.x > 0 || move.y > 0 ) // if there is a move to actually perform check axes.
	{
		write("\r\nDomove: moving "); // just some debug messaging to inform of move request magnitude.
		if ( move.x > 0)
		{
			write(getnumstr(move.x));
			write(" steps ");
			if ( xdirection == left )
				write("Left ");
			else
				write("Right ");
		}
		if ( move.y > 0)
		{
			write(getnumstr(move.y));
			write(" steps ");
			if ( ydirection == up )
				write("Up");
			else
				write("Down");
			}
		write("\r\n");
		// setup directions

		DirX->write(xdirection);
		DirY->write(ydirection);

		if ( move.x < move.y ) // check if we are in a steep line for Bresentham
		{
			Step1 = StepY; // if so then swap x/y axes order to 2/1 for ISR calls, so that axis with larger movement is primary.
			Step2 = StepX;
			Step1Limit = &MotorXY::StepYLimit;
			Step2Limit = &MotorXY::StepXLimit;
			// request actual movement
			moved =  RIT_start( move.y, move.x, 1000000/initialpps, 1000000/maxpps, rpm, skipaccel); // 1us = 1000000 hz, so divide this by desired pps
			int32_t temp = moved.x;
			moved.x = moved.y;
			moved.y = temp;
		} else // axis not swapped, proceed with x/y as 1/2
		{
			Step1 = StepX;
			Step2 = StepY;
			Step1Limit = &MotorXY::StepXLimit;
			Step2Limit = &MotorXY::StepYLimit;
			moved =  RIT_start( move.x, move.y, 1000000/initialpps, 1000000/maxpps, rpm, skipaccel); // 1us = 1000000 hz, so divide this by desired pps
		}

		if ( moved.x != move.x  || moved.y != move.y ) // if movement did not fully complete, a limit switch was hit.
		{
			write("\r\nError: Limit switch hit with ");
			write(getnumstr(move.x-moved.y));
			write(" steps left.\r\n");
		}
		if ( xdirection == left ) moved.x=0-moved.x; // return negative movement for left
		if ( ydirection == down ) moved.y=0-moved.y; // return negative movement for down
	}

	if ( supress ) // if supression requested don't inform errors, just return original requested movement.
	{
		return move;
	} else
	return moved;
}



DigitalIoPin * MotorXY::getActiveLimit( void )
{
	int ylimit = 0;
	if ( ymotor ) // check if we have both axes active.
	{
		 ylimit = Limit3->read() + Limit4->read();
	}

	if (  Limit1->read() + Limit2->read() + ylimit != 1 ) // read limit switches.
	{
		write("Error: did not stop on limit switch on home run"); // active limit only used during init before calibration.
		return nullptr;
	} else
	{
		if ( Limit1->read() ) return Limit1;
		else if ( Limit2->read() ) return Limit2;
		else if ( ymotor && Limit3->read() ) return Limit3;
		else if ( ymotor && Limit4->read() ) return Limit4;
		else
		{
			write("Error: Limit switch stopped reading before selection.");
			return nullptr;
		}
	}

}

// automated discovery of limit switches and track lengths.
bool MotorXY::trackinit() {
	int32xy_t correction = {0,0};
	// home to left, returns when hit limit switch. For first move, no care how many steps left, just that we get to switch
	int32xy_t moved = {0,0};

	// find X home & left limit switch

	gotoxy( {-99999,0}, false, ppsslow, true );

	// 50ms delays to ensure switch bounce has settled during calibration.
	// instantaneus reads in RIT interrupt seem to work ok, but could be refactored to interrupt flags.

	vTaskDelay(50);

	LimitXL = getActiveLimit();

	if ( LimitXL == nullptr )
	{
		write("Error: did not stop on a single limit switch on home run\r\n");
		sendLED(1);
		return false;
	}

	correction = dorecovery(left, none);
	abspos.x = 0+correction.x;

	// moved off first switch, lets fine second

	// find Y home & up limit switch
	if ( ymotor )
	{
		gotoxy( {0,-99999}, false, ppsslow, true );

		vTaskDelay(10);

		LimitYD = getActiveLimit();

		if ( LimitYD == nullptr )
		{
			write("Error: did not stop on a single limit switch on home run\r\n");
			sendLED(1);
			return false;
		}


		correction = dorecovery(none, down);
		abspos.y = 0+correction.y;
	}

	// find X home & right limit switch

	moved =  gotoxy( {99999,0}, false, ppsslow, true );

	vTaskDelay(50);

	LimitXR = getActiveLimit();

	if ( LimitXR == nullptr )
	{
		write("Error: did not stop on a single limit switch on home run\r\n");
		sendLED(1);
		return false;
	}

	correction = dorecovery(right, none);
	abspos.x +=moved.x+correction.x;

	// find X home & left limit switch

	if ( ymotor )
	{

		moved =  gotoxy( {0,99999}, false, ppsslow, true );

		vTaskDelay(50);

		LimitYU = getActiveLimit();

		if ( LimitYU == nullptr && ymotor )
		{
			write("Error: did not stop on a single limit switch on home run\r\n");
			sendLED(1);
			return false;
		}

		if ( ymotor )
			YLimitsSet = true;
		abspos.y += moved.y;//+correction.y;
	} else abspos.y=0;

	XLimitsSet = true;

	moved = gotoxy( {20,0}, false, ppsslow, true );
	abspos.x +=moved.x;


// limit switches found and set.

	if ( abspos.x < 50 || (abspos.y < 50 && ymotor ) )
	{
		write("Error: did not Find limits wide enough apart\r\n");
		sendLED(1);
		return false;
	}

	int32_t approxwidth = abspos.x-10; // allow a little fudge value for initilisatin verification


	int32_t approxheight = abspos.y-10;
	if ( ! ymotor )
		approxheight = 0;

	XYdir direction = left;

	int32_t initwidth = 0; // finding width, start at zero.
	int32_t initheight = 0;

	widest = 0;

	for ( int i=0;i<INITRUNS;i++)
	{
		 // move to just short of limit switch with fast accelerated movement.
		moved = domove({approxwidth,approxheight}, ppsslow, ppsfast, direction, direction, false, 0 );//accelrpm );

		// do slower individual moves into limit switches.
		int32xy_t small=domove({20,0}, ppsslow, ppsslow, direction, none, false );
		abspos.x+=moved.x+small.x;

		moved.x += small.x;

		if ( ymotor )
		{
			small=domove({0,20}, ppsslow, ppsslow, none, direction, false );
			abspos.y+=moved.y+small.y;

			moved.y += small.y;
		}

		// check if this distance is wider than previously discovered
		if ( moved.x > widest ) widest = moved.x;

		write("Moved ");
		write(getnumstr(moved.x));
		write(" X steps ");
		initwidth+=abs(moved.x);
		write(getnumstr(moved.y));
		write(" Y steps\r\n");
		initheight+=abs(moved.y);

		if ( ! StepAnyLimit() )
		{
			write("Error: did not stop on limit switch\r\n");
			sendLED(1);
			return false;
		}

		toggledir(direction);
	}

	// after init, dorecovery should not be needed.

	width = (initwidth/INITRUNS);

	height = (initheight/INITRUNS);

	write("Widest Init run ");
	write(getnumstr(widest));
	write(" steps\r\n");


	write("Moving ");
	write(getnumstr(width/2));
	write("x ");
	write(getnumstr(height/2));
	write("y to center.\r\n");

	moved = domove({width/2,height/2}, ppsslow, ppsslow, direction, direction, false );

	correction = {0,0};
	abspos.x+=moved.x;
	abspos.y+=moved.y;

	virtpos = abspos; // sync real and virtual positions after init.

	sendLED(0);

	return true;
}

// primary movement command.
int32xy_t MotorXY::gotoxy( int32xy_t move, bool absolute, uint32_t speed, bool limit, uint32_t rpm, bool skipaccel ) {
	XYdir xdir;
	XYdir ydir;
	int32xy_t moved = {0,0};

    if ( !ymotor )
    {
    	move.y = 0;
    }

	if ( speed == 0 ) speed = drawslow(); // set default speed, slow if pen down.

	int32xy_t startpos = virtpos;

	if ( absolute ) // calculate relative reference from absolute, to give to move command.
	{
		move -=virtpos;
	}

	 // check if move is starting outside drawable area, calculate out of bounds part of move instead of performing
	if ( ! inbounds(virtpos) )
	{
		// virtually draw line and see if it intersects with bounds.
		virtpos = plotLine(virtpos, move+virtpos);

		// store how much movement happened to either end of command or intersect.
		moved = virtpos-startpos;

		// get the plotter back at right position by moving from absolute position to new inbounds
		// virtual position for restarting line if it's back in bounds.
		if ( inbounds(virtpos) )
		{
			int32_t xmove = virtpos.x-abspos.x;
			int32_t ymove = virtpos.y-abspos.y;

			if ( xmove == 0 ) xdir = none;
			else if ( xmove < 0 ) xdir = left; //  = (startpos.x-abspos.x)?left:right;
			else if ( xmove > 0 ) xdir = right;

			if ( ymove == 0 ) ydir = none;
			else if ( ymove < 0 ) ydir = down; //  = (startpos.x-abspos.x)?left:right;
			else if ( ymove > 0 ) ydir = up;

			int32xy_t realmove = abs(abspos-virtpos);
			// if drawing pen up here before head reposition.
			if ( draw != nullptr ) draw->SetOutofbounds(true);

			realmove = domove(realmove, ppsslow, speed, xdir, ydir, false, rpm, skipaccel );

			// if drawing pen back down here.

			abspos += realmove;
		}

		// removed already moved portion from original request.
		move = move-moved;
	}

	if ( move != 0 ) // requested position still needs actual movement.
	{
		// check if end position inbounds.

		int32xy_t realmove = move;
		if ( !inbounds(abspos+move) )
		{
			realmove = plotLine(virtpos, move+virtpos, true);

			realmove = realmove-abspos;
		}

		if ( draw != nullptr ) draw->SetOutofbounds(false);

		// setup move directions.
		if ( realmove.x < 0 )
			xdir = left;
		else if ( realmove.x > 0 )
			xdir = right;
		else xdir = none;

		if ( realmove.y < 0 )
			ydir = down;
		else if ( realmove.y > 0 )
			ydir = up;
		else ydir = none;

		realmove = abs(realmove);

		moved = domove(realmove, ppsslow, speed, xdir, ydir, false, rpm, skipaccel );

		if ( width > 0 && height > 0)
		{
			abspos += moved;
		}

		// if we're not liming the the range of movement to inbounds ( for testing limits etc )
		// just return the full move as having been done set full movement as new position
		// and ignore the rest of it after physical move, don't need to know what line would look like there..
		if ( ! limit ) //  limit is only meant for initilisation and speed runs etc.
		{
			virtpos = virtpos + move;
			moved +=move;
		} else
		{
			// adjust our actual absolute position based on what move actually happened.
			// adjust our virtual position to end point of reqeuested line.
			virtpos = virtpos + moved;
		}
	}


	return moved;
}

int32xy_t MotorXY::gotohome( XYdir xdir, XYdir ydir, bool forcelimit ) {
	int32xy_t movedx = {0,0};
	int32xy_t movedy = {0,0};

	if ( forcelimit || (width == 0 || ( height == 0 && ymotor ) ) ) // not yet calibrated.
	{
		if ( xdir != none)
			movedx =  domove({99999, 0}, ppsslow, ppsslow, xdir, none, false, 0);
		if ( ydir != none)
			movedy =  domove({0, 99999}, ppsslow, ppsslow, none, ydir, false, 0);

		if ( xdir == left)
		{
			abspos.x =0;
			virtpos.x = 0;
		}
		else if ( xdir == right )
		{
			abspos.x = width;
			virtpos.x = width;
		}


		if ( ydir == down)
		{
			abspos.y = 0;
			virtpos.y = 0;
		}
		else if ( xdir == up )
		{
			abspos.y = height;
			virtpos.y = height;
		}
	} else
	{
		int32xy_t move = {0,0};
		if ( xdir == right ) move.x = width;
		if ( ydir == up )  move.y = height;
		movedx = gotoxy(move, true, ppsfast, true);
//		write("Error: calibrated gohome not yet implemented.");
	}
	return {movedx.x, movedy.y};
}

int32xy_t MotorXY::gotomid() {
	int32xy_t moved;
	if ( width != 0 )
	{
		moved = gotoxy(
				{( width-2)/2, 0},
				true);
	}
	return moved;
}

void MotorXY::setPPS(int32_t ppsslow, int32_t ppsfast ) {
	this->ppsslow = ppsslow;
	this->ppsfast = ppsfast;
}

int MotorXY::getwidth() {
	return width;
}

int MotorXY::getmaxwidth() {
	return widest;
}

int MotorXY::getxpos() {
	return virtpos.x;
}

int MotorXY::getypos() {
	return virtpos.y;
}


int MotorXY::getabsxpos() {
	return abspos.x;
}

int MotorXY::getabsypos() {
	return abspos.y;
}

int MotorXY::getheight() {
	return height;
}

MotorXY::~MotorXY() {
}

uint32_t MotorXY::drawslow()
{
	if (draw == nullptr ) return ppsfast; // we don't have draw control, assume draw fast.
	else
	{
		if ( draw->slowDraw() )
			return ppsslow;
		else
			return ppsfast;
	}
}

int32xy_t MotorXY::plotLine(int32xy_t posstart, int32xy_t posend, bool inside)
{

    if ( !ymotor ) // no ymotor, no y movement!
    {
    	posstart.y = 0;
    	posend.y = 0;
    }

    int32_t d1 = posend.x-posstart.x;
    int32_t d2 = posend.y-posstart.y;

    int32_t linexpos = posstart.x;
    int32_t lineypos = posstart.y;

    int32_t * axis1 = &linexpos;
    int32_t * axis2 = &lineypos;

    int32_t step1 = 1;
    int32_t step2 = 1;


    if ( abs(d2) > abs(d1) ) // bigger y than x, swap axis.
    {
        axis1 = &lineypos;
        axis2 = &linexpos;
        int32_t dt = d1;
        d1 = d2;
        d2 = dt;
    }

    if ( d1 < 0 ) step1 = -1;
    if ( d2 < 0 ) step2 = -1;


    d2 = abs(d2);
    d1 = abs(d1);

	int32_t D = (2 * d2) - d1;

	for ( int count1=0;count1<d1;count1++)
	{
		// move x stepper

		*axis1+=step1;
		if ( inside && !inbounds({linexpos, lineypos} ) )
		{ // step took us out of bounds
			*axis1-=step1;
			break;
		}
		if ( d2 != 0 )
		{
			if ( D > 0 )
			{
				// move y stepper
				   *axis2+=step2;
					if ( inside && !inbounds({linexpos, lineypos} ) )
					{ // step took us out of bounds
						*axis2-=step2;
						break;
					}
				   D = D + (2 * (d2 - d1));
			} else
				   D = D + 2*d2;
		}


		if ( !inside && inbounds( {linexpos, lineypos}) )
		{
			break; // we've reached inbounds positin, return!
		}
	}
	return {linexpos, lineypos}; // return our new position, whether at end of move or where we've reached in bounds.
}

int32xy_t MotorXY::dorecovery( XYdir xdirection, XYdir ydirection ) // recovery from direction in movement direction.
{
//	xSemaphoreTake(xAllowMotor);

	XYdir MoveDirX = xdirection;
	XYdir MoveDirY = ydirection;

	int32_t xcorrectioncount=0;
	int32_t ycorrectioncount=0;

	vTaskDelay(1);

	// allow movement upto 20 steps to recover out of limit switch. Any more and there is likely some bigger physical issue.
	// if axis calibrated, check only relevant sensor, otherwise check all sensors.

	if ( xdirection != none)
	{
		toggledir(MoveDirX);

		DirX->write(xdirection);
		while ( xcorrectioncount < 20 && StepXLimit() )
		{
			DirX->write(MoveDirX);
			xcorrectioncount++;
			// move slowly to allow switch movement to be registered accurately and not overshoot reovery.
			StepX->write(true);
			vTaskDelay(10);
			StepX->write(false); // sim steps on this.
			vTaskDelay(10);
			DirX->write(xdirection);
		}

		if ( xcorrectioncount == 20 )
		{
			write("Limit switch recovery failed, too many steps.");
		}

		if ( xcorrectioncount != 0 )
		{

			write("Limit switch recovered with ");
			write(getnumstr(xcorrectioncount));
			write(" steps going ");
			if ( xdirection == right )
				write(" Left ( recovering from Right move )\r\n");
			else if ( xdirection == left )
				write(" Right ( recovering from Left move )\r\n");
		//	xSemaphoregive(xAllowMotor);
			if ( xdirection == right ) xcorrectioncount=0-xcorrectioncount;
		} else
		{
			write("Not at limit switch, no X recovery\r\n");
		}
	}

	if ( ydirection != none && ymotor )
	{
		toggledir(MoveDirY);
		DirY->write(ydirection); // thus switch the actual desired movement direction
		while ( ycorrectioncount < 20 && StepYLimit() )
		{
			DirY->write(MoveDirY);
			ycorrectioncount++;
			// move slowly to allow switch movement to be registered accurately and not overshoot reovery.
			StepY->write(true);
			vTaskDelay(10);
			StepY->write(false); // sim steps on this.
			vTaskDelay(10);
			DirY->write(ydirection);
		}

		if ( ycorrectioncount == 20 )
		{
			write("Limit switch recovery failed, too many steps.");
		}

		if ( ycorrectioncount != 0 )
		{
			write("Limit switch recovered with ");
			write(getnumstr(ycorrectioncount));
			write(" steps going ");
			if ( ydirection == down )
				write(" Up ( recovering from Down move )\r\n");
			else if ( ydirection == up )
				write(" Down ( recovering from Up move )\r\n");
		//	xSemaphoregive(xAllowMotor);
			if ( ydirection == up ) ycorrectioncount=0-ycorrectioncount;
		} else
		{
			write("Not at limit switch, no X recovery\r\n");
		}
	}

	return {xcorrectioncount, ycorrectioncount};
}

void MotorXY::setInvert(bool xinv, bool yinv)
{
	if ( xinverted != xinv ) // doesn't match current, swap limit.
	{
		DirX->inverted(xinv);
		xinverted = xinv;

		DigitalIoPin * TempLimit = LimitXR;
		LimitXR = LimitXL;
		LimitXL = TempLimit;

		abspos.x = width - abspos.x;
		virtpos.x = width - virtpos.x;

	}

	if ( yinverted != yinv && ymotor ) // doesn't match current, swap limit.
	{
		DirY->inverted(yinv);

		DigitalIoPin * TempLimit = LimitYD;
		LimitYD = LimitYU;
		LimitYU = TempLimit;
		yinverted = yinv;
		abspos.y = height - abspos.y;
		virtpos.y = height - virtpos.y;
	}
}

void MotorXY::sendLED( const uint32_t msg )
{
	if ( xLEDQ != nullptr )
	{
		xQueueSendToBack( xLEDQ, &msg, portMAX_DELAY );
	}
}


MotorXY::MotorXY(const MotorConfig &cfg, DrawControl *Draw, LpcUart *UART ) {
	if ( MXY != nullptr ) return; // already created, can't have a second control object.

	if ( UART == nullptr )
		uart = nullptr;
	else
		uart = UART;

	if ( Draw == nullptr )
		draw = nullptr;
	else
		draw = Draw;


	if ( cfg.xLEDQ == nullptr )
		xLEDQ = nullptr;
	else
		xLEDQ = cfg.xLEDQ;

	stepsperrev = cfg.steps;

	MXY = this;

	YLimitsSet = false;
	XLimitsSet = false;


	ppsslow = cfg.ppsslow; // set default safe speed at init.
	ppsfast = cfg.ppsfast;
	accelrpm = cfg.accelrpm;

	sbRIT = xSemaphoreCreateBinary();

	StepX = new DigitalIoPin(cfg.motorx.port,cfg.motorx.pin,false, false, false);
	DirX = new DigitalIoPin(cfg.dirx.port,cfg.dirx.pin,false, false, cfg.invertx);
	xinverted = cfg.invertx;

	Limit1 = new DigitalIoPin(cfg.limit1.port,cfg.limit1.pin,true, true, true);
	Limit2 = new DigitalIoPin(cfg.limit2.port,cfg.limit2.pin,true, true, true);

	Step1 = StepX; // allows X/Y flipping in isr.

	if ( cfg.motory.port == -1 ) // no Y motor
	{
		ymotor = false;

		StepY = nullptr;
		DirY = nullptr;
		Limit3 = nullptr;
		Limit4 = nullptr;
		Step2 = StepX;
	} else
	{
		ymotor = true;
		StepY = new DigitalIoPin(cfg.motory.port,cfg.motory.pin,false, false, false);
		DirY = new DigitalIoPin(cfg.diry.port,cfg.diry.pin,false, false, cfg.inverty);

		yinverted = cfg.inverty;

		Limit3 = new DigitalIoPin(cfg.limit3.port,cfg.limit3.pin,true, true, true);
		Limit4 = new DigitalIoPin(cfg.limit4.port,cfg.limit4.pin,true, true, true);
		Step2 = StepY;
	}

	LimitXL = nullptr;
	LimitXR = nullptr;
	LimitYU = nullptr;
	LimitYD = nullptr;

	Step1Limit = &MotorXY::StepAnyLimit;
	Step2Limit = &MotorXY::StepAnyLimit;
}
