/*
 * helper.h
 *
 *  Created on: 22 Sep 2020
 *      Author: visa
 */

#ifndef HELPER_H_
#define HELPER_H_

#if defined (__USE_LPCOPEN)
#if defined(NO_BOARD_LIB)
#include "chip.h"
#else
#include "board.h"
#endif
#endif

#include "math.h"

#ifndef M_PI
#define M_PI		3.14159265358979323846
#endif

#define degToRad(angleInDegrees) ((angleInDegrees) * M_PI / 180.0)
#define radToDeg(angleInRadians) ((angleInRadians) * 180.0 / M_PI)

struct PinMap {
	int port; /* set to -1 to indicate unused pin */
	int pin;  /* set to -1 to indicate unused pin  */
};

char * getnumstr( int32_t num );
void strtoupper(char* str);

class helper {
public:
	helper();
	virtual ~helper();
};

#endif /* HELPER_H_ */
