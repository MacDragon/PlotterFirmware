/*
 * DebugPrint.h
 *
 *  Created on: 29 Sep 2020
 *      Author: visa
 */

#ifndef DEBUGPRINT_H_
#define DEBUGPRINT_H_

#include "LpcUart.h"

struct debugEvent {
	const char *format;
	uint32_t data[3];
};

bool SetupDebugPrint( LpcUart * uartin );

void debug(const char *format, uint32_t d1, uint32_t d2, uint32_t d3);

#endif /* DEBUGPRINT_H_ */
