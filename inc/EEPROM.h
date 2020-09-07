/*
 * EEPROM.h
 *
 *  Created on: 3 Sep 2020
 *      Author: visa
 */

#ifndef EEPROM_H_
#define EEPROM_H_

#if defined (__USE_LPCOPEN)
#if defined(NO_BOARD_LIB)
#include "chip.h"
#else
#include "board.h"
#endif
#endif

class EEPROM {
public:
	EEPROM();
	bool save();
	uint16_t getPUp();
	uint16_t getPDown();
	uint32_t getXSize();
	uint32_t getYSize();
	bool	 getXDir();
	bool	 getYDir();
	uint8_t  getSpeed();
	bool	 setPen(uint16_t posup, uint16_t posdown);
	bool 	 setStepper(bool dirx, bool diry, uint32_t sizex, uint32_t sizey,  uint8_t speed);
private:
	struct eepromdata * eepromsaved;
	virtual ~EEPROM();
};

#endif /* EEPROM_H_ */
