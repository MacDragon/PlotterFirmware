/*
 * EEPROM.cpp
 *
 *  Created on: 3 Sep 2020
 *      Author: visa
 */

#include "EEPROM.h"
#include "string.h"
#include "ITM_write.h"


/* EEPROM Address used for storage */
#define EEPROM_ADDRESS      0x00000000

struct eepromdata {
	char 	 header[10]; // header to verify eeprom read ok.
	uint16_t PUp;
	uint16_t PDown;
	uint32_t XSize;
	uint32_t YSize;
	bool 	 XDir;
	bool	 YDir;
	uint8_t  Speed;
};



/* Tag for checking if a string already exists in EEPROM */
#define HEADERTAG       "Plotxy10"
#define HEADERTAG_SIZE     8

EEPROM::EEPROM() {
	uint8_t ret_code;

	eepromsaved = new eepromdata;

	/* Enable EEPROM clock and reset EEPROM controller */
	Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_EEPROM);
	Chip_SYSCTL_PeriphReset(RESET_EEPROM);

	/* Data to be read from EEPROM */
	ret_code = Chip_EEPROM_Read(EEPROM_ADDRESS, (uint8_t * ) eepromsaved, sizeof(eepromdata));

	/* Error checking */


	if (ret_code != IAP_CMD_SUCCESS) {
		ITM_write("EEPROM Read Failed.");
	} else ITM_write("EEPROM Read.\n");

	if ( strcmp(eepromsaved->header, HEADERTAG) != 0)
	{
		ITM_write("EEPROM Header not found, assume blank.\n");
// header didn't match, set default values and warn.
		strcpy(eepromsaved->header, HEADERTAG);
		eepromsaved->PUp=160;
		eepromsaved->PDown=90;
		eepromsaved->XSize=380;
		eepromsaved->YSize=310;
		eepromsaved->XDir=0;
		eepromsaved->YDir=0;
		eepromsaved->Speed=100;
		if ( EEPROM::save() )
			ITM_write("Default values initialised and saved.\n");
	}
}


uint16_t EEPROM::getPUp()
{
	return eepromsaved->PUp;
}

uint16_t EEPROM::getPDown()
{
	return eepromsaved->PDown;
}

uint32_t EEPROM::getXSize()
{
	return eepromsaved->XSize;
}

uint32_t EEPROM::getYSize()
{
	return eepromsaved->YSize;
}

bool	 EEPROM::getXDir()
{
	return eepromsaved->XDir;
}

bool	 EEPROM::getYDir()
{
	return eepromsaved->YDir;
}

uint8_t  EEPROM::getSpeed()
{
	return eepromsaved->Speed;
}

bool	 EEPROM::setPen(uint16_t posup, uint16_t posdown)
{
	eepromsaved->PUp = posup;
	eepromsaved->PDown = posdown;
	return save();
}

bool 	 EEPROM::setStepper(bool dirx, bool diry, uint32_t sizex, uint32_t sizey,  uint8_t speed)
{
	eepromsaved->XDir = dirx;
	eepromsaved->YDir = diry;
	eepromsaved->XSize = sizex;
	eepromsaved->YSize = sizey;
	eepromsaved->Speed = speed;
	return save();
}

bool  EEPROM::save()
{
	uint8_t ret_code;
/* Data to be written to EEPROM */
	ret_code = Chip_EEPROM_Write(EEPROM_ADDRESS, (uint8_t *) eepromsaved, sizeof(eepromdata));

	if ( ret_code != IAP_CMD_SUCCESS )
		return false;
	else
		return true;

}

EEPROM::~EEPROM() {

	delete eepromsaved;
	// save on destroy?
}

