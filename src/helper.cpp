/*
 * helper.cpp
 *
 *  Created on: 22 Sep 2020
 *      Author: visa
 */

#include "helper.h"
#include <string>
#include <string.h>
#include <cstring>

// declare outside function so can return pointer
char numstr[13];

char * getnumstr( const int32_t num )
{
	uint8_t i = 0;
	int32_t temp=num;
	bool negative = false;

	if ( temp < 0 )
	{
		negative=true;
		temp = ~temp+1; // convert the two's complement back to positive.

	}

	for(i=1; i<=11; i++)
	{
		numstr[12-i] = (uint8_t) ((temp % 10UL) + '0');
		temp/=10;
	}
	numstr[i] = '\0';
	for ( i=1;i<11&&numstr[i]=='0';++i);

	if ( negative )
	{
		i--;
		numstr[i]='-';
	}

	return &numstr[i];
}



void strtoupper(char* str)
{
	while ((*str = (char) toupper(*str))) str++;
}

helper::helper() {
	// TODO Auto-generated constructor stub

}

helper::~helper() {
	// TODO Auto-generated destructor stub
}

