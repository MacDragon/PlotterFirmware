/*
 * int32xy.cpp
 *
 *  Created on: 28 Sep 2020
 *      Author: visa
 */

#include "int32xy.h"

int32xy_t abs(const int32xy_t &val)
{
	return {abs(val.x), abs(val.y)};
}

