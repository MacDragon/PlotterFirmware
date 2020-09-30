/*
 * int32xy.h
 *
 *  Created on: 28 Sep 2020
 *      Author: visa
 */

#ifndef INT32XY_H_
#define INT32XY_H_

#if defined (__USE_LPCOPEN)
#if defined(NO_BOARD_LIB)
#include "chip.h"
#else
#include "board.h"
#endif
#endif

#include <cmath>

struct int32xy_t {
	int32_t x;
	int32_t y;

	int32xy_t(int32_t x=0, int32_t y=0) : x(x), y(y) { }

//	int32xy_t operator=(const int32xy_t& that) const
//	{
//	    return {that.x, that.y};
//	}

	bool operator==(const int32xy_t& that) const
	{
	    return (x == that.x && y == that.y);
	}

	bool operator!=(const int32xy_t& that) const
	{
	    return (x != that.x || y != that.y);
	}

	bool operator!=(const int32_t& that) const
	{
	    return (x != that || y != that);
	}

	bool operator>(const int32xy_t& that) const
	{
	    return (x > that.x && y > that.y);
	}

	bool operator>(const int32_t& that) const
	{
	    return (x > that && y > that);
	}

	int32xy_t& operator+=( const int32xy_t& that )
	{
		x += that.x;
		y += that.y;
		return *this;
	};

	int32xy_t& operator-=( const int32xy_t& that )
	{
		x -= that.x;
		y -= that.y;
		return *this;
	};

	int32xy_t operator-( const int32xy_t & that)
	{
		int32xy_t result;
		result.x = this->x - that.x;
		result.y = this->y - that.y;
		return result;
	}

	int32xy_t operator+( const int32xy_t& that )
	{
		int32xy_t result;
		result.x = this->x + that.x;
		result.y = this->y + that.y;
		return result;
	};

};

int32xy_t abs(const int32xy_t &val);
#endif /* INT32XY_H_ */
