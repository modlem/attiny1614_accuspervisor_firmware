/*
 * ticker.c
 *
 * Created: 2020-09-23 오후 2:42:17
 *  Author: Jinsan Kwon
 */ 

#include <stdint.h>
#include "ticker.h"

static uint8_t nowtick = 0;

inline void incTick(void)
{
	nowtick++;
}

uint8_t getCurrentTick(void)
{
	return nowtick;
}

uint8_t getDelta(uint8_t oldone, uint8_t newone)
{
	uint8_t rtn = 0;
	if(newone >= oldone) return newone - oldone;
	rtn = 255 - oldone;
	rtn += newone;
	rtn += 1;
	
	return rtn;
}

uint8_t getDelta_current(uint8_t old)
{
	return getDelta(old, nowtick);
}