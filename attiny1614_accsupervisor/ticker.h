/*
 * ticker.h
 *
 * Created: 2020-09-23 오후 2:42:46
 *  Author: Jinsan Kwon
 */ 


#ifndef TICKER_H_
#define TICKER_H_

#include <stdint.h>

inline void incTick(void);
uint8_t getCurrentTick(void);
uint8_t getDelta(uint8_t oldone, uint8_t newone);
uint8_t getDelta_current(uint8_t old);


#endif /* TICKER_H_ */