/*
 * definitions.h
 *
 * Created: 2020-09-24 오후 3:10:02
 *  Author: Jinsan Kwon
 */ 


#ifndef DEFINITIONS_H_
#define DEFINITIONS_H_


#define TX2_TIMEOUT_SEC		30

#define PENDING_SLEEP_UART	0x1
#define PENDING_SLEEP_ADC	0x2
#define PENDING_SLEEP_MASK	(PENDING_SLEEP_UART | PENDING_SLEEP_ADC)


#endif /* DEFINITIONS_H_ */