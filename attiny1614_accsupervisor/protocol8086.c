﻿/*
 * protocol8086.c
 *
 * Created: 2020-09-23 오전 10:34:47
 *  Author: Jinsan Kwon
 */ 

#include "protocol8086.h"
#include <stdint.h>
#include <stddef.h>

#include "definitions.h"

#define _ROLE	PWR_MONITOR

static enum UART_STATE uartState;
static void (*parseDoneCallBack)(uint8_t data1, uint8_t data2, uint8_t data3, uint8_t data4, uint8_t data5) = NULL;
static void (*uartSend)(uint8_t *buf, uint8_t szBuf) = NULL;

static int parserState = 0;

void _encapsulateData(uint8_t *buf8, uint8_t one, uint8_t two, uint8_t three, uint8_t four, uint8_t five)
{
	buf8[0] = UART_STX;
	buf8[1] = one;
	buf8[2] = two;
	buf8[3] = three;
	buf8[4] = four;
	buf8[5] = five;
	buf8[6] = one + two + three + four + five;
	buf8[7] = UART_ETX;
}

void _parseDone(uint8_t data1, uint8_t data2, uint8_t data3, uint8_t data4, uint8_t data5)
{
	//static uint8_t _dont_sleep = 0;
	
	extern uint16_t vbat_volt;
	extern uint16_t vacc_volt;
	extern uint16_t vbat_threshold;
	extern uint16_t vacc_threshold;
	extern volatile uint8_t tx2_timeout;
	//extern uint8_t pending_sleep_flag;
	uint8_t sendBuf[8] = {0,};

	if(parseDoneCallBack != NULL) parseDoneCallBack(data1, data2, data3, data4, data5);
	else
	{
		/* Default Command Behaviour Parser */
#if _ROLE == PWR_MONITOR
		tx2_timeout = TX2_TIMEOUT_SEC;
		switch(data1)
		{
		case CMD_HELLO:
			_encapsulateData(sendBuf, CMD_HELLO2, (uint8_t)((vbat_volt >> 8) & 0xFF), (uint8_t)(vbat_volt & 0xFF), (uint8_t)((vacc_volt >> 8) & 0xFF), (uint8_t)(vacc_volt & 0xFF));
			if(uartSend != NULL) uartSend(sendBuf, 8);
			break;
		case CMD_HELLO2:
			break;
		case CMD_PING:
			_encapsulateData(sendBuf, CMD_PONG, (uint8_t)((vbat_volt >> 8) & 0xFF), (uint8_t)(vbat_volt & 0xFF), (uint8_t)((vacc_volt >> 8) & 0xFF), (uint8_t)(vacc_volt & 0xFF));
			if(uartSend != NULL) uartSend(sendBuf, 8);
			break;
		case CMD_PONG:
			break;
		case CMD_BKUP:
			break;
		case CMD_BKOK:
			break;
		case CMD_HALT:
			break;
		case CMD_HTOK:
			break;
		case CMD_STAT:
			_encapsulateData(sendBuf, CMD_STAT_RSP, (uint8_t)((vbat_volt >> 8) & 0xFF), (uint8_t)(vbat_volt & 0xFF), (uint8_t)((vacc_volt >> 8) & 0xFF), (uint8_t)(vacc_volt & 0xFF));
			if(uartSend != NULL) uartSend(sendBuf, 8);
			break;
		case CMD_STAT_RSP:
			break;
		case CMD_GET_THRESH:
			_encapsulateData(sendBuf, CMD_GET_THRESH_RSP, (uint8_t)((vbat_threshold >> 8) & 0xFF), (uint8_t)(vbat_threshold & 0xFF), (uint8_t)((vacc_threshold >> 8) & 0xFF), (uint8_t)(vacc_threshold & 0xFF));
			if(uartSend != NULL) uartSend(sendBuf, 8);
			break;
		case CMD_GET_THRESH_RSP:
			break;
		case CMD_SET_THRESH:
			vbat_threshold = ((uint16_t)data2) << 8;
			vbat_threshold += data3;
			vacc_threshold = ((uint16_t)data4) << 8;
			vacc_threshold += data5;
			_encapsulateData(sendBuf, CMD_SET_THRESH_RSP, (uint8_t)((vbat_threshold >> 8) & 0xFF), (uint8_t)(vbat_threshold & 0xFF), (uint8_t)((vacc_threshold >> 8) & 0xFF), (uint8_t)(vacc_threshold & 0xFF));
			if(uartSend != NULL) uartSend(sendBuf, 8);
			break;
		case CMD_SET_THRESH_RSP:
			break;
#if 0
		case CMD_DONT_SLEEP:
			_dont_sleep = 1;
			_encapsulateData(sendBuf, CMD_DONT_SLEEP_RSP, _dont_sleep, 0, 0, 0);
			if(uartSend != NULL) uartSend(sendBuf, 8);
			break;
		case CMD_DONT_SLEEP_RSP:
			break;
		case CMD_OKAY_SLEEP:
			_dont_sleep = 0;
			_encapsulateData(sendBuf, CMD_OKAY_SLEEP_RSP, _dont_sleep, 0, 0, 0);
			if(uartSend != NULL) uartSend(sendBuf, 8);
			break;
		case CMD_OKAY_SLEEP_RSP:
			break;
#endif
		default:
			break;
		}

#if 0		
		if(_dont_sleep)
		{
			pending_sleep_flag |= PENDING_SLEEP_UART;
		}
		else
		{
			pending_sleep_flag &= ~PENDING_SLEEP_UART;
		}
#endif
#elif _ROLE == DVR_RECORDER
#else
#endif
	}
}


void parserInit()
{
	parserState = 1;
}

void setParseDoneCallback(void (*func)(uint8_t, uint8_t, uint8_t, uint8_t, uint8_t))
{
	parseDoneCallBack = func;
}

void setUartSendFunc(void (*func)(uint8_t *, uint8_t))
{
	uartSend = func;
}

void sendCmd(uint8_t cmd, uint8_t one, uint8_t two, uint8_t three, uint8_t four)
{
	uint8_t sendBuf[8] = {0,};
	_encapsulateData(sendBuf, cmd, one, two, three, four);
	if(uartSend != NULL) uartSend(sendBuf, 8);
}

void parseData(uint8_t data)
{
	static uint8_t databuf[8] = {0,};
	static uint32_t recvdcnt = 0;
	uint8_t tmpChksum = 0;

	if(parserState)
	{
		switch(uartState)
		{
		case UART_STATE_IDLE:
			if(data == UART_STX)
			{
				recvdcnt = 0;
				uartState = UART_STATE_RECEIVING;
				databuf[recvdcnt++] = data;
			}
			break;
		case UART_STATE_RECEIVING:
			if(recvdcnt < 8)
			{
				databuf[recvdcnt++] = data;
				if(recvdcnt == 8)
				{
					if(data == UART_ETX)
					{
						tmpChksum = databuf[1] + databuf[2] + databuf[3] + databuf[4] + databuf[5];
						if(tmpChksum == databuf[6])
						{
							_parseDone(databuf[1], databuf[2], databuf[3], databuf[4], databuf[5]);
						}
					}
					//recvdcnt = 0;
					uartState = UART_STATE_IDLE;
				}
			}
			else
			{
				// Highly unlikely case
				//recvdcnt = 0;
				uartState = UART_STATE_IDLE;
			}
			break;
		default:
			break;
		}
	}
}