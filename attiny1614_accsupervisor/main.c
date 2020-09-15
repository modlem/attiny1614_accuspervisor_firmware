/*
 * attiny1614_accsupervisor.c
 *
 * Created: 2020-09-12 오전 11:48:19
 * Author : Jinsan Kwon
 */ 

#include <avr/io.h>
#include <string.h>
#include <stdio.h>

// See Microchip TB3216 - Getting Started with USART for details
// http://ww1.microchip.com/downloads/en/AppNotes/TB3216-Getting-Started-with-USART-90003216A.pdf
#define USART0_BAUD_RATE(BAUD_RATE) ((float)(F_CPU * 64 / (16 * (float)BAUD_RATE)) + 0.5)

FILE USART_stream = FDEV_SETUP_STREAM(USART0_sendChar, NULL, _FDEV_SETUP_WRITE);

int USART0_init(uint32_t baud)
{
	// TODO: Global INT disable
	
	PORTMUX.CTRLB &= ~PORTMUX_USART0_bm;
	PORTB.DIR &= ~PIN3_bm;
	PORTB.DIR |= PIN2_bm;
	
	USART0.BAUD = (uint16_t)USART0_BAUD_RATE(baud);
	USART0.CTRLA = 0x0 | USART_RXCIE_bm;
	// USART0.CTRLA |= USART_TXCIE_bm;	// Tx INT enable if needed
	USART0.CTRLB = 0x0 | USART_RXEN_bm | USART_TXEN_bm;
	USART0.CTRLC = 0x0 | USART_CHSIZE_8BIT_gc;
	// USART0.CTRLC |= USART_PMODE_ODD_gc;	// Odd parity if needed
	USART0.DBGCTRL = 0x0;	// Halt USART IP when UPDI(JTAG) debug break is fired
	return 0;
}

unsigned int USART0_readChar(uint8_t *buf, unsigned int buflen)
{
	unsigned int rtn = 0;
	while(rtn < buflen && (USART0.RXDATAH & USART_RXCIF_bm))
	{
		if(USART0.RXDATAH & USART_PERR_bm) continue;
		buf[rtn] = USART0.RXDATAL;
		rtn++;
	}
	
	return rtn;
}

int USART0_sendChar(char c)
{
	while (!(USART0.STATUS & USART_DREIF_bm))
	{
		;
	}
	USART0.TXDATAL = c;
	
	return 0;
}

void USART0_sendString(char *str)
{
	for(size_t i = 0; i < strlen(str); i++)
	{
		USART0_sendChar(str[i]);
	}
}

int main(void)
{
	USART0_init(9600);
	stdout = &USART_stream;
	
    /* Replace with your application code */
    while (1) 
    {
    }
}

