/*
 * attiny1614_accsupervisor.c
 *
 * Created: 2020-09-12 오전 11:48:19
 * Author : Jinsan Kwon
 */ 

#include <avr/io.h>
#include <string.h>
#include <stdio.h>
#include <avr/interrupt.h>

// See Microchip TB3216 - Getting Started with USART for details
// http://ww1.microchip.com/downloads/en/AppNotes/TB3216-Getting-Started-with-USART-90003216A.pdf
// Peri.clock: 3.3MHz for 20MHz sys.clock, 2.6MHz for 16MHz sys.
#define USART0_BAUD_RATE(BAUD_RATE) ((float)(3333333 * 64 / (16 * (float)BAUD_RATE)) + 0.5)
#define UART_BUFLEN	32
volatile uint8_t uart0_rbuf[UART_BUFLEN] = {0,};
volatile uint16_t uart0_rbuf_wpnt = 0;
volatile uint16_t uart0_rbuf_rpnt = 0;
volatile uint8_t _pit_flag = 0;

ISR(RTC_PIT_vect)
{
	RTC.PITINTFLAGS = RTC_PI_bm;
	_pit_flag = 1;
}

ISR(USART0_RXC_vect)
{
	while(USART0.RXDATAH & USART_RXCIF_bm)
	{
		if(USART0.RXDATAH & USART_PERR_bm) continue;
		uart0_rbuf[uart0_rbuf_wpnt++] = USART0.RXDATAL;
		if(uart0_rbuf_wpnt >= UART_BUFLEN) uart0_rbuf_wpnt = 0;
	}
	
	if(uart0_rbuf_rpnt == uart0_rbuf_wpnt)
	{
		uart0_rbuf_rpnt = uart0_rbuf_wpnt + 1;
		if(uart0_rbuf_rpnt >= UART_BUFLEN) uart0_rbuf_rpnt = 0;
	}
}

// See Microchip TB3213 - Getting Started with RTC for details
// http://ww1.microchip.com/downloads/en/AppNotes/TB3213-Getting-Started-with-RTC-90003213A.pdf
int RTC_init(void)
{
	/* Initialize RTC: */
	while (RTC.STATUS > 0)
	{
		; /* Wait for all register to be synchronized */
	}
	
	/* 1kHz Internal Oscillator (OSCULP32K/DIV32) */
	RTC.CLKSEL = RTC_CLKSEL_INT1K_gc;
	/* Run in debug: enabled */
	RTC.DBGCTRL = RTC_DBGRUN_bm;
	RTC.PITINTCTRL = RTC_PI_bm; /* Periodic Interrupt: enabled */
	RTC.PITCTRLA = RTC_PERIOD_CYC1024_gc /* RTC Clock Cycles 1024 */ | RTC_PITEN_bm; /* Enable: enabled */
	
	return 0;
}

int USART0_init(uint32_t baud)
{	
	int8_t sigrow_val = SIGROW.OSC20ERR3V; // read signed error
	int32_t baud_reg_val = 0; // ideal BAUD register value
	
	PORTMUX.CTRLB &= ~PORTMUX_USART0_bm;
	PORTB.DIR &= ~PIN3_bm;
	PORTB.DIR |= PIN2_bm;
	
	baud_reg_val = USART0_BAUD_RATE(baud);
	baud_reg_val *= (1024 + sigrow_val); // sum resolution + error
	baud_reg_val /= 1024; // divide by resolution
	
	USART0.BAUD = (int16_t) baud_reg_val; // set adjusted baud rate
	USART0.CTRLA = 0x0 | USART_RXCIE_bm;
	// USART0.CTRLA |= USART_TXCIE_bm;	// Tx INT enable if needed
	USART0.CTRLC = 0x0 | USART_CHSIZE_8BIT_gc;
	// USART0.CTRLC |= USART_PMODE_ODD_gc;	// Odd parity if needed
	USART0.DBGCTRL = 0x0;	// Halt USART IP when UPDI(JTAG) debug break is fired
	
	USART0.CTRLB = 0x0 | USART_RXEN_bm | USART_TXEN_bm;
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

int USART0_sendChar(char c, struct __file *notused)
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
		USART0_sendChar(str[i], NULL);
	}
}
// FILE USART_stream = FDEV_SETUP_STREAM(USART0_sendChar, NULL, _FDEV_SETUP_WRITE);
int main(void)
{
	cli();
	RTC_init();
	USART0_init(115200);
	sei();
	// stdout = &USART_stream;
	
    /* Replace with your application code */
    while (1)
    {
		if(_pit_flag)
		{
			USART0_sendChar('t', NULL);
			_pit_flag = 0;
		}
		
		if(uart0_rbuf_rpnt != uart0_rbuf_wpnt)
		{
			// TODO: put data into protocol parser
			//uart0_rbuf[uart0_rbuf_rpnt++]
			if(uart0_rbuf_rpnt >= UART_BUFLEN) uart0_rbuf_rpnt = 0;
		}
    }
}

