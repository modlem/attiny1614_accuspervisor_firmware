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
#include <avr/sleep.h>
#include <util/delay.h>

#include "definitions.h"
#include "protocol8086.h"
#include "ticker.h"

// See Microchip TB3216 - Getting Started with USART for details
// http://ww1.microchip.com/downloads/en/AppNotes/TB3216-Getting-Started-with-USART-90003216A.pdf
// Peri.clock: 3.3MHz for 20MHz sys.clock, 2.6MHz for 16MHz sys.
#define USART0_BAUD_RATE(BAUD_RATE) ((float)(F_CPU * 64 / (16 * (float)BAUD_RATE)) + 0.5)
#define UART_BUFLEN	32
volatile uint8_t uart0_rbuf[UART_BUFLEN] = {0,};
volatile uint16_t uart0_rbuf_wpnt = 0;
volatile uint16_t uart0_rbuf_rpnt = 0;

static uint8_t _gpio_status = 0;
#define GPIO_RELAY_MASK	0x1
#define GPIO_ADC_MASK	0x2

// Actual voltage is {vbat/vacc}_volt * 0.01787109375
// Due to the lack of 32bit multiplication capability,...
// ...we have to send the raw values and let the counterpart(TX2) do the math.
uint16_t vbat_volt = 0;
uint16_t vacc_volt = 0;
uint8_t adc_state = 0;
// Low battery cut-off threshold, default: 10.5v (588)
uint16_t vbat_threshold = 588;
// Accessory power detection threshold, default: 10.5v (588)
uint16_t vacc_threshold = 588;

volatile uint8_t tx2_timeout = 0;
uint8_t pending_sleep_flag = 0 & PENDING_SLEEP_MASK;

ISR(RTC_PIT_vect)
{
	RTC.PITINTFLAGS = RTC_PI_bm;
	incTick();
	if(tx2_timeout > TX2_TIMEOUT_SEC) tx2_timeout = TX2_TIMEOUT_SEC;
	if(tx2_timeout > 0) tx2_timeout--;
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

int SLPCTRL_init(void)
{
	SLPCTRL.CTRLA |= SLPCTRL_SMODE_PDOWN_gc;
	SLPCTRL.CTRLA |= SLPCTRL_SEN_bm;
	
	return 0;
}
int GPIO_init(void)
{
	PORTA.DIR |= PIN3_bm;	// ADC enabler
	PORTB.DIR |= PIN1_bm;	// Relay
	
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
	USART0.STATUS |= USART_TXCIF_bm;
	USART0.TXDATAL = c;
	while(!(USART0.STATUS & USART_TXCIF_bm));
	
	return 0;
}

void USART0_sendStringSz(char *str)
{
	for(size_t i = 0; i < strlen(str); i++)
	{
		USART0_sendChar(str[i], NULL);
	}
}void USART0_sendBuf(uint8_t *str, uint8_t len)
{
	for(size_t i = 0; i < len; i++)
	{
		USART0_sendChar(str[i], NULL);
	}
}
void relayOn()
{
	_gpio_status |= GPIO_RELAY_MASK;
	//PORTB.OUTSET |= PIN1_bm;
	PORTB.OUT |= PIN1_bm;
	//while((PORTB.OUT & PIN1_bm) == 0);
}

void relayOff()
{
	_gpio_status &= ~GPIO_RELAY_MASK;
	//PORTB.OUTCLR |= PIN1_bm;
	PORTB.OUT &= ~PIN1_bm;
	//while((PORTB.OUT & PIN1_bm) == 1);
}

void adcOn()
{
	_gpio_status |= GPIO_ADC_MASK;
	//PORTA.OUTSET |= PIN3_bm;
	PORTA.OUT |= PIN3_bm;
	//while((PORTA.OUT & PIN3_bm) == 0);
}

void adcOff()
{
	_gpio_status &= ~GPIO_ADC_MASK;
	//PORTA.OUTCLR |= PIN3_bm;
	PORTA.OUT &= ~PIN3_bm;
	//while((PORTA.OUT & PIN3_bm) == 1);
}

int isRelayOn()
{
	return _gpio_status & GPIO_RELAY_MASK;
}

int isAdcOn()
{
	return _gpio_status & GPIO_ADC_MASK;
}

int ADC_init(void)
{
	// No inversion, no pull-up, no int., dig. buffer disable on PA1 and PA2.
	PORTA.PIN1CTRL &= ~PORT_ISC_gm;
	PORTA.PIN2CTRL &= ~PORT_ISC_gm;
	PORTA.PIN1CTRL |= PORT_ISC_INPUT_DISABLE_gc;
	PORTA.PIN2CTRL |= PORT_ISC_INPUT_DISABLE_gc;
	
	// No run in standby, full 10-bit resolution, no freerun, enable.
	ADC0.CTRLA = (1 & ADC_ENABLE_bm);
	// No accumulation. Max. value will be 0x3FF (max. of 10-bit)
	ADC0.CTRLB = ADC_SAMPNUM_ACC1_gc;
	// Big sample cap., VDD ref., DIV256 prescaler.
	// Ya, I know. It's an overkill. Better than going under.
	ADC0.CTRLC = ADC_REFSEL_VDDREF_gc | ADC_PRESC_DIV256_gc;
	// 256clk startup delay, no auto sample delay, no delay between samples.
	ADC0.CTRLD = ADC_INITDLY_DLY256_gc;
	// No window comp.
	ADC0.CTRLE = ADC_WINCM_NONE_gc;
	// 0 sample len.
	ADC0.SAMPCTRL = (0 & ADC_SAMPLEN_gm);
	// Not using evt. control
	ADC0.EVCTRL = (0 & ADC_STARTEI_bm);
	// Not using int. control
	ADC0.INTCTRL = (0 & ADC_WCMP_bm) | (0 & ADC_RESRDY_bm);
	// Clearing possible previous int. flags
	ADC0.INTFLAGS = (1 & ADC_WCMP_bm) | (1 & ADC_RESRDY_bm);
	// Halting the peripheral in debug halt
	ADC0.DBGCTRL = (0 & ADC_DBGRUN_bm);
	
	return 0;
}

void doAdcThings()
{
	if((ADC0.COMMAND & ADC_STCONV_bm) == 0)
	{
		if(ADC0.INTFLAGS & ADC_RESRDY_bm)
		{
			// Something has been done from previous conversion.
			if((ADC0.MUXPOS & ADC_MUXPOS_gm) == ADC_MUXPOS_AIN1_gc)
			{
				// VBAT
				vbat_volt = ADC0.RES;
				adc_state |= 0x1;
			}
			else if((ADC0.MUXPOS & ADC_MUXPOS_gm) == ADC_MUXPOS_AIN2_gc)
			{
				// VACC
				vacc_volt = ADC0.RES;
				adc_state |= 0x2;
			}
			ADC0.INTFLAGS = (1 & ADC_RESRDY_bm);
		}
		
		if((ADC0.MUXPOS & ADC_MUXPOS_gm) == ADC_MUXPOS_AIN1_gc)
		{
			ADC0.MUXPOS = ADC_MUXPOS_AIN2_gc;
		}
		else
		{
			ADC0.MUXPOS = ADC_MUXPOS_AIN1_gc;
		}
		ADC0.COMMAND = (1 & ADC_STCONV_bm);
		pending_sleep_flag |= PENDING_SLEEP_ADC;
	}
}

void doSwitchingThings(void)
{
	if(vacc_volt >= vacc_threshold)
	{
		tx2_timeout = TX2_TIMEOUT_SEC;
	}
	
	if(isRelayOn())
	{
		if(vacc_volt < vacc_threshold || vbat_volt < vbat_threshold)
		{
			if(tx2_timeout == 0)
			{
				relayOff();
				pending_sleep_flag &= ~PENDING_SLEEP_UART;
			}
		}
	}
	else
	{
		// Relay is always on if the accessory power presents
		if(vacc_volt >= vacc_threshold)
		{
			relayOn();
			pending_sleep_flag |= PENDING_SLEEP_UART;
		}
	}
}

int main(void)
{
	//FILE USART_stream = FDEV_SETUP_STREAM(USART0_sendChar, NULL, _FDEV_SETUP_WRITE);
	uint8_t oldTick = 255;
	uint8_t currentTick;
	
	cli();
	RTC_init();
	SLPCTRL_init();
	GPIO_init();
	USART0_init(115200);
	ADC_init();
	sei();
	// Protocol8086 parser in action
	// TODO: comment out. Debugging in progress
	//stdout = &USART_stream;
	parserInit();
	setParseDoneCallback(NULL);
	setUartSendFunc(USART0_sendBuf);
	
	pending_sleep_flag = 0 & PENDING_SLEEP_MASK;
	adcOn();

	// Static scheduling loop
    while (1)
    {
		currentTick = getCurrentTick();
		if(oldTick != currentTick)
		{
			oldTick = currentTick;
			//printf("bat: %d, acc: %d, tout: %d, adcstat:%02x, sleepflag: %02x\r\n", vbat_volt, vacc_volt, tx2_timeout, adc_state, pending_sleep_flag);
			/*if(isRelayOn())
			{
				sendCmd(CMD_HELLO, (uint8_t)((vbat_volt >> 8) & 0xFF), (uint8_t)(vbat_volt & 0xFF), (uint8_t)((vacc_volt >> 8) & 0xFF), (uint8_t)(vacc_volt & 0xFF));
				pending_sleep_flag |= PENDING_SLEEP_UART;
			}*/
		}
		
		doAdcThings();
		if((adc_state & 0x3) == 0x3)
		{
			adc_state = 0;
			doSwitchingThings();
			pending_sleep_flag &= ~PENDING_SLEEP_ADC;
		}

		if(uart0_rbuf_rpnt != uart0_rbuf_wpnt)
		{
			parseData(uart0_rbuf[uart0_rbuf_rpnt++]);
			if(uart0_rbuf_rpnt >= UART_BUFLEN) uart0_rbuf_rpnt = 0;
		}
		
		if((pending_sleep_flag & PENDING_SLEEP_MASK) == 0)
		{
			adcOff();
			_delay_ms(10);
			sleep_cpu();
			_delay_ms(10);
			adcOn();
		}
    }
}

