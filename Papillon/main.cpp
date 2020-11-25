/*
 * Papillon.cpp
 *
 * Created: 19/11/2020 14:56:54
 * Author : Be3
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdlib.h>
#include <avr/sleep.h>
#include <avr/pgmspace.h>
//#include "twi.h"
#include "MMA8453.h"

inline uint8_t cumul(uint8_t value, uint8_t plus);
void neoPixelTest(uint8_t b);

// const uint8_t sine[128] PROGMEM = {
// 		0,  0,  1,  1,  2,  4,  6,  8, 10, 12, 15, 18, 22, 25, 29, 34,
// 		38, 42, 47, 52, 57, 63, 68, 74, 80, 86, 92, 98,104,110,116,123,
// 		129,135,142,148,154,160,166,172,178,184,189,195,200,205,210,215,
// 		219,224,228,231,235,238,241,244,246,248,250,252,253,254,255,255,
// 		255,255,254,253,252,250,248,246,244,241,238,235,231,228,224,219,
// 		215,210,205,200,195,189,184,178,172,166,160,154,148,142,135,129,
// 		123,116,110,104, 98, 92, 86, 80, 74, 68, 63, 57, 52, 47, 42, 38,
// 		34, 29, 25, 22, 18, 15, 12, 10,  8,  6,  4,  2,  1,  1,  0,  0};

uint8_t g_counter=0;
bool g_eco = false;


int main(void)
{
	static uint8_t sleepEngage = 0;
	static uint8_t accX = 0;
	static uint8_t accY = 0;
	static uint8_t accZ = 0;
	static uint8_t cumulX = 0;
	static uint8_t cumulY = 0;
	static uint8_t cumulZ = 0;
	
	MCUCR |= 1<<PUD;	// disable pull-up
	ACSR |= (1<<ACD);	// disable analog comparator
	PRR = 1<<PRTIM1 | 1<<PRTIM0 | 1<<PRUSI | 1<<PRADC;	// disable peripherals
	
	sei();				// enable interrupts
	
	SoftI2CInit();
	accel::init();
	accel::enableTransientIntLatch();
	
 	DDRB |= 1<<PB3 | 1<<PB4;	// outputs
	PORTB |= 1<<PB3;			// Enable LED
 	PORTB &= ~(1<<PB4);			// reset LED_CMD
	
    while (1) 
    {
		if(!g_eco){
			accel::move(&accX, &accY, &accZ);
			accX = (accX & 0xFE)>>1;
			accY = (accY & 0xFE)>>1;
			accZ = (accZ & 0xFE)>>1;
			
			cumulX = cumul(cumulX, accX);
			cumulY = cumul(cumulY, accY);
			cumulZ = cumul(cumulZ, accZ);
			g_counter = (g_counter + 1)%5;	// Decrement cumul only 1/5th of the time
			
			if(cumulX + cumulY + cumulZ == 0){
				sleepEngage ++;
				DDRB = 0;
				PORTB &= ~(1<<PB3 | 1<<PB4);
			}else{
				sleepEngage = 0;
				DDRB |= (1<<PB3 | 1<<PB4);
				PORTB |= 1<<PB3;			// Enable LED
				neoPixelTest(cumulX);
				neoPixelTest(cumulY);
				neoPixelTest(cumulZ);
				neoPixelTest(cumulY);
				neoPixelTest(cumulZ);
				neoPixelTest(cumulX);
			}
		}
		if(sleepEngage == 255 || g_eco){
			sleepEngage = 0;
			g_eco = true;
			WDTCR = 1<<WDIE | 1<<WDCE | 1<<WDE | 3;	// enable watchdog  timer and interrupt for .25sec
			DDRB = 0;								// set PORTB to Hi-Z
//			accel::init();
// 			accel::enableTransientIntLatch();
// 			_delay_ms(25);
// 			accel::checkIntSource();				// Unlatch int event
		/* slow system-clock */
			CLKPR = 0x80;							// Initialize CLKPR write sequence
			CLKPR = 0x08;							// Set system prescaler to 1/...
			set_sleep_mode(SLEEP_MODE_PWR_DOWN);	// SLEEP_MODE_PWR_DOWN
	 		sleep_mode();							// sleep enable
		/* Wake-up */
		}
		if(!(PINB & 1<<PB1)){
		/* Normal system-clock */
			CLKPR = 0x80;							// Initialize CLKPR write sequence
			CLKPR = 0x00;							// Set system prescaler to 1/...
			g_eco = false;
			accel::checkIntSource();				// Unlatch int event
//			accel::disableTransientIntLatch();
			DDRB |= 1<<PB3 | 1<<PB4;				// Outputs
		}
    }
}

inline uint8_t cumul(uint8_t value, uint8_t plus)
{
//	static dummy = 255-value;
	if(value > 0 && g_counter == 0) value --;
	if(plus < (255-value)){
		value += plus;
	}else value = 255;
	return value;
}

void neoPixelTest(uint8_t b)
{
	cli();
	
	DDRB |= 1<<PB4;
//	volatile uint8_t *port = &PORTB;
	volatile uint8_t n1, n2 = 0;  // First, next bits out
	uint8_t hi = PORTB |  (1<<PB4);
	uint8_t lo = PORTB & ~(1<<PB4);
	n1 = lo;
	if(b & 0x80) n1 = hi;

	asm volatile(
	// Bit 7:
	"out  %[port] , %[hi]"    "\n\t" // 1    PORT = hi
	"mov  %[n2]   , %[lo]"    "\n\t" // 1    n2   = lo
	"out  %[port] , %[n1]"    "\n\t" // 1    PORT = n1
	"rjmp .+0"                "\n\t" // 2    nop nop
	"sbrc %[byte] , 6"        "\n\t" // 1-2  if(b & 0x40)
	"mov %[n2]   , %[hi]"    "\n\t" // 0-1   n2 = hi
	"out  %[port] , %[lo]"    "\n\t" // 1    PORT = lo
	"rjmp .+0"                "\n\t" // 2    nop nop
//	"nop"					  "\n\t" // 1	 nop
	// Bit 6:
	"out  %[port] , %[hi]"    "\n\t" // 1    PORT = hi
	"mov  %[n1]   , %[lo]"    "\n\t" // 1    n1   = lo
	"out  %[port] , %[n2]"    "\n\t" // 1    PORT = n2
	"rjmp .+0"                "\n\t" // 2    nop nop
	"sbrc %[byte] , 5"        "\n\t" // 1-2  if(b & 0x20)
	"mov %[n1]   , %[hi]"    "\n\t" // 0-1   n1 = hi
	"out  %[port] , %[lo]"    "\n\t" // 1    PORT = lo
	"rjmp .+0"                "\n\t" // 2    nop nop
//	"nop"					  "\n\t" // 1	 nop
	// Bit 5:
	"out  %[port] , %[hi]"    "\n\t" // 1    PORT = hi
	"mov  %[n2]   , %[lo]"    "\n\t" // 1    n2   = lo
	"out  %[port] , %[n1]"    "\n\t" // 1    PORT = n1
	"rjmp .+0"                "\n\t" // 2    nop nop
	"sbrc %[byte] , 4"        "\n\t" // 1-2  if(b & 0x10)
	"mov %[n2]   , %[hi]"    "\n\t" // 0-1   n2 = hi
	"out  %[port] , %[lo]"    "\n\t" // 1    PORT = lo
	"rjmp .+0"                "\n\t" // 2    nop nop
//	"nop"					  "\n\t" // 1	 nop
	// Bit 4:
	"out  %[port] , %[hi]"    "\n\t" // 1    PORT = hi
	"mov  %[n1]   , %[lo]"    "\n\t" // 1    n1   = lo
	"out  %[port] , %[n2]"    "\n\t" // 1    PORT = n2
	"rjmp .+0"                "\n\t" // 2    nop nop
	"sbrc %[byte] , 3"        "\n\t" // 1-2  if(b & 0x08)
	"mov %[n1]   , %[hi]"    "\n\t" // 0-1   n1 = hi
	"out  %[port] , %[lo]"    "\n\t" // 1    PORT = lo
	"rjmp .+0"                "\n\t" // 2    nop nop
//	"nop"					  "\n\t" // 1	 nop
	// Bit 3:
	"out  %[port] , %[hi]"    "\n\t" // 1    PORT = hi
	"mov  %[n2]   , %[lo]"    "\n\t" // 1    n2   = lo
	"out  %[port] , %[n1]"    "\n\t" // 1    PORT = n1
	"rjmp .+0"                "\n\t" // 2    nop nop
	"sbrc %[byte] , 2"        "\n\t" // 1-2  if(b & 0x04)
	"mov %[n2]   , %[hi]"    "\n\t" // 0-1   n2 = hi
	"out  %[port] , %[lo]"    "\n\t" // 1    PORT = lo
	"rjmp .+0"                "\n\t" // 2    nop nop
//	"nop"					  "\n\t" // 1	 nop
	// Bit 2:
	"out  %[port] , %[hi]"    "\n\t" // 1    PORT = hi
	"mov  %[n1]   , %[lo]"    "\n\t" // 1    n1   = lo
	"out  %[port] , %[n2]"    "\n\t" // 1    PORT = n2
	"rjmp .+0"                "\n\t" // 2    nop nop
	"sbrc %[byte] , 1"        "\n\t" // 1-2  if(b & 0x02)
	"mov %[n1]   , %[hi]"    "\n\t" // 0-1   n1 = hi
	"out  %[port] , %[lo]"    "\n\t" // 1    PORT = lo
	"rjmp .+0"                "\n\t" // 2    nop nop
//	"nop"					  "\n\t" // 1	 nop
	// Bit 1:
	"out  %[port] , %[hi]"    "\n\t" // 1    PORT = hi
	"mov  %[n2]   , %[lo]"    "\n\t" // 1    n2   = lo
	"out  %[port] , %[n1]"    "\n\t" // 1    PORT = n1
	"rjmp .+0"                "\n\t" // 2    nop nop
	"sbrc %[byte] , 0"        "\n\t" // 1-2  if(b & 0x01)
	"mov %[n2]   , %[hi]"    "\n\t" // 0-1   n2 = hi
	"out  %[port] , %[lo]"    "\n\t" // 1    PORT = lo
	"rjmp .+0"                "\n\t" // 2    nop nop
//	"nop"					  "\n\t" // 1	 nop
	// Bit 0:
	"out  %[port] , %[hi]"    "\n\t" // 1    PORT = hi
	"mov  %[n1]   , %[lo]"    "\n\t" // 1    n1   = lo
	"out  %[port] , %[n2]"    "\n\t" // 1    PORT = n2
	"rjmp .+0"				  "\n\t" // 2    b = *ptr++
	"sbrc %[byte] , 7"        "\n\t" // 1-2  if(b & 0x80)
	"mov %[n1]   , %[hi]"     "\n\t" // 0-1   n1 = hi
	"out  %[port] , %[lo]"    "\n\t" // 1    PORT = lo
	"rjmp .+0"	              "\n"   // 2    while(i) (Z flag set above)
//	"nop"					  "\n"   // 1	 nop
	: [byte]  "+r" (b),
	[n1]    "+r" (n1),
	[n2]    "+r" (n2)
	: [port]   "I" (_SFR_IO_ADDR(PORTB)),
	[hi]     "r" (hi),
	[lo]     "r" (lo)
	);
	
	sei();
}

ISR(WDT_vect)
{
	WDTCR |= 1<<WDCE | 1<<WDE;
	WDTCR = 0x00;
}