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

void colorHSV(uint16_t hue, uint8_t sat, uint8_t val, uint8_t* r, uint8_t* g, uint8_t* b);
inline uint8_t cumul(uint8_t value, uint8_t plus);
inline void resetMean();
void neoPixelTest(uint8_t b);
inline int16_t sq(int16_t n){return n*n;}

// const uint8_t sine[128] PROGMEM = {
// 		0,  0,  1,  1,  2,  4,  6,  8, 10, 12, 15, 18, 22, 25, 29, 34,
// 		38, 42, 47, 52, 57, 63, 68, 74, 80, 86, 92, 98,104,110,116,123,
// 		129,135,142,148,154,160,166,172,178,184,189,195,200,205,210,215,
// 		219,224,228,231,235,238,241,244,246,248,250,252,253,254,255,255,
// 		255,255,254,253,252,250,248,246,244,241,238,235,231,228,224,219,
// 		215,210,205,200,195,189,184,178,172,166,160,154,148,142,135,129,
// 		123,116,110,104, 98, 92, 86, 80, 74, 68, 63, 57, 52, 47, 42, 38,
// 		34, 29, 25, 22, 18, 15, 12, 10,  8,  6,  4,  2,  1,  1,  0,  0};

#define MEAN_THRESHOLD	8
#define MEAN_TAB_LEN	40
uint8_t accMeanTab[MEAN_TAB_LEN];
uint8_t accMeanIndex = 0;
uint16_t accMean = 0;

uint8_t g_counter=0;
bool g_eco = false;
bool g_sleepCoolDown = false;

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
	
	resetMean();
	
	sei();				// enable interrupts
	
	SoftI2CInit();
	accel::init();
	accel::enableTransientIntLatch();
	accel::enableAutoSleep();
	
 	DDRB |= 1<<PB3 | 1<<PB4;	// outputs
	PORTB |= 1<<PB3;			// Enable LED
 	PORTB &= ~(1<<PB4);			// reset LED_CMD
	
    while (1) 
    {
		if(!g_eco && !g_sleepCoolDown){
			static uint8_t sumAxes = 0;
			static uint16_t hue = 0;
			static uint8_t brightness = 0;
			
			accel::move(&accX, &accY, &accZ);
// 			accX = (accX & 0xFE)>>1;
// 			accY = (accY & 0xFE)>>1;
// 			accZ = (accZ & 0xFE)>>1;

			sumAxes = accX + accY + accZ;
 			accMean -= accMeanTab[accMeanIndex];
			accMean += sumAxes;
			accMeanTab[accMeanIndex] = sumAxes;
			accMeanIndex++;
			if(accMeanIndex == MEAN_TAB_LEN) accMeanIndex = 0;
// 			cumulX = cumul(cumulX, accX);
// 			cumulY = cumul(cumulY, accY);
// 			cumulZ = cumul(cumulZ, accZ);
 			g_counter = (g_counter + 1)%12;	// Decrement cumul only 1/5th of the time
			
			if(accMean > MEAN_THRESHOLD){
				hue += 8;
				if(!g_counter && brightness < 96) brightness +=2;
			}else{
				if(!g_counter && brightness > 0) brightness --;
			}
// 			if(!g_counter && accMean > 0) accMean -=1;
// 			if(accMean > MEAN_TAB_LEN * 255 * 3) accMean = 0;
			
			colorHSV(hue, 255, brightness, &cumulX, &cumulY, &cumulZ);
			cumulX += sq(2);
			if(brightness == 0){
				sleepEngage ++;
				DDRB = 0;
				PORTB &= ~(1<<PB3 | 1<<PB4);
			}else{
				sleepEngage = 0;
				DDRB |= (1<<PB3 | 1<<PB4);
				PORTB |= 1<<PB3;			// Enable LED
				
				tI2cStatus i2cStatusTmp = SoftI2CStatus();
				neoPixelTest(cumulX);	// Green
				neoPixelTest(cumulY);	// Red
				neoPixelTest(cumulZ);	// Blue
				if(i2cStatusTmp != I2cIdle && i2cStatusTmp != I2cOk){
					neoPixelTest(0);
					neoPixelTest(255);
					neoPixelTest(0);
					_delay_ms(1);
				}else{
					neoPixelTest(cumulZ);
					neoPixelTest(cumulX);
					neoPixelTest(cumulY);
				}
				
			}
		}
		if(sleepEngage == 255 || g_eco){
			if(!g_eco){
				g_eco = true;
				g_sleepCoolDown = true;
 				accel::enableAutoSleep();
// 				_delay_ms(30);
// 				accel::checkIntSource();				// Unlatch int event
				sleepEngage = 0;
				DDRB = 0;								// set PORTB to Hi-Z
			}
			WDTCR = 1<<WDIE | 1<<WDCE | 1<<WDE | 3;	// enable watchdog  timer and interrupt for .125sec
//			accel::init();
// 			accel::enableTransientIntLatch();
// 			_delay_ms(25);
// 			accel::checkIntSource();				// Unlatch int event
		/* slow system-clock */
// 			CLKPR = 0x80;							// Initialize CLKPR write sequence
// 			CLKPR = 0x08;							// Set system prescaler to 1/...
			set_sleep_mode(SLEEP_MODE_PWR_DOWN);	// SLEEP_MODE_PWR_DOWN
	 		sleep_mode();							// sleep enable
		/* Wake-up */
		}
		if(!(PINB & 1<<PB1) && accel::autoSleep){
			if(g_eco && !g_sleepCoolDown){
			/* Normal system-clock */
				CLKPR = 0x80;							// Initialize CLKPR write sequence
				CLKPR = 0x00;							// Set system prescaler to 1/...
				g_eco = false;
//				_delay_ms(30);
				accel::disableAutoSleep();
//				accel::disableTransientIntLatch();
				DDRB |= 1<<PB3 | 1<<PB4;				// Outputs
//				resetMean();
			}else{
				g_sleepCoolDown = false;
			}
			accel::checkIntSource();				// Unlatch int event
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

inline void resetMean()
{
	for(uint8_t index=0; index < MEAN_TAB_LEN; index ++){
		accMeanTab[index]=0;
	}
	accMeanIndex = 0;
	accMean = 0;
}

void colorHSV(uint16_t hue, uint8_t sat, uint8_t val, uint8_t* r, uint8_t* g, uint8_t* b)
{

	// Remap 0-65535 to 0-1529. Pure red is CENTERED on the 64K rollover;
	// 0 is not the start of pure red, but the midpoint...a few values above
	// zero and a few below 65536 all yield pure red (similarly, 32768 is the
	// midpoint, not start, of pure cyan). The 8-bit RGB hexcone (256 values
	// each for red, green, blue) really only allows for 1530 distinct hues
	// (not 1536, more on that below), but the full unsigned 16-bit type was
	// chosen for hue so that one's code can easily handle a contiguous color
	// wheel by allowing hue to roll over in either direction.
	hue = (hue * 1530L + 32768) / 65536;
	// Because red is centered on the rollover point (the +32768 above,
	// essentially a fixed-point +0.5), the above actually yields 0 to 1530,
	// where 0 and 1530 would yield the same thing. Rather than apply a
	// costly modulo operator, 1530 is handled as a special case below.

	// So you'd think that the color "hexcone" (the thing that ramps from
	// pure red, to pure yellow, to pure green and so forth back to red,
	// yielding six slices), and with each color component having 256
	// possible values (0-255), might have 1536 possible items (6*256),
	// but in reality there's 1530. This is because the last element in
	// each 256-element slice is equal to the first element of the next
	// slice, and keeping those in there this would create small
	// discontinuities in the color wheel. So the last element of each
	// slice is dropped...we regard only elements 0-254, with item 255
	// being picked up as element 0 of the next slice. Like this:
	// Red to not-quite-pure-yellow is:        255,   0, 0 to 255, 254,   0
	// Pure yellow to not-quite-pure-green is: 255, 255, 0 to   1, 255,   0
	// Pure green to not-quite-pure-cyan is:     0, 255, 0 to   0, 255, 254
	// and so forth. Hence, 1530 distinct hues (0 to 1529), and hence why
	// the constants below are not the multiples of 256 you might expect.

	// Convert hue to R,G,B (nested ifs faster than divide+mod+switch):
	if(hue < 510) {         // Red to Green-1
		*b = 0;
		if(hue < 255) {       //   Red to Yellow-1
			*r = 255;
			*g = hue;            //     g = 0 to 254
			} else {              //   Yellow to Green-1
			*r = 510 - hue;      //     r = 255 to 1
			*g = 255;
		}
		} else if(hue < 1020) { // Green to Blue-1
		*r = 0;
		if(hue <  765) {      //   Green to Cyan-1
			*g = 255;
			*b = hue - 510;      //     b = 0 to 254
			} else {              //   Cyan to Blue-1
			*g = 1020 - hue;     //     g = 255 to 1
			*b = 255;
		}
		} else if(hue < 1530) { // Blue to Red-1
		*g = 0;
		if(hue < 1275) {      //   Blue to Magenta-1
			*r = hue - 1020;     //     r = 0 to 254
			*b = 255;
			} else {              //   Magenta to Red-1
			*r = 255;
			*b = 1530 - hue;     //     b = 255 to 1
		}
		} else {                // Last 0.5 Red (quicker than % operator)
		*r = 255;
		*g = *b = 0;
	}

	// Apply saturation and value to R,G,B, pack into 32-bit result:
	uint32_t v1 =   1 + val; // 1 to 256; allows >>8 instead of /255
	uint16_t s1 =   1 + sat; // 1 to 256; same reason
	uint8_t  s2 = 255 - sat; // 255 to 0
	*r = (((((*r * s1) >> 8) + s2) * v1) & 0xff00) >> 8;
	*g = (((((*g * s1) >> 8) + s2) * v1) & 0xff00) >> 8;
	*b = (((((*b * s1) >> 8) + s2) * v1) & 0xff00) >> 8;
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