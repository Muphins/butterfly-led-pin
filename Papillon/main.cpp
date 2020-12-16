/*******************************************************************************
*                                  INCLUDES                                    *
*******************************************************************************/
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdlib.h>
#include <avr/sleep.h>
#include <avr/pgmspace.h>

#include "MMA8453.h"
#include "timers.h"
#include "filters.h"
#include "rng.h"
/*******************************************************************************
*								  TYPEDEFS									   *
*******************************************************************************/
struct tSplit{
	unsigned LSB :8;
	unsigned MSB :8;
};

union t16_t{	// used to get individual bytes of a 16-bit integer without bitshift
	uint16_t value;
	tSplit	 split;
};

enum tAnim{
	AnimNone		= 0,
	AnimSnake		= 1,
	AnimWings		= 2,
	AnimBallTilt	= 3,
	AnimStarlight	= 4
	};

/*******************************************************************************
*								  PROTOTYPES								   *
*******************************************************************************/
void colorHSV(uint16_t hue, uint8_t sat, uint8_t val, uint8_t* r, uint8_t* g, uint8_t* b);
inline uint8_t cumul(uint8_t value, uint8_t plus);
void neoPixelSendPixel(uint8_t b);
inline int16_t sq(int16_t n){return n*n;}
/*******************************************************************************
*								  CONSTANTS									   *
*******************************************************************************/
// const uint8_t sine[128] PROGMEM = {
// 		0,  0,  1,  1,  2,  4,  6,  8, 10, 12, 15, 18, 22, 25, 29, 34,
// 		38, 42, 47, 52, 57, 63, 68, 74, 80, 86, 92, 98,104,110,116,123,
// 		129,135,142,148,154,160,166,172,178,184,189,195,200,205,210,215,
// 		219,224,228,231,235,238,241,244,246,248,250,252,253,254,255,255,
// 		255,255,254,253,252,250,248,246,244,241,238,235,231,228,224,219,
// 		215,210,205,200,195,189,184,178,172,166,160,154,148,142,135,129,
// 		123,116,110,104, 98, 92, 86, 80, 74, 68, 63, 57, 52, 47, 42, 38,
// 		34, 29, 25, 22, 18, 15, 12, 10,  8,  6,  4,  2,  1,  1,  0,  0};

#define STRIP_LEN		18
#define MEAN_THRESHOLD	3

#define LED_FADE_SPEED	4
#define ANIM_STARS_LEN	13
#define ANIM_SNAKE_LEN	1
#define ANIM_SWIPE_LEN	2
#define PROXI_LIGHT_LEN	16

#define RIGHT_BOT_START_INDEX	0
#define LEFT_BOT_START_INDEX	(RIGHT_BOT_START_INDEX+4)
#define LEFT_TOP_START_INDEX	(LEFT_BOT_START_INDEX+4)
#define RIGHT_TOP_START_INDEX	(LEFT_TOP_START_INDEX+5)

/*******************************************************************************
*								  VARIABLES									   *
*******************************************************************************/

/* General */
bool g_firstBoot = true;
/* Time base related */
uint8_t g_cycleCounter=0;
bool g_dataReady = false;

/* Power saving related */
bool g_eco = false;
bool g_sleepCoolDown = false;
uint8_t g_sleepEngage = 0;

/* LED strip related */
bool g_LedsOn = true;
uint8_t stripRed__[STRIP_LEN];
uint8_t stripGreen[STRIP_LEN];
uint8_t stripBlue_[STRIP_LEN];
uint8_t stripBright[STRIP_LEN];
uint16_t stripHue[STRIP_LEN];

/* LEDs positions */
uint8_t leftBotToTop[9] =  { 6, 5, 7, 4, 9, 8,10,12,11};
uint8_t rightBotToTop[9] = { 2, 1, 3, 0,14,13,15,17,16};
int8_t ledXPos[STRIP_LEN] = { 22, 49, 35, 22,-22,-49,-35,-22,-16,-38,-49,-50,-28, 16, 38, 49, 50, 28};
int8_t ledYPos[STRIP_LEN] = { 19, 25, 55, 36, 19, 25, 55, 36,-16,-14,-31,-50,-34,-16,-14,-31,-50,-34};

/* Accelerometer data */
static int8_t accX = 0;
static int8_t accY = 0;
static int8_t accZ = 0;
uint8_t g_accelIntSource = 0;	// Interrupt source of accelerometer

/* Filters */
cLPF lowPassX(2);		// low-freq low-pass
cLPF lowPassY(2);
cLPF lowPassZ(2);
cLPF lowPass2X(128);	// Hi-freq low pass
cLPF lowPass2Y(128);
//cLPF lowPass2Z(48);
uint8_t filteredX = 0;
uint8_t filteredY = 0;
uint8_t filteredZ = 0;

/* LED animation related */
cRng rando;	// random number generator
uint8_t g_animationCounter=0;
int8_t g_animationDir = 1;
/* proxiLight: Look-up table for LED brightness to distance calculated using a²+b² */
uint8_t proxiLight[PROXI_LIGHT_LEN] = {128,113,98,85,72,61,50,41,32,25,18,13,8,5,2,1};
/*******************************************************************************
*                                    CODE                                      *
*******************************************************************************/
int main(void)
{
	uint8_t i;
	uint8_t indexLed=0;
	int8_t posX, posY;
	/* Setup AVR */
	ACSR |= (1<<ACD);	// disable analog comparator
	PRR = 1<<PRTIM1 | 1<<PRTIM0 | 1<<PRUSI | 1<<PRADC;	// disable peripherals
	
	MCUCR |= 1<<PUD;	// disable pull-up
 	DDRB |= 1<<PB3 | 1<<PB4;	// outputs
	PORTB |= 1<<PB3;			// Enable LED
 	PORTB &= ~(1<<PB4);			// reset LED_CMD
	
	sei();				// enable interrupts
	
	/* Setup accelerometer */
	SoftI2CInit();
	accel::init();
	accel::enableTransientIntLatch();
	
    while (1) 
    {
		/* generates random numbers outside of the main program,
		   allowing the non-predictability of the numbers generated inside the main part
		   since its duration depends on the acceleration which depend on external events */
		rando.run();
		
		if(!g_eco && !g_sleepCoolDown){
			static uint8_t sumAxes = 0;
			static uint16_t hue = 0;
			static uint8_t brightness = 0;
			
			if(g_dataReady){					// Synchronized to accelerometer data-rate
				/* count timeBase */
 				g_cycleCounter++;
				if(g_cycleCounter == 4){		// 25 tick per second at 100Hz data rate
					g_cycleCounter = 0;
					g_animationCounter ++;
				}
				
				/* Get acceleration data */
				g_dataReady = false;
				accel::getAcc(&accX, &accY, &accZ);
				
				/* low-pass-filter acceleration data */
 				filteredX = abs(accX - lowPassX.run(accX));
 				filteredY = abs(accY - lowPassY.run(accY));
 				filteredZ = abs(accZ - lowPassZ.run(accZ));
				
				/* compute activation/Sleep threshold */
				sumAxes = filteredX + filteredY + filteredZ;
			
				if(sumAxes > MEAN_THRESHOLD){
					hue += 32;
					if(!g_cycleCounter && brightness < 128) brightness +=1;
				}else{
					if(!g_cycleCounter && brightness > 0) brightness --;
				}
				
				/* Compute Starlight animation */
// 				if(g_animationCounter == ANIM_STARS_LEN){
// 					g_animationCounter = 0;
// 					indexLed = rando.run();
// 					/* Faster and more precise than a division */
// 					/* Maps 18 LEDs to a random range(0;255) */
// 					if     (indexLed <  15)	indexLed= 0;
// 					else if(indexLed <  29)	indexLed= 1;
// 					else if(indexLed <  43)	indexLed= 2;
// 					else if(indexLed <  57)	indexLed= 3;
// 					else if(indexLed <  71)	indexLed= 4;
// 					else if(indexLed <  86)	indexLed= 5;
// 					else if(indexLed < 100)	indexLed= 6;
// 					else if(indexLed < 114)	indexLed= 7;
// 					else if(indexLed < 128)	indexLed= 8;
// 					else if(indexLed < 142)	indexLed= 9;
// 					else if(indexLed < 156)	indexLed=10;
// 					else if(indexLed < 171)	indexLed=11;
// 					else if(indexLed < 185)	indexLed=12;
// 					else if(indexLed < 199)	indexLed=13;
// 					else if(indexLed < 213)	indexLed=14;
// 					else if(indexLed < 227)	indexLed=15;
// 					else if(indexLed < 241)	indexLed=16;
// 					else					indexLed=17;
// 					stripBright[indexLed] = brightness;
// 					stripHue[indexLed] = hue;
// 				}
				
				/* Compute Wings animation */
				posX = lowPassY.read() - lowPass2Y.run(accY);	// Band-pass filter
				posY = lowPassX.read() - lowPass2X.run(accX);
				uint8_t saturation = 255;
				if(!g_cycleCounter && g_LedsOn){
					int16_t hueXShift = ((int16_t)posX)<<8;
					int16_t hueYShift = ((int16_t)posY)<<8;
					uint8_t bright = abs(posX) + abs(posY);
					//bright = bright<<1;
					saturation -= bright;
					if(brightness > 0){
						if(bright < (255 - (brightness>>1))){
							bright += (brightness>>1);
						}else{
							bright = 255;
						}
					}else{
						bright = 0;
					}
					uint16_t wingHue = hue + hueXShift + hueYShift;	// Bot Right
					for(i=RIGHT_BOT_START_INDEX;i<LEFT_BOT_START_INDEX; i++){
						stripBright[i] = bright;
						stripHue[i] = wingHue;
					}
					wingHue = hue - hueXShift + hueYShift;	// Bot Left
					for(;i<LEFT_TOP_START_INDEX; i++){
						stripBright[i] = bright;
						stripHue[i] = wingHue;
					}
					wingHue = hue - hueXShift - hueYShift;	// Top Left
					for(;i<RIGHT_TOP_START_INDEX; i++){
						stripBright[i] = bright;
						stripHue[i] = wingHue;
					}
					wingHue = hue + hueXShift - hueYShift;	// Top Right
					for(;i<STRIP_LEN; i++){
						stripBright[i] = bright;
						stripHue[i] = wingHue;
					}
				}
				
				/* Compute Ball-tilt animation */
// 				if(!g_cycleCounter){
// 					if(posX >  63) posX =  63;	// Set limits for X,Y accel position
// 					if(posX < -63) posX = -63;
// 					if(posY >  63) posY =  63;
// 					if(posY < -63) posY = -63;
// 					for(i=0; i<STRIP_LEN; i++){
// 						uint8_t x = abs(posX - ledXPos[i]);	// Relative position of LED to acceleration
// 						uint8_t y = abs(posY - ledYPos[i]);
// 						t16_t distComp;
// 						distComp.value = ((x*x)+(y*y))<<1;	// Pythagorean formula (+ left shift for more precision)
// 						if(distComp.split.MSB < PROXI_LIGHT_LEN){
// 							uint8_t bright = proxiLight[distComp.split.MSB];	// indexing to "proxiLight" is more precise (there is more values) thanks to the left-shift
// 							bright = ((uint16_t)(bright*brightness))>>7;		// Scale LED brightness to global brightness
// 							if(bright > stripBright[i]){						// Only affects less bright LEDs
// 								//colorHSV(0, 255, bright, &stripRed__[i], &stripGreen[i], &stripBlue_[i]);
// 								stripBright[i] = bright;
// 								stripHue[i] = hue + 32768;
// 							}
// 						}/*else{
// 							stripRed__[i] = 0;
// 							stripGreen[i] = 0;
// 							stripBlue_[i] = 0;
// 						}*/
// 					}
//				}
				/* Compute Snake animation */
// 				if(g_animationCounter == ANIM_SNAKE_LEN){
// 					g_animationCounter = 0;
// 					stripBright[indexLed] = brightness;
// 					indexLed++;
// 					if(indexLed == STRIP_LEN) indexLed = 0;
// 				}
				/* compute Swipe animation */
// 				if(g_animationCounter == ANIM_SWIPE_LEN){
// 					g_animationCounter = 0;
// 					stripBright[leftBotToTop[indexLed]]  = brightness;
// 					stripBright[rightBotToTop[indexLed]] = brightness;
// 					indexLed += g_animationDir;
// 					if(indexLed == STRIP_LEN/2-1) g_animationDir = -1;
// 					if(indexLed == 0) g_animationDir = 1;
// 				}
				/* Compute colors */
				if(!g_cycleCounter){
					for(i=0; i<STRIP_LEN; i++){
						colorHSV(stripHue[i], saturation, stripBright[i], &stripRed__[i], &stripGreen[i], &stripBlue_[i]);
// 						if(stripBright[i] > LED_FADE_SPEED){
// 							stripBright[i]-=LED_FADE_SPEED;
// 						}else{
// 							stripBright[i]=0;
// 						}
					}
				}
				/* Send colors to Pixels */
				if(brightness == 0 && !g_LedsOn){
					g_sleepEngage ++;
				}else{
					g_LedsOn = true;
					g_sleepEngage = 0;
				
					for(i=0; i<STRIP_LEN; i++){
						neoPixelSendPixel(stripGreen[i]);	// Green
						neoPixelSendPixel(stripRed__[i]);	// Red
						neoPixelSendPixel(stripBlue_[i]);	// Blue
					}
					if(brightness == 0) g_LedsOn = false;
				}
			}
		}
		/* Sleep management */
		if(g_sleepEngage == 255 || g_eco || g_firstBoot){
			g_firstBoot = false;
			if(!g_eco){
				g_eco = true;
				g_sleepCoolDown = true;
 				accel::enableAutoSleep();
				g_sleepEngage = 0;
				DDRB = 0;								// set PORTB to Hi-Z
				PORTB &= ~(1<<PB3 | 1<<PB4);
			}
			WDTCR = 1<<WDIE | 1<<WDCE | 1<<WDE | 3;	// enable watchdog  timer and interrupt for .125sec
			set_sleep_mode(SLEEP_MODE_PWR_DOWN);	// SLEEP_MODE_PWR_DOWN
	 		sleep_mode();							// sleep enable
		/* Wake-up */
		}
		/* Interrupt management */
		if(!(PINB & 1<<PB1)){
			/* Check int sources from Accelerometer */
			if(accel::checkIntSource(&g_accelIntSource) == I2cOk){
				if(g_accelIntSource & accel::IntTRANS){		// Transient event
					accel::readIntTransient();				// Unlatch TRANS int event
					if(g_eco && !g_sleepCoolDown){			// Conditions for exiting sleep mode
						DDRB |= (1<<PB3 | 1<<PB4);	// Outputs
						PORTB |= 1<<PB3;			// Enable LEDs
						g_eco = false;
						accel::disableAutoSleep();
					}else{									// eliminate Trans_INT that happens when accelerometer goes to sleep
						g_sleepCoolDown = false;
					}
				}
				if(g_accelIntSource & accel::IntDRDY){
					g_dataReady = true;
				}
			}
		}
    }
}
/*******************************************************************************
*                                 FUNCTIONS                                    *
*******************************************************************************/
void colorHSV(uint16_t hue, uint8_t sat, uint8_t val, uint8_t* r, uint8_t* g, uint8_t* b)	// ~934.132 cycles => 116.767µs @ 8MHz
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

void neoPixelSendPixel(uint8_t b)
{
	cli();
	
	DDRB |= 1<<PB4;
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