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
	AnimStatic		= 1,
	AnimSnake		= 2,
	AnimWings		= 3,
	AnimBallTilt	= 4,
	AnimStarlight	= 5
	};

/*******************************************************************************
*								  PROTOTYPES								   *
*******************************************************************************/
void computeAnimations(uint8_t startIndex, uint8_t endIndex);
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

/* Accelerometer filters ratios and activation threshold */
#define ACTIVATION_THRESHOLD	3
#define LOWPASS1_RATIO			3
#define LOWPASS2_RATIO			100

/* Animation activation */
#define ANIM_STARLIGHT
//#define ANIM_BALLTILT
#define BALL_TIL_HIPASS
#define ANIM_STATIC_FRENCHFLAG

/* Animations Duration */
#define CYCLE_LENGTH		4	// 25Hz with Data-rate of 100Hz
#define LED_FADE_SPEED		4
#define ANIM_STARLIGHT_LEN	13
#define ANIM_SNAKE_LEN		1
#define ANIM_SWIPE_LEN		2

/* Calculation subsets' start cycle */
#define SUBSET0_CYCLE	0
#define SUBSET1_CYCLE	1
#define SUBSET2_CYCLE	2
#define SUBSET3_CYCLE	3

/* LED strip tables length */
#define STRIP_LEN		18
#define PROXI_LIGHT_LEN	16
/* Wings' LEDs subsets */
#define RIGHT_BOT_START_INDEX	0
#define LEFT_BOT_START_INDEX	(RIGHT_BOT_START_INDEX+4)
#define LEFT_TOP_START_INDEX	(LEFT_BOT_START_INDEX+4)
#define RIGHT_TOP_START_INDEX	(LEFT_TOP_START_INDEX+5)
/* Starlight LED count */
#define STARLIGHT_LED_COUNT	5

#define SATURATION_DEFAULT		255
#define SATURATION_SATRLIGHT	128

#define HUE_RED		0
#define HUE_BLUE	(43520)

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

/* LEDs positions */
int8_t ledXPos[STRIP_LEN] = { 22, 49, 35, 22,-22,-49,-35,-22,-16,-38,-49,-50,-28, 16, 38, 49, 50, 28};
int8_t ledYPos[STRIP_LEN] = { 19, 25, 55, 36, 19, 25, 55, 36,-16,-14,-31,-50,-34,-16,-14,-31,-50,-34};

/* Accelerometer data */
static int8_t accX = 0;
static int8_t accY = 0;
static int8_t accZ = 0;
uint8_t g_accelIntSource = 0;	// Interrupt source of accelerometer
uint8_t sumAxes = 0;
int8_t posX, posY;	// X,Y position of the acceleration data

/* Filters */
cLPF lowPassX(3);		// low-freq low-pass
cLPF lowPassY(3);
cLPF lowPassZ(3);
cLPF lowPass2X(100);	// Hi-freq low pass
cLPF lowPass2Y(100);
//cLPF lowPass2Z(48);
uint8_t filteredX = 0;
uint8_t filteredY = 0;
uint8_t filteredZ = 0;

/* LED animation related */
uint16_t hue = 0;
uint8_t brightness = 0;
// For starlight animation
cRng rando;	// random number generator
uint16_t starlightHue[STARLIGHT_LED_COUNT];
uint8_t startlightHueIndex = 0;
// For Ball-tilt animation
uint8_t x;			// relative ball to LED x-distance
uint8_t y;			// relative ball to LED y-distance
t16_t distComp;		// relative ball to LED vector length
uint16_t hueBall;
// For wings animation
int16_t hueXShift;
int16_t hueYShift;
/* proxiLight: Look-up table for LED brightness to distance calculated using a²+b² */
uint8_t proxiLight[PROXI_LIGHT_LEN] = {128,113,98,85,72,61,50,41,32,25,18,13,8,5,2,1};
/* LED strip related */
bool g_LedsOn = true;
uint8_t stripRed__[STRIP_LEN];
uint8_t stripGreen[STRIP_LEN];
uint8_t stripBlue_[STRIP_LEN];
uint8_t stripBright[STRIP_LEN];
tAnim stripAnim[STRIP_LEN];

/*******************************************************************************
*                                    CODE                                      *
*******************************************************************************/
int main(void)
{
	uint8_t i;
	uint8_t indexLed=0;
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
			/*********************************************************/
			/* Run main loop every time accelerometer data are ready */
			/*********************************************************/
			if(g_dataReady){					// Synchronized to accelerometer data-rate (cf. MMA_CTRL_REG1_STB)
				/* count timeBase */
 				g_cycleCounter++;
				if(g_cycleCounter == CYCLE_LENGTH){		// 25 tick per second at 100Hz data rate
					g_cycleCounter = 0;
				}
				
				/* Get acceleration data */
				g_dataReady = false;
				accel::getAcc(&accY, &accX, &accZ); // X & Y are exchanged relative to the chip placement on the PCB
				
				/* Filter acceleration data */
 				filteredX = abs(accX - lowPassX.run(accX));
 				filteredY = abs(accY - lowPassY.run(accY));
 				filteredZ = abs(accZ - lowPassZ.run(accZ));
				lowPass2X.run(accX);
				lowPass2Y.run(accY);
				
				/* compute activation/Sleep threshold and global brightness */
				sumAxes = filteredX + filteredY + filteredZ;
				if(sumAxes > ACTIVATION_THRESHOLD){
					if(!g_cycleCounter && brightness < 128) brightness ++;
				}else{
					if(!g_cycleCounter && brightness > 0) brightness --;
				}
				/**********************************************/
				/* Compute animations and send colors to LEDs */
				/**********************************************/
				hue += 32;
				/* divide the calculations in 4 subset to spread the time spent 
				   on it and allow future lengthening of the LED string. Those
				   subsets are run when g_cycleCounter has the corresponding
				   value */
				if(g_cycleCounter == SUBSET0_CYCLE){
				//============================	/* Compute animation's next cycle */
					/* Band-pass filters X,Y acceleration data */
#ifdef BALL_TIL_HIPASS
					posX = lowPassX.read() - lowPass2X.run(accX);
					posY = lowPassY.read() - lowPass2Y.run(accY);
#else
					posX = -lowPass2X.read();
					posY = -lowPass2Y.read();
#endif
					// Hue-shift calculated before posX,Y have been clipped
					hueXShift = ((int16_t)posX)<<8;	// Hue-shift depending of hi-pass filtered X-axis acceleration
					hueYShift = ((int16_t)posY)<<8;	// Hue-shift depending of hi-pass filtered Y-axis acceleration
#ifdef ANIM_STARLIGHT
					/* Compute Starlight animation */
					static uint8_t animCounterStarlight = 0;
					animCounterStarlight++;
 					if(animCounterStarlight == ANIM_STARLIGHT_LEN){
 						animCounterStarlight = 0;
 						indexLed = rando.run();
 						/* Faster and more precise than a division */
 						/* Maps 18 LEDs to a random range(0;255) */
 						if     (indexLed <  15)	indexLed= 0;
 						else if(indexLed <  29)	indexLed= 1;
 						else if(indexLed <  43)	indexLed= 2;
 						else if(indexLed <  57)	indexLed= 3;
 						else if(indexLed <  71)	indexLed= 4;
 						else if(indexLed <  86)	indexLed= 5;
 						else if(indexLed < 100)	indexLed= 6;
 						else if(indexLed < 114)	indexLed= 7;
 						else if(indexLed < 128)	indexLed= 8;
 						else if(indexLed < 142)	indexLed= 9;
 						else if(indexLed < 156)	indexLed=10;
 						else if(indexLed < 171)	indexLed=11;
 						else if(indexLed < 185)	indexLed=12;
 						else if(indexLed < 199)	indexLed=13;
 						else if(indexLed < 213)	indexLed=14;
 						else if(indexLed < 227)	indexLed=15;
 						else if(indexLed < 241)	indexLed=16;
 						else					indexLed=17;
 						stripBright[indexLed] = brightness;
 						starlightHue[startlightHueIndex] = hue;
						stripAnim[indexLed] = (tAnim)((uint8_t)AnimStarlight + startlightHueIndex);
						
						startlightHueIndex++;
						if(startlightHueIndex == STARLIGHT_LED_COUNT) startlightHueIndex = 0;
 					}
#endif
#if defined(ANIM_BALLTILT) || defined(ANIM_STATIC_FRENCHFLAG)
					if(posX >  63) posX =  63;	// Set limits for X,Y accel position
					if(posX < -63) posX = -63;
					if(posY >  63) posY =  63;
					if(posY < -63) posY = -63;
					hueBall = hue + 32768;
#endif
				/* Calculation subsets start here */
				//============================	/* Compute Bottom-Right wing (LEDs[0..3]) */
					computeAnimations(0, RIGHT_BOT_START_INDEX);
						
				}else if(g_cycleCounter == SUBSET1_CYCLE){
				//============================	/* Compute Bottom-Left wing (LEDs[4..7]) */
					computeAnimations(RIGHT_BOT_START_INDEX, LEFT_BOT_START_INDEX);
					
				}else if(g_cycleCounter == SUBSET2_CYCLE){
				//============================	/* Compute Top-Left wing (LEDs[8..12]) */
					computeAnimations(LEFT_BOT_START_INDEX, LEFT_TOP_START_INDEX);
					
				}else if(g_cycleCounter == SUBSET3_CYCLE){
				//============================	/* Compute Top-Right wing (LEDs[13..17]) */
					computeAnimations(LEFT_TOP_START_INDEX, STRIP_LEN);
					
				//============================	/* Send pixels */
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
void computeAnimations(uint8_t startIndex, uint8_t endIndex)
{
	uint8_t& i = startIndex;
	for(; i<endIndex; i++){
		if(stripAnim[i] < AnimStarlight){
			stripAnim[i] = AnimNone;
		}
#ifdef ANIM_BALLTILT
		// Ball-Tilt animation
		x = abs(posX - ledXPos[i]);	// Relative position of LED to acceleration
		y = abs(posY - ledYPos[i]);
		distComp.value = ((x*x)+(y*y))<<1;	// Pythagorean formula (+ left shift for more precision)
		if(distComp.split.MSB < PROXI_LIGHT_LEN){
			uint8_t bright = proxiLight[distComp.split.MSB];				// indexing to "proxiLight" is more precise (there is more values) thanks to the left-shift
			bright = ((uint16_t)(bright*brightness))>>7;					// Scale LED brightness to global brightness
			if(bright > stripBright[i] || stripAnim[i] <= AnimBallTilt){	// Handles superposition of animations
				stripAnim[i] = AnimBallTilt;
				colorHSV(hueBall, /*SATURATION_DEFAULT*/0, bright, &stripRed__[i], &stripGreen[i], &stripBlue_[i]);
				stripBright[i] = bright;
				stripHue[i] = hueBall;
			}
		}
#endif // ANIM_BALLTILT
		// Wings animation
#ifdef ANIM_STATIC_FRENCHFLAG
		// Display french flag
		if(stripAnim[i] <= AnimStatic){
			stripAnim[i] = AnimStatic;
			if(ledXPos[i] < -22){
				colorHSV(HUE_BLUE, SATURATION_DEFAULT, brightness, &stripRed__[i], &stripGreen[i], &stripBlue_[i]);
			}else if(ledXPos[i] < 28){
				colorHSV(0, 0, brightness, &stripRed__[i], &stripGreen[i], &stripBlue_[i]);
			}else{
				colorHSV(HUE_RED, SATURATION_DEFAULT, brightness, &stripRed__[i], &stripGreen[i], &stripBlue_[i]);
			}
		}
#endif // ANIM_STATIC_FRENCHFLAG
		// Set color to unused LEDs
		if(stripAnim[i] == AnimNone){
			stripRed__[i] = 0;
			stripGreen[i] = 0;
			stripBlue_[i] = 0;
		// Render Starlight LEDs
		}else if(stripAnim[i] >= AnimStarlight){
			uint8_t starIndex = stripAnim[i] - AnimStarlight;
			colorHSV(starlightHue[starIndex], SATURATION_SATRLIGHT, stripBright[i], &stripRed__[i], &stripGreen[i], &stripBlue_[i]);
			if(stripBright[i] > LED_FADE_SPEED){
				stripBright[i]-=LED_FADE_SPEED;
			}else{
				stripBright[i]=0;
				stripAnim[i] = AnimNone;
			}
		}
	}
}

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