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
#include "filters.h"
#include "rng.h"
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
/* LEDs PORT PINS */
#define LED_POWER	(1<<PB3)
#define LED_CMD		(1<<PB4)
/* Accelerometer filters ratios and activation threshold */
#define ACTIVATION_THRESHOLD	3
#define LOWPASS1_RATIO			3
#define LOWPASS2_RATIO			100
#define HALF_G					63

/* Animation activation */
#define ANIM_STARLIGHT
//#define ANIM_BALLTILT
//#define BALL_TILT_HIPASS
//#define ANIM_STATIC_FRENCHFLAG
#define ANIM_WINGS

/* Animations Duration */
#define CYCLE_LENGTH		4	// 25Hz with Data-rate of 100Hz
#define LED_FADE_SPEED		3	// The amount of starlight's brightness decrease every cycle
#define ANIM_STARLIGHT_LEN	13	// The animation triggers every ANIM_STARLIGHT_LEN full cycles

/* Calculation subsets' start cycle */
/* Subsets are spaced to allow for the maximum time between two calculations */
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
#define STARLIGHT_LED_COUNT	10	// MUST be <= (255 - AnimStarlight)

#define SATURATION_DEFAULT		255
#define SATURATION_SATRLIGHT	192
#define BRIGHTNESS_DEFAULT		128
#define BRIGHTNESS_STARLIGHT	255

#define HUE_RED		0			//   0° * 65536 / 360
#define HUE_YELLOW	10923		//  60°
#define HUE_GREEN	21845		// 120°
#define HUE_CYAN	32768		// 180°
#define HUE_BLUE	43691		// 240°
#define HUE_MAGENTA	54613		// 300°

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
*								  VARIABLES									   *
*******************************************************************************/

/* General */
uint8_t i;
bool g_firstBoot = true;
/* Time base related */
uint8_t g_cycleCounter=0;
bool g_dataReady = false;

/* Power saving related */
bool g_eco = false;
bool g_sleepCoolDown = false;
uint8_t g_sleepEngage = 0;

/* LEDs positions */
const int8_t ledXPos[STRIP_LEN] PROGMEM = { 22, 49, 35, 22,-22,-49,-35,-22,-16,-38,-49,-50,-28, 16, 38, 49, 50, 28};
const int8_t ledYPos[STRIP_LEN] PROGMEM = { 19, 25, 55, 36, 19, 25, 55, 36,-16,-14,-31,-50,-34,-16,-14,-31,-50,-34};

/* Accelerometer data */
static int8_t accX = 0;
static int8_t accY = 0;
static int8_t accZ = 0;
uint8_t g_accelIntSource = 0;	// Interrupt source of accelerometer
uint8_t sumAxes = 0;
int8_t posXHiPass, posYHiPass;	// X,Y hi-pass filtered position of the acceleration data
int8_t posXSmooth, posYSmooth;	// X,Y position of the smoothed acceleration data

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
uint8_t starlightLedIndex=0;
uint16_t starlightHue[STARLIGHT_LED_COUNT];	// Stores the hue for the currently active starlight LEDs
uint8_t startlightHueIndex = 0;
// For Ball-tilt animation
uint8_t xBall;		// relative ball-to-LED x-distance
uint8_t yBall;		// relative ball-to-LED y-distance
t16_t distComp;		// relative ball-to-LED vector length
uint16_t hueBall;	// Hue for the ball-tilt animation
// For wings animation
int16_t hueXShift;
int16_t hueYShift;
uint16_t hueWing;
uint8_t wingBright;
uint8_t wingRed;
uint8_t wingGreen;
uint8_t wingBlue;
/* proxiLight: Look-up table for LED brightness to distance calculated using a²+b² */
const uint8_t proxiLight[PROXI_LIGHT_LEN] PROGMEM = {128,113,98,85,72,61,50,41,32,25,18,13,8,5,2,1};
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
	/* Setup AVR */
	ACSR |= (1<<ACD);	// disable analog comparator
	PRR = 1<<PRTIM1 | 1<<PRTIM0 | 1<<PRUSI | 1<<PRADC;	// disable peripherals
	
	MCUCR |= 1<<PUD;	// disable pull-up
 	DDRB |= LED_POWER | LED_CMD;	// outputs
	PORTB |= LED_POWER;			// Enable LED
 	PORTB &= ~(LED_CMD);			// reset LED_CMD
	
	sei();				// enable interrupts
	
	/* Setup accelerometer */
	SoftI2CInit();
	accel::init();
	accel::enableTransientIntLatch();
	
    while (1)
    {
		/* generates random numbers outside of the main loop, allowing the non-predictability of the 
		   numbers generated when needed. It is possible because the duration of the main loop varies 
		   depending on the acceleration which depend on external events */
		rando.run();
		
		if(!g_eco && !g_sleepCoolDown){
			/*********************************************************/
			/* Run main loop every time accelerometer data are ready */
			/*********************************************************/
			/* The accelerometer's clock is precise enough to rely on it and save power and execution 
			   cycles by not using an internal timer. */
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
				
				/* Test activation/Sleep threshold and compute global brightness */
				sumAxes = filteredX + filteredY + filteredZ;
				if(sumAxes > ACTIVATION_THRESHOLD){
					if(!g_cycleCounter && brightness < BRIGHTNESS_DEFAULT) brightness ++;
				}else{
					if(!g_cycleCounter && brightness > 0) brightness --;
				}
				/**********************************************/
				/* Compute animations and send colors to LEDs */
				/**********************************************/
				hue += 32;
				/* divide the calculations in 4 subsets to spread the time spent 
				   on it and allow future lengthening of the LED string. Those
				   subsets are run when g_cycleCounter has the corresponding
				   value */
				if(g_cycleCounter == SUBSET0_CYCLE){
				//============================	/* Compute animation's next cycle */
					/* Band-pass filters X,Y acceleration data */
					posXHiPass = lowPassX.read() - lowPass2X.read();
					posYHiPass = lowPassY.read() - lowPass2Y.read();
					posXSmooth = -lowPass2X.read();
					posYSmooth = -lowPass2Y.read();
					// Hue-shift calculated before posX,Y have been clipped
					hueXShift = ((int16_t)posXHiPass)<<8;	// Hue-shift depending of hi-pass filtered X-axis acceleration
					hueYShift = ((int16_t)posYHiPass)<<8;	// Hue-shift depending of hi-pass filtered Y-axis acceleration
#ifdef ANIM_STARLIGHT
					/* Compute Starlight animation */
					static uint8_t animCounterStarlight = 0;
					animCounterStarlight++;
 					if(animCounterStarlight == ANIM_STARLIGHT_LEN){	// Test for starlight animation's trigger
 						animCounterStarlight = 0;
 						starlightLedIndex = rando.run();
 						/* Faster and more precise than a division */
 						/* Maps 18 LEDs to a random range(0;255) */
 						if     (starlightLedIndex <  15)	starlightLedIndex= 0;
 						else if(starlightLedIndex <  29)	starlightLedIndex= 1;
 						else if(starlightLedIndex <  43)	starlightLedIndex= 2;
 						else if(starlightLedIndex <  57)	starlightLedIndex= 3;
 						else if(starlightLedIndex <  71)	starlightLedIndex= 4;
 						else if(starlightLedIndex <  86)	starlightLedIndex= 5;
 						else if(starlightLedIndex < 100)	starlightLedIndex= 6;
 						else if(starlightLedIndex < 114)	starlightLedIndex= 7;
 						else if(starlightLedIndex < 128)	starlightLedIndex= 8;
 						else if(starlightLedIndex < 142)	starlightLedIndex= 9;
 						else if(starlightLedIndex < 156)	starlightLedIndex=10;
 						else if(starlightLedIndex < 171)	starlightLedIndex=11;
 						else if(starlightLedIndex < 185)	starlightLedIndex=12;
 						else if(starlightLedIndex < 199)	starlightLedIndex=13;
 						else if(starlightLedIndex < 213)	starlightLedIndex=14;
 						else if(starlightLedIndex < 227)	starlightLedIndex=15;
 						else if(starlightLedIndex < 241)	starlightLedIndex=16;
 						else								starlightLedIndex=17;
						/* scale BRIGHTNESS_STARLIGHT to global brightness */
 						stripBright[starlightLedIndex] = (uint16_t)(BRIGHTNESS_STARLIGHT * brightness) >> 7;	// The bitshift works only if BRIGHTNESS_DEFAULT is a power of 2
						/* What follows is a small trick to reduce ram usage by using a small table 
						   "starlightHue[]" to store the hue of any active starlight. Since their number 
						   will never exceed STARLIGHT_LED_COUNT, there is no need to store the hue 
						   of each LED in the string. The Starlight effect being the one with the 
						   highest priority, we can use the table "stripAnim" to store the index to 
						   "starlightHue" containing the hue for this LED. */
 						starlightHue[startlightHueIndex] = hue;
						stripAnim[starlightLedIndex] = (tAnim)((uint8_t)AnimStarlight + startlightHueIndex);
						startlightHueIndex++;
						if(startlightHueIndex == STARLIGHT_LED_COUNT) startlightHueIndex = 0;
 					}
#endif
#if defined(ANIM_WINGS)
					if(brightness > 0){ // test to avoid having wingBright > 0 while brightness is 0 and sleepMode is being engaged
						wingBright = (brightness >> 3) + abs(posYHiPass) + abs(posYHiPass);
					}else{
						wingBright = 0;
					}
#endif
#if defined(ANIM_BALLTILT) || defined(ANIM_STATIC_FRENCHFLAG)
					if(posXHiPass >  HALF_G) posXHiPass =  HALF_G;	// Set limits for X,Y accel position
					if(posXHiPass < -HALF_G) posXHiPass = -HALF_G;
					if(posYHiPass >  HALF_G) posYHiPass =  HALF_G;
					if(posYHiPass < -HALF_G) posYHiPass = -HALF_G;
					hueBall = hue + 32768;
#endif
				/* Calculation subsets start here */
				//============================	/* Compute Bottom-Right wing (LEDs[0..3]) */
					hueWing = hue + hueXShift + hueYShift + 32768;
					computeAnimations(RIGHT_BOT_START_INDEX, LEFT_BOT_START_INDEX);
						
				}else if(g_cycleCounter == SUBSET1_CYCLE){
				//============================	/* Compute Bottom-Left wing (LEDs[4..7]) */
					hueWing = hue - hueXShift + hueYShift + 32768;
					computeAnimations(LEFT_BOT_START_INDEX, LEFT_TOP_START_INDEX);
					
				}else if(g_cycleCounter == SUBSET2_CYCLE){
				//============================	/* Compute Top-Left wing (LEDs[8..12]) */
					hueWing = hue - hueXShift - hueYShift + 32768;
					computeAnimations(LEFT_TOP_START_INDEX, RIGHT_TOP_START_INDEX);
					
				}else if(g_cycleCounter == SUBSET3_CYCLE){
				//============================	/* Compute Top-Right wing (LEDs[13..17]) */
					hueWing = hue + hueXShift - hueYShift + 32768;
					computeAnimations(RIGHT_TOP_START_INDEX, STRIP_LEN);
					
				//============================	/* Send pixels OR Wait to sleep */
					/* Data are sent to pixels only if each one is OFF. g_LedsOn allows to send 0 to 
					   all pixels and effectively turning them OFF before the next cycle where nothing 
					   will be sent. */
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
						if(brightness == 0){
							g_LedsOn = false;
						}
					}
				} // End of subsets
			} // if(g_dataReady)
		} // if(!g_eco && !g_sleepCoolDown)
		
		/********************/
		/* Sleep management */
		/********************/
		if(g_sleepEngage == 255 || g_eco || g_firstBoot){
			/* Sleep mode is forced on the first boot to avoid an unsolved glitch with the LEDs that 
			   happens only before everything has been put to sleep for the first time. */
			g_firstBoot = false;
			if(!g_eco){
				g_eco = true;
				g_sleepCoolDown = true;
 				accel::enableAutoSleep();
				g_sleepEngage = 0;
				DDRB = 0;								// set PORTB to Hi-Z
				PORTB &= ~(LED_POWER | LED_CMD);
			}
			WDTCR = 1<<WDIE | 1<<WDCE | 1<<WDE | 3;	// enable watchdog  timer and interrupt for .125sec
			set_sleep_mode(SLEEP_MODE_PWR_DOWN);	// SLEEP_MODE_PWR_DOWN
	 		sleep_mode();							// sleep enable
		/* Wake-up */
		}
		
		/************************/
		/* Interrupt management */
		/************************/
		if(!(PINB & 1<<PB1)){
			/* Check int sources from Accelerometer */
			if(accel::checkIntSource(&g_accelIntSource) == I2cOk){
				if(g_accelIntSource & accel::IntTRANS){		// Transient event
					accel::readIntTransient();				// Unlatch TRANS int event
					if(g_eco && !g_sleepCoolDown){			// Conditions for exiting sleep mode
						DDRB |= (LED_POWER | LED_CMD);	// Outputs
						PORTB |= LED_POWER;			// Enable LEDs
						g_eco = false;
						accel::disableAutoSleep();
					}else{									// eliminates Trans_INT that happens when accelerometer goes to sleep
						g_sleepCoolDown = false;
					}
				}
				if(g_accelIntSource & accel::IntDRDY){
					/* Interrupt event will be unlatched at the begining of the next cycle when the 
					   accelerometer data will be read. */
					g_dataReady = true;
				}
			}
		} // End of int management
    } // End of global loop
}
/*******************************************************************************
*                                 FUNCTIONS                                    *
*******************************************************************************/
void computeAnimations(uint8_t startIndex, uint8_t endIndex)
{
	uint8_t& i = startIndex;
	
#if defined(ANIM_WINGS)
	colorHSV(hueWing, SATURATION_DEFAULT, wingBright, &wingRed, &wingGreen, &wingBlue);
#endif // ANIM_WINGS

	for(; i<endIndex; i++){
		if(stripAnim[i] < AnimStarlight){
			stripAnim[i] = AnimNone;		// reset animation priority of the current LED
		}
#ifdef ANIM_BALLTILT
		// Ball-Tilt animation
//#if defined(BALL_TILT_HIPASS)
 		xBall = abs(posXHiPass - (int8_t)pgm_read_byte(&ledXPos[i]));	// Relative position of LED-to-acceleration
 		yBall = abs(posYHiPass - (int8_t)pgm_read_byte(&ledYPos[i]));
//#else
//		xBall = abs(posXSmooth - (int8_t)pgm_read_byte(&ledXPos[i]));	// Relative position of LED-to-acceleration
//		yBall = abs(posYSmooth - (int8_t)pgm_read_byte(&ledYPos[i]));
//#endif // BALL_TILT_HIPASS
		distComp.value = ((xBall*xBall)+(yBall*yBall))<<1;						// Pythagorean formula (+ left shift for more precision)
		if(distComp.split.MSB < PROXI_LIGHT_LEN){								// Find relative brightness in proximity LUT
			uint8_t bright = pgm_read_byte(&proxiLight[distComp.split.MSB]);	// indexing to "proxiLight" is more precise (there is more values) thanks to the left-shift
			bright = ((uint16_t)(bright*brightness))>>7;						// Scale LED brightness to global brightness
			if(bright > stripBright[i] || stripAnim[i] <= AnimBallTilt){		// Handles superposition of animations
				stripAnim[i] = AnimBallTilt;									// Set animation priority to the current LED
				colorHSV(hueBall, SATURATION_DEFAULT, bright, &stripRed__[i], &stripGreen[i], &stripBlue_[i]);
				stripBright[i] = bright;
			}
		}
#endif // ANIM_BALLTILT
		// Wings animation
#ifdef ANIM_STATIC_FRENCHFLAG
		// Display French flag
		if(stripAnim[i] <= AnimStatic){
			stripAnim[i] = AnimStatic;
			int8_t tmpLedX = (int8_t)pgm_read_byte(&ledXPos[i]);
			if(tmpLedX < -22){
				colorHSV(HUE_BLUE, SATURATION_DEFAULT, brightness, &stripRed__[i], &stripGreen[i], &stripBlue_[i]);
			}else if(tmpLedX < 28){
				colorHSV(0, 0, brightness, &stripRed__[i], &stripGreen[i], &stripBlue_[i]);
			}else{
				colorHSV(HUE_RED, SATURATION_DEFAULT, brightness, &stripRed__[i], &stripGreen[i], &stripBlue_[i]);
			}
		}
#endif // ANIM_STATIC_FRENCHFLAG
#if defined(ANIM_WINGS)
		// Display colored wing
		if(stripAnim[i] <= AnimWings){			// Handles superposition of animations.
			stripAnim[i] = AnimWings;			// Set animation priority to the current LED.
			stripRed__[i] = wingRed;			// Set colors to current pixel using precomputed values for this set of LEDs.
			stripGreen[i] = wingGreen;
			stripBlue_[i] = wingBlue;
			/* While stripBright[] is not used for this animation, it is required by the starlight 
			   animation to be able to give its priority when the LED fades below this LED's brightness. */
			stripBright[i] = wingBright;
		}
#endif // ANIM_WINGS
		// Set color to unused LEDs
		if(stripAnim[i] == AnimNone){
			stripRed__[i] = 0;
			stripGreen[i] = 0;
			stripBlue_[i] = 0;
		// Render Starlight LEDs
		}else if(stripAnim[i] >= AnimStarlight){
			uint8_t starIndex = stripAnim[i] - AnimStarlight;		// Recover the index to starlightHue[] for this LED.
			colorHSV(starlightHue[starIndex], SATURATION_SATRLIGHT, stripBright[i], &stripRed__[i], &stripGreen[i], &stripBlue_[i]);
			if(stripBright[i] > LED_FADE_SPEED){
				stripBright[i]-=LED_FADE_SPEED;
			}else{
				stripBright[i]=0;
				stripAnim[i] = AnimNone;	// Resets this LED's priority.
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
	
	DDRB |= LED_CMD;
	volatile uint8_t n1, n2 = 0;  // First, next bits out
	uint8_t hi = PORTB |  (LED_CMD);
	uint8_t lo = PORTB & ~(LED_CMD);
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