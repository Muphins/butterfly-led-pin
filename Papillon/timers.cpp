/*******************************************************************************
*                                  INCLUDES                                    *
*******************************************************************************/
#include <avr/io.h>
#include <avr/interrupt.h>
#include "timers.h"
/*******************************************************************************
*								   VARIABLES								   *
*******************************************************************************/
static volatile bool _timeBaseInt = false;
/*******************************************************************************
*                                    CODE                                      *
*******************************************************************************/

/******************************************************************************/
/*																	  TIMER 3 */
/*																	TIME BASE */
/******************************************************************************/

//======================================================================== Init
#define MAX_MS (uint8_t)(255/2)
#define TIMEBASE_STEPS	(TIMEBASE_MS * 2)
void timeBaseInit()
{
	/* Provide power to timer1 */
	PRR &= ~(1<<PRTIM1);
	
	/* CPU clock = 8 MHz
	 * 1/4096 prescaler -> 512µs steps.
	 * ~2 steps for 1ms. -> totalStep = delay_ms * 2
	 */
	/* Load delay */
	OCR1C = TIMEBASE_STEPS;
	/* Stop timer1 */
	TCCR1 = 0;
	/* Timer 1 interrupt on overflow */
	TIMSK = (1<<TOIE1);
}
//======================================================================== Stop
void timeBaseSleep()
{
	/* Cut power to timer1 */
	PRR |= (1<<PRTIM1);
	/* Stop timer1 */
	TCCR1 = 0;
}
//======================================================================== Start
void	timeBaseStart()
{
	/* Provide power to timer1 */
	PRR &= ~(1<<PRTIM1);
	/* Reset counter */
	TCNT1 = 0;
	/* Start Timer */
	/* PWM1A needs to be enabled for the overflow int to trigger on a compare match with OCR1C
	   As PWM is not needed otherwise, output pin OC1A is disconnected from the timer */
	TCCR1 = (1<<CTC1) | (1<<PWM1A) | (0<<COM1A0) | (0xD<<CS10);	// Clear on compare match, enable PWM, OC1A disconnected, 1/4096 prescaler
}
//======================================================================== timeBaseWait
void timeBaseWait()
{
	while(!_timeBaseInt);
	_timeBaseInt=false;
}
//======================================================================== timeBaseInt
bool timeBaseInt()
{
	if(_timeBaseInt){
		_timeBaseInt=false;
		return true;
	}
	return false;
}
//======================================================================== Interrupts
//ISR(TIMER1_COMPA_vect){}
ISR(TIMER1_OVF_vect){
	_timeBaseInt = true;
}
//ISR(TIMER1_COMPB_vect){}