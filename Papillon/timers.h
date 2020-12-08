#ifndef TIMERS_H_
#define TIMERS_H_
/*******************************************************************************
*                                  INCLUDES                                    *
*******************************************************************************/
/*******************************************************************************
*								   DEFINES									   *
*******************************************************************************/
#define TIMEBASE_MS			20
#define ONE_SECOND			(1000/TIMEBASE_MS)

#define DELAYS_RESET		255
/*******************************************************************************
*								  TYPEDEFS									   *
*******************************************************************************/
/*******************************************************************************
*								 VARIABLES									   *
*******************************************************************************/
/*******************************************************************************
*								 Prototypes									   *
*******************************************************************************/

/* Initializes timer 1 for timeBase operation. */
void timeBaseInit();

/* Put timer1 in low power mode */
void timeBaseSleep();

/* Starts timer1 with a time base of TIMEBASE_MS milliseconds. */
void timeBaseStart();

/* Waits until timer1 has reached TIMEBASE_MS milliseconds. */
void timeBaseWait();

/* Returns True if a timeBase interrupt has occurred. */
bool timeBaseInt();

#endif /* TIMERS_H_ */