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
//#include "twi.h"
#include "MMA8453.h"

int main(void)
{
	sei();
	SoftI2CInit();
	
	accel::init();
	
 	DDRB |= 1<<PB3 | 1<<PB4;
 	PORTB &= ~(1<<PB3);
	
	uint8_t accX = 0;
	uint8_t accY = 0;
	uint8_t accZ = 0;
	uint8_t delayPwm;
	uint16_t cycles;
    while (1) 
    {
		//accX = accel::test();
		//accX = abs((int8_t)accX) << 1;
		//if(accX < 30) accX = 0;
		accel::move(&accX, &accY, &accZ);
		accX+=accY+accZ;
		accX=accX>>2;
		//for(accX = 0; accX < 100; accX++){
			//accX = 0;
			for(cycles = 0; cycles < 200; cycles++){
				if(accX > 0){
					PORTB |= 1<<PB3 | 1<<PB4;
					_delay_loop_2(accX);
				}
				
				delayPwm = 255 - accX;
				if(delayPwm > 0){
					PORTB &= ~(1<<PB3 | 1<<PB4);
					_delay_loop_2(delayPwm);
				}
			}
		//}
    }
}

