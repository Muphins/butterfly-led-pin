/*******************************************************************************
*                                  INCLUDES                                    *
*******************************************************************************/
#include <avr/io.h>
#include <util/delay.h>
#include <stdlib.h>
#include "MMA8453.h"

namespace accel{
/*******************************************************************************
*								   VARIABLES								   *
*******************************************************************************/

/*******************************************************************************
*                                    CODE                                      *
*******************************************************************************/
void init()
{
	SoftI2CStart();
	if(SoftI2CWriteByte(I2C_MMA8453_WRITE) == I2cOk)
		if(SoftI2CWriteByte(0x2A) == I2cOk)
			SoftI2CWriteByte(0b00000011 | 0<<3);	// 800Hz data-rate
	SoftI2CStop();
	SoftI2CStart();
	if(SoftI2CWriteByte(I2C_MMA8453_WRITE) == I2cOk)
		if(SoftI2CWriteByte(0x2B) == I2cOk)
			SoftI2CWriteByte(0b00000000);			// normal mode
	SoftI2CStop();
}

void sleep()
{
	/* standby */
	SoftI2CStart();
	if(SoftI2CWriteByte(I2C_MMA8453_WRITE) == I2cOk)
		if(SoftI2CWriteByte(0x2A) == I2cOk)
			SoftI2CWriteByte(0b11000010);	// Set to standby mode
	SoftI2CStop();
	/* configure data-rate and wake/sleep scheme */
	SoftI2CStart();
	if(SoftI2CWriteByte(I2C_MMA8453_WRITE) == I2cOk)
		if(SoftI2CWriteByte(0x2B) == I2cOk)
			SoftI2CWriteByte(0b00011111);	// set low power for sleep and wake modes, enable auto-sleep
	SoftI2CStop();
	SoftI2CStart();
	if(SoftI2CWriteByte(I2C_MMA8453_WRITE) == I2cOk)
		if(SoftI2CWriteByte(0x29) == I2cOk)
			SoftI2CWriteByte(6);			// 1.92s of inactivity before sleep (320ms steps)
	SoftI2CStop();
	SoftI2CStart();
	if(SoftI2CWriteByte(I2C_MMA8453_WRITE) == I2cOk)
		if(SoftI2CWriteByte(0x2D) == I2cOk)
			SoftI2CWriteByte(0b00100000);	// enable transient detection
	SoftI2CStop();
	SoftI2CStart();
	if(SoftI2CWriteByte(I2C_MMA8453_WRITE) == I2cOk)
		if(SoftI2CWriteByte(0x2C) == I2cOk)
			SoftI2CWriteByte(0b01000000);	// Enable wake by transient detection
	SoftI2CStop();
	/* set pulse detection registers */
	SoftI2CStart();
	if(SoftI2CWriteByte(I2C_MMA8453_WRITE) == I2cOk)
		if(SoftI2CWriteByte(0x1D) == I2cOk)
			SoftI2CWriteByte(0b00011110);	// detection on x, y, z through hi-pass filter
	SoftI2CStop();
	SoftI2CStart();
	if(SoftI2CWriteByte(I2C_MMA8453_WRITE) == I2cOk)
		if(SoftI2CWriteByte(0x1F) == I2cOk)
			SoftI2CWriteByte(1);			// Threshold = value * .063g
	SoftI2CStop();
	
	/* Activate device */
	SoftI2CStart();
	if(SoftI2CWriteByte(I2C_MMA8453_WRITE) == I2cOk)
		if(SoftI2CWriteByte(0x2A) == I2cOk)
			SoftI2CWriteByte(0b01000011);	// set 12.5Hz sleep, 800Hz wake, active mode
	SoftI2CStop();
}

void wake()
{
	SoftI2CStart();
	if(SoftI2CWriteByte(I2C_MMA8453_WRITE) == I2cOk)
		if(SoftI2CWriteByte(0x2B) == I2cOk)
			SoftI2CWriteByte(0b00011011);	// set low power for sleep and wake modes, disable auto-sleep
	SoftI2CStop();
}

tI2cStatus move(uint8_t *x, uint8_t *y, uint8_t *z)
{
	static uint8_t x_old,
				   y_old,
				   z_old;
	uint8_t x_cur,
			y_cur,
			z_cur;
	
	SoftI2CStart();
	if(SoftI2CWriteByte(I2C_MMA8453_WRITE) != I2cOk) return I2cNok;
	if(SoftI2CWriteByte(0x01) != I2cOk) return I2cNok;
	SoftI2CStart();
	if(SoftI2CWriteByte(I2C_MMA8453_READ) != I2cOk) return I2cNok;
	SoftI2CReadByte(&x_cur, false);
	SoftI2CStop();
	
	SoftI2CStart();
	if(SoftI2CWriteByte(I2C_MMA8453_WRITE) != I2cOk) return I2cNok;
	if(SoftI2CWriteByte(0x03) != I2cOk) return I2cNok;
	SoftI2CStart();
	if(SoftI2CWriteByte(I2C_MMA8453_READ) != I2cOk) return I2cNok;
	SoftI2CReadByte(&y_cur, false);
	SoftI2CStop();
	
	SoftI2CStart();
	if(SoftI2CWriteByte(I2C_MMA8453_WRITE) != I2cOk) return I2cNok;
	if(SoftI2CWriteByte(0x05) != I2cOk) return I2cNok;
	SoftI2CStart();
	if(SoftI2CWriteByte(I2C_MMA8453_READ) != I2cOk) return I2cNok;
	SoftI2CReadByte(&z_cur, false);
	SoftI2CStop();
	
	*x=abs((int8_t)x_old - (int8_t)x_cur);
	*y=abs((int8_t)y_old - (int8_t)y_cur);
	*z=abs((int8_t)z_old - (int8_t)z_cur);
	x_old = x_cur;
	y_old = y_cur;
	z_old = z_cur;
	
	return I2cOk;
}

//end of namespace
}