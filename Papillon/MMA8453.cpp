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
	/* standby */
	SoftI2CStart();
	if(SoftI2CWriteByte(I2C_MMA8453_WRITE) == I2cOk)
		if(SoftI2CWriteByte(MMA_CTRL_REG1) == I2cOk)
			SoftI2CWriteByte(MMA_CTRL_REG1_VAL);	// Set to standby mode
	SoftI2CStop();
	/* configure data-rate and wake/sleep scheme */
	SoftI2CStart();
	if(SoftI2CWriteByte(I2C_MMA8453_WRITE) == I2cOk)
		if(SoftI2CWriteByte(MMA_CTRL_REG2) == I2cOk)
			//SoftI2CWriteByte(0b00011111);	// set low power for sleep and wake modes, enable auto-sleep
			SoftI2CWriteByte(0b00011011);	// set low power for sleep and wake modes, enable auto-sleep
	SoftI2CStop();
	SoftI2CStart();
	if(SoftI2CWriteByte(I2C_MMA8453_WRITE) == I2cOk)
		if(SoftI2CWriteByte(MMA_ASLP_COUNT) == I2cOk)
			SoftI2CWriteByte(6);			// 1.92s of inactivity before sleep (320ms steps)
	SoftI2CStop();
	SoftI2CStart();
	if(SoftI2CWriteByte(I2C_MMA8453_WRITE) == I2cOk)
		if(SoftI2CWriteByte(MMA_CTRL_REG4) == I2cOk)
			SoftI2CWriteByte(0b00100000);	// enable transient detection
	SoftI2CStop();
	SoftI2CStart();
	if(SoftI2CWriteByte(I2C_MMA8453_WRITE) == I2cOk)
		if(SoftI2CWriteByte(MMA_CTRL_REG3) == I2cOk)
			SoftI2CWriteByte(0b01000000);	// Enable wake by transient detection
	SoftI2CStop();
	/* set pulse detection registers */
	SoftI2CStart();
	if(SoftI2CWriteByte(I2C_MMA8453_WRITE) == I2cOk)
		if(SoftI2CWriteByte(MMA_TRANSIENT_CFG) == I2cOk)
			SoftI2CWriteByte(0b00001110);	// detection on x, y, z through hi-pass filter
	SoftI2CStop();
	SoftI2CStart();
	if(SoftI2CWriteByte(I2C_MMA8453_WRITE) == I2cOk)
		if(SoftI2CWriteByte(MMA_TRANSIENT_THS) == I2cOk)
			SoftI2CWriteByte(1);			// Threshold = value * .063g
	SoftI2CStop();
// 	SoftI2CStart();
// 	if(SoftI2CWriteByte(I2C_MMA8453_WRITE) == I2cOk)
// 		if(SoftI2CWriteByte(MMA_TRANSIENT_COUNT) == I2cOk)
// 			SoftI2CWriteByte(2);			// Debounce
// 	SoftI2CStop();
	
	
	/* Activate device */
	SoftI2CStart();
	if(SoftI2CWriteByte(I2C_MMA8453_WRITE) == I2cOk)
		if(SoftI2CWriteByte(MMA_CTRL_REG1) == I2cOk)
			SoftI2CWriteByte(MMA_CTRL_REG1_VAL|MMA_ACTIVE_MODE);	// set 12.5Hz sleep, 800Hz wake, active mode
	SoftI2CStop();
}

tI2cStatus checkIntSource()
{
	uint8_t dummy;
	SoftI2CStart();
	if(SoftI2CWriteByte(I2C_MMA8453_WRITE) != I2cOk) return I2cNok;
	if(SoftI2CWriteByte(MMA_TRANSIENT_SRC) != I2cOk) return I2cNok;
	SoftI2CStart();
	if(SoftI2CWriteByte(I2C_MMA8453_READ) != I2cOk) return I2cNok;
	SoftI2CReadByte(&dummy, false);
	SoftI2CStop();
	return I2cOk;
}

tI2cStatus enableTransientIntLatch()
{
	/* standby */
	SoftI2CStart();
	if(SoftI2CWriteByte(I2C_MMA8453_WRITE) == I2cOk)
		if(SoftI2CWriteByte(MMA_CTRL_REG1) == I2cOk)
			SoftI2CWriteByte(MMA_CTRL_REG1_VAL);	// Set to standby mode
	SoftI2CStop();
	/* set transient detection registers */
	SoftI2CStart();
	if(SoftI2CWriteByte(I2C_MMA8453_WRITE) == I2cOk)
		if(SoftI2CWriteByte(MMA_TRANSIENT_CFG) == I2cOk)
			SoftI2CWriteByte(0b00011110);	// Int latched; detection on x, y, z through hi-pass filter
	SoftI2CStop();
	/* Activate device */
	SoftI2CStart();
	if(SoftI2CWriteByte(I2C_MMA8453_WRITE) == I2cOk)
		if(SoftI2CWriteByte(MMA_CTRL_REG1) == I2cOk)
			SoftI2CWriteByte(MMA_CTRL_REG1_VAL|MMA_ACTIVE_MODE);	// set 12.5Hz sleep, 800Hz wake, active mode
	SoftI2CStop();
	return I2cOk;
}

tI2cStatus disableTransientIntLatch()
{
	/* standby */
	SoftI2CStart();
	if(SoftI2CWriteByte(I2C_MMA8453_WRITE) == I2cOk)
		if(SoftI2CWriteByte(MMA_CTRL_REG1) == I2cOk)
			SoftI2CWriteByte(0b01000010);	// Set to standby mode
	SoftI2CStop();
	/* set transient detection registers */
	SoftI2CStart();
	if(SoftI2CWriteByte(I2C_MMA8453_WRITE) == I2cOk)
		if(SoftI2CWriteByte(MMA_TRANSIENT_CFG) == I2cOk)
			SoftI2CWriteByte(0b00001110);	// Int not-latched; detection on x, y, z through hi-pass filter
	SoftI2CStop();
	/* Activate device */
	SoftI2CStart();
	if(SoftI2CWriteByte(I2C_MMA8453_WRITE) == I2cOk)
		if(SoftI2CWriteByte(MMA_CTRL_REG1) == I2cOk)
			SoftI2CWriteByte(MMA_CTRL_REG1_VAL|MMA_ACTIVE_MODE);	// set 12.5Hz sleep, 800Hz wake, active mode
	SoftI2CStop();
	return I2cOk;
}

tI2cStatus enableAutoSleep()
{
	/* standby */
	SoftI2CStart();
	if(SoftI2CWriteByte(I2C_MMA8453_WRITE) == I2cOk)
		if(SoftI2CWriteByte(MMA_CTRL_REG1) == I2cOk)
			SoftI2CWriteByte(MMA_CTRL_REG1_VAL);	// Set to standby mode
	SoftI2CStop();
	/* configure data-rate and wake/sleep scheme */
	SoftI2CStart();
	if(SoftI2CWriteByte(I2C_MMA8453_WRITE) == I2cOk)
		if(SoftI2CWriteByte(MMA_CTRL_REG2) == I2cOk)
			//SoftI2CWriteByte(0b00011111);	// set low power for sleep and wake modes, enable auto-sleep
			SoftI2CWriteByte(0b00011111);	// set low power for sleep and wake modes, enable auto-sleep
	SoftI2CStop();
	/* Activate device */
	SoftI2CStart();
	if(SoftI2CWriteByte(I2C_MMA8453_WRITE) == I2cOk)
		if(SoftI2CWriteByte(MMA_CTRL_REG1) == I2cOk)
			SoftI2CWriteByte(MMA_CTRL_REG1_VAL|MMA_ACTIVE_MODE);	// set 12.5Hz sleep, 800Hz wake, active mode
	SoftI2CStop();
}

tI2cStatus disableAutoSleep()
{
	/* standby */
	SoftI2CStart();
	if(SoftI2CWriteByte(I2C_MMA8453_WRITE) == I2cOk)
		if(SoftI2CWriteByte(MMA_CTRL_REG1) == I2cOk)
			SoftI2CWriteByte(MMA_CTRL_REG1_VAL);	// Set to standby mode
	SoftI2CStop();
	/* configure data-rate and wake/sleep scheme */
	SoftI2CStart();
	if(SoftI2CWriteByte(I2C_MMA8453_WRITE) == I2cOk)
		if(SoftI2CWriteByte(MMA_CTRL_REG2) == I2cOk)
			//SoftI2CWriteByte(0b00011111);	// set low power for sleep and wake modes, enable auto-sleep
			SoftI2CWriteByte(0b00011011);	// set low power for sleep and wake modes, enable auto-sleep
	SoftI2CStop();
	/* Activate device */
	SoftI2CStart();
	if(SoftI2CWriteByte(I2C_MMA8453_WRITE) == I2cOk)
		if(SoftI2CWriteByte(MMA_CTRL_REG1) == I2cOk)
			SoftI2CWriteByte(MMA_CTRL_REG1_VAL|MMA_ACTIVE_MODE);	// set 12.5Hz sleep, 800Hz wake, active mode
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
	if(SoftI2CWriteByte(MMA_OUT_X_MSB) != I2cOk) return I2cNok;
	SoftI2CStart();
	if(SoftI2CWriteByte(I2C_MMA8453_READ) != I2cOk) return I2cNok;
	SoftI2CReadByte(&x_cur, false);
	SoftI2CStop();
	
	SoftI2CStart();
	if(SoftI2CWriteByte(I2C_MMA8453_WRITE) != I2cOk) return I2cNok;
	if(SoftI2CWriteByte(MMA_OUT_Y_MSB) != I2cOk) return I2cNok;
	SoftI2CStart();
	if(SoftI2CWriteByte(I2C_MMA8453_READ) != I2cOk) return I2cNok;
	SoftI2CReadByte(&y_cur, false);
	SoftI2CStop();
	
	SoftI2CStart();
	if(SoftI2CWriteByte(I2C_MMA8453_WRITE) != I2cOk) return I2cNok;
	if(SoftI2CWriteByte(MMA_OUT_Z_MSB) != I2cOk) return I2cNok;
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