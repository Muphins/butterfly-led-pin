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
bool autoSleep = false;
static bool activeMode = false;
/*******************************************************************************
*                                    CODE                                      *
*******************************************************************************/
void init()
{
	/* standby */
	activeMode = false;
	SoftI2CStart();
	if(SoftI2CWriteByte(I2C_MMA8453_WRITE) == I2cOk)
		if(SoftI2CWriteByte(MMA_CTRL_REG1) == I2cOk)
			SoftI2CWriteByte(MMA_CTRL_REG1_STB);	// Set to standby mode
	SoftI2CStop();
	/* configure data-rate and wake/sleep scheme */
	SoftI2CStart();
	if(SoftI2CWriteByte(I2C_MMA8453_WRITE) == I2cOk)
		if(SoftI2CWriteByte(MMA_CTRL_REG2) == I2cOk)
			//SoftI2CWriteByte(0b00011111);	// set low power for sleep and wake modes, enable auto-sleep
			SoftI2CWriteByte(MMA_CTRL_REG2_DEFAULT);	// set low power for sleep and wake modes, enable auto-sleep
	SoftI2CStop();
	autoSleep = false;
	SoftI2CStart();
	if(SoftI2CWriteByte(I2C_MMA8453_WRITE) == I2cOk)
		if(SoftI2CWriteByte(MMA_ASLP_COUNT) == I2cOk)
			SoftI2CWriteByte(2);			// 1.92s of inactivity before sleep (320ms steps)
	SoftI2CStop();
	SoftI2CStart();
	if(SoftI2CWriteByte(I2C_MMA8453_WRITE) == I2cOk)
		if(SoftI2CWriteByte(MMA_CTRL_REG4) == I2cOk)
			SoftI2CWriteByte(MMA_CTRL_REG4_DEFAULT);	// enable transient detection
	SoftI2CStop();
	SoftI2CStart();
	if(SoftI2CWriteByte(I2C_MMA8453_WRITE) == I2cOk)
		if(SoftI2CWriteByte(MMA_CTRL_REG3) == I2cOk)
			SoftI2CWriteByte(MMA_CTRL_REG3_DEFAULT);	// Enable wake by transient detection
	SoftI2CStop();
	/* set pulse detection registers */
	SoftI2CStart();
	if(SoftI2CWriteByte(I2C_MMA8453_WRITE) == I2cOk)
		if(SoftI2CWriteByte(MMA_TRANSIENT_CFG) == I2cOk)
			SoftI2CWriteByte(MMA_TRANSIENT_CFG_DEFAULT);	// detection on x, y, z through hi-pass filter
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
			SoftI2CWriteByte(MMA_CTRL_REG1_ACT);
	SoftI2CStop();
	activeMode = true;
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
	activeMode = false;
	SoftI2CStart();
	if(SoftI2CWriteByte(I2C_MMA8453_WRITE) == I2cOk)
		if(SoftI2CWriteByte(MMA_CTRL_REG1) == I2cOk)
			SoftI2CWriteByte(MMA_CTRL_REG1_STB);	// Set to standby mode
	SoftI2CStop();
	/* set transient detection registers */
	SoftI2CStart();
	if(SoftI2CWriteByte(I2C_MMA8453_WRITE) == I2cOk)
		if(SoftI2CWriteByte(MMA_TRANSIENT_CFG) == I2cOk)
			SoftI2CWriteByte(MMA_TRANSIENT_CFG_EVLATCH);	// Int latched; detection on x, y, z through hi-pass filter
	SoftI2CStop();
	/* Activate device */
	SoftI2CStart();
	if(SoftI2CWriteByte(I2C_MMA8453_WRITE) == I2cOk)
		if(SoftI2CWriteByte(MMA_CTRL_REG1) == I2cOk)
			SoftI2CWriteByte(MMA_CTRL_REG1_ACT);	// set 12.5Hz sleep, 800Hz wake, active mode
	SoftI2CStop();
	activeMode = true;
	return I2cOk;
}

tI2cStatus disableTransientIntLatch()
{
	/* standby */
	activeMode = false;
	SoftI2CStart();
	if(SoftI2CWriteByte(I2C_MMA8453_WRITE) == I2cOk)
		if(SoftI2CWriteByte(MMA_CTRL_REG1) == I2cOk)
			SoftI2CWriteByte(MMA_CTRL_REG1_STB);	// Set to standby mode
	SoftI2CStop();
	/* set transient detection registers */
	SoftI2CStart();
	if(SoftI2CWriteByte(I2C_MMA8453_WRITE) == I2cOk)
		if(SoftI2CWriteByte(MMA_TRANSIENT_CFG) == I2cOk)
			SoftI2CWriteByte(MMA_TRANSIENT_CFG_DEFAULT);	// Int not-latched; detection on x, y, z through hi-pass filter
	SoftI2CStop();
	/* Activate device */
	SoftI2CStart();
	if(SoftI2CWriteByte(I2C_MMA8453_WRITE) == I2cOk)
		if(SoftI2CWriteByte(MMA_CTRL_REG1) == I2cOk)
			SoftI2CWriteByte(MMA_CTRL_REG1_ACT);
	SoftI2CStop();
	activeMode = true;
	return I2cOk;
}

tI2cStatus enableAutoSleep()
{
	/* standby */
	activeMode = false;
	SoftI2CStart();
	if(SoftI2CWriteByte(I2C_MMA8453_WRITE) == I2cOk)
		if(SoftI2CWriteByte(MMA_CTRL_REG1) == I2cOk)
			SoftI2CWriteByte(MMA_CTRL_REG1_STB);
	SoftI2CStop();
	/* configure data-rate and wake/sleep scheme */
	SoftI2CStart();
	if(SoftI2CWriteByte(I2C_MMA8453_WRITE) == I2cOk)
		if(SoftI2CWriteByte(MMA_CTRL_REG2) == I2cOk)
			SoftI2CWriteByte(MMA_CTRL_REG2_DEFAULT | 1<<AUTO_SLEEP);
	SoftI2CStop();
	/* Activate device */
	SoftI2CStart();
	if(SoftI2CWriteByte(I2C_MMA8453_WRITE) == I2cOk)
		if(SoftI2CWriteByte(MMA_CTRL_REG1) == I2cOk)
			SoftI2CWriteByte(MMA_CTRL_REG1_ACT);
	SoftI2CStop();
	activeMode = true;
	autoSleep = true;
	return I2cOk;
}

tI2cStatus disableAutoSleep()
{
	/* standby */
	activeMode = false;
	SoftI2CStart();
	if(SoftI2CWriteByte(I2C_MMA8453_WRITE) == I2cOk)
		if(SoftI2CWriteByte(MMA_CTRL_REG1) == I2cOk)
			SoftI2CWriteByte(MMA_CTRL_REG1_STB);
	SoftI2CStop();
	/* configure data-rate and wake/sleep scheme */
	SoftI2CStart();
	if(SoftI2CWriteByte(I2C_MMA8453_WRITE) == I2cOk)
		if(SoftI2CWriteByte(MMA_CTRL_REG2) == I2cOk)
			SoftI2CWriteByte(MMA_CTRL_REG2_DEFAULT);
	SoftI2CStop();
	/* Activate device */
	SoftI2CStart();
	if(SoftI2CWriteByte(I2C_MMA8453_WRITE) == I2cOk)
		if(SoftI2CWriteByte(MMA_CTRL_REG1) == I2cOk)
			SoftI2CWriteByte(MMA_CTRL_REG1_ACT);
	SoftI2CStop();
	activeMode = true;
	autoSleep = false;
	return I2cOk;
}

tI2cStatus getAcc(int8_t *x, int8_t *y, int8_t *z)
{
	
	if(activeMode){
		SoftI2CStart();
		if(SoftI2CWriteByte(I2C_MMA8453_WRITE) != I2cOk) return I2cNok;
		if(SoftI2CWriteByte(MMA_OUT_X_MSB) != I2cOk) return I2cNok;
		SoftI2CStart();
		if(SoftI2CWriteByte(I2C_MMA8453_READ) != I2cOk) return I2cNok;
		SoftI2CReadByte((uint8_t*)x, false);
		SoftI2CStop();
		
		SoftI2CStart();
		if(SoftI2CWriteByte(I2C_MMA8453_WRITE) != I2cOk) return I2cNok;
		if(SoftI2CWriteByte(MMA_OUT_Y_MSB) != I2cOk) return I2cNok;
		SoftI2CStart();
		if(SoftI2CWriteByte(I2C_MMA8453_READ) != I2cOk) return I2cNok;
		SoftI2CReadByte((uint8_t*)y, false);
		SoftI2CStop();
		
		SoftI2CStart();
		if(SoftI2CWriteByte(I2C_MMA8453_WRITE) != I2cOk) return I2cNok;
		if(SoftI2CWriteByte(MMA_OUT_Z_MSB) != I2cOk) return I2cNok;
		SoftI2CStart();
		if(SoftI2CWriteByte(I2C_MMA8453_READ) != I2cOk) return I2cNok;
		SoftI2CReadByte((uint8_t*)z, false);
		SoftI2CStop();
	}else{
		*x = 0;
		*y = 0;
		*z = 0;
	}
	
	return I2cOk;
}

//end of namespace
}