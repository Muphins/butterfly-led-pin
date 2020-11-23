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
			SoftI2CWriteByte(0b00000011);
	SoftI2CStop();
}

uint8_t test()
{
	uint8_t data, dummy;

	SoftI2CStart();
	if(SoftI2CWriteByte(I2C_MMA8453_WRITE) != I2cOk) return 0;
	if(SoftI2CWriteByte(0x01) != I2cOk) return 0;
// 	SoftI2CStop();
// 	
// 	_delay_ms(10);
	SoftI2CStart();
	if(SoftI2CWriteByte(I2C_MMA8453_READ) != I2cOk) return 0;
	SoftI2CReadByte(&data, true);
	SoftI2CReadByte(&dummy, true);
	SoftI2CReadByte(&dummy, false);
	SoftI2CStop();
	//if(data == 0) data = 0x44;
	return data;
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
}

//end of namespace
}