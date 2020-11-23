/**********************************************************

Software I2C Library for AVR Devices.

Copyright 2008-2012
eXtreme Electronics, India
www.eXtremeElectronics.co.in
**********************************************************/


#ifndef _I2CSOFT_H
#define _I2CSOFT_H

#include <stdint.h>
/*******************************************************************************
*								   DEFINES									   *
*******************************************************************************/
/* 
I/O Configuration 
*/
#define SCLPORT	PORTB
#define SCLDDR	DDRB

#define SDAPORT	PORTB
#define SDADDR	DDRB

#define SDAPIN	PINB
#define SCLPIN	PINB

#define SCL	PB2
#define SDA	PB0


#define SOFT_I2C_SDA_LOW	SDADDR|=((1<<SDA))
#define SOFT_I2C_SDA_HIGH	SDADDR&=(~(1<<SDA))

#define SOFT_I2C_SCL_LOW	SCLDDR|=((1<<SCL))
#define SOFT_I2C_SCL_HIGH	SCLDDR&=(~(1<<SCL))

#define I2C_READ	1
#define I2C_WRITE	0

/*******************************************************************************
*								  TYPEDEFS									   *
*******************************************************************************/
enum tI2cStatus{
	I2cIdle			= 0xFF,
	I2cOk			= 0,
	I2cNok			= 1,
	I2cStartOk		= 2,
	I2cStartFail	= 3,
	I2cAddrOk		= 4,
	I2cAddrFail		= 5,
	I2cWriteOk		= 6,
	I2cWriteFail	= 7,
	I2cReadOk		= 8,
	I2cReadFail		= 9
};
/*******************************************************************************
*								 Prototypes									   *
*******************************************************************************/

/**********************************************************
SoftI2CInit()

Description:
	Initializes the Soft I2C Engine.
	Must be called before using any other lib functions.
	
Arguments:
	NONE
	
Returns:
	Nothing

**********************************************************/
void SoftI2CInit();	

/**********************************************************
SoftI2CStart()

Description:
	Generates a START(S) condition on the bus.
	NOTE: Can also be used for generating repeat start(Sr)
	condition too.
	
Arguments:
	NONE
	
Returns:
	Nothing

**********************************************************/
void SoftI2CStart();

/**********************************************************
SoftI2CStop()

Description:
	Generates a STOP(P) condition on the bus.
	NOTE: Can also be used for generating repeat start
	condition too.
	
Arguments:
	NONE
	
Returns:
	Nothing

**********************************************************/
void SoftI2CStop();

/**********************************************************
SoftI2CWriteByte()

Description:
	Sends a Byte to the slave.
	
Arguments:
	8 bit date to send to the slave.
	
Returns:
	non zero if slave acknowledge the data receipt.
	zero other wise.

**********************************************************/
tI2cStatus SoftI2CWriteByte(uint8_t data);

/**********************************************************
SoftI2CReadByte()

Description:
	Reads a byte from slave.
	
Arguments:
	1 if you want to acknowledge the receipt to slave.
	0 if you don't want to acknowledge the receipt to slave.
	
Returns:
	The 8 bit data read from the slave.

**********************************************************/
tI2cStatus SoftI2CReadByte(uint8_t *data, bool ack);


#endif 