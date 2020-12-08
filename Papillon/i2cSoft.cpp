/**********************************************************

Software I2C Library for AVR Devices.

Copyright 2008-2012
eXtreme Electronics, India
www.eXtremeElectronics.co.in
**********************************************************/

#include <avr/io.h>
#include <util/delay.h>

#include "i2csoft.h"

#define Q_DEL _delay_loop_2(1)	//3
#define H_DEL _delay_loop_2(1)	//5

static tI2cStatus m_i2cStatus = I2cIdle;

void SoftI2CInit()
{
	SDAPORT&=(1<<SDA);
	SCLPORT&=(1<<SCL);
	
	SOFT_I2C_SDA_HIGH;
	SOFT_I2C_SCL_HIGH;
	m_i2cStatus = I2cIdle;
}
void SoftI2CStart()
{
	SOFT_I2C_SCL_HIGH;
	H_DEL;
	
	SOFT_I2C_SDA_LOW;
	H_DEL;
//	m_i2cStatus = I2cStartOk;
}

void SoftI2CStop()
{
	SOFT_I2C_SDA_LOW;
	H_DEL;
	SOFT_I2C_SCL_HIGH;
	Q_DEL;
	SOFT_I2C_SDA_HIGH;
	H_DEL;
}

tI2cStatus SoftI2CWriteByte(uint8_t data)
{
	
	uint8_t i;
	
	for(i=0;i<8;i++)
	{
		SOFT_I2C_SCL_LOW;
		Q_DEL;
		
		if(data & 0x80)
			SOFT_I2C_SDA_HIGH;
		else
			SOFT_I2C_SDA_LOW;
		
		H_DEL;
		
		SOFT_I2C_SCL_HIGH;
		H_DEL;
		
		while((SCLPIN & (1<<SCL))==0);
		
		data=data<<1;
	}
	
	//The 9th clock (ACK Phase)
	SOFT_I2C_SCL_LOW;
	Q_DEL;
	
	SOFT_I2C_SDA_HIGH;
	H_DEL;
	
	SOFT_I2C_SCL_HIGH;
	H_DEL;
	
	tI2cStatus ack = I2cWriteFail;
	if(!(SDAPIN & (1<<SDA))) ack = I2cOk;
	
	SOFT_I2C_SCL_LOW;
	H_DEL;
	
	if(ack != I2cOk) m_i2cStatus = ack;
	return ack;
}


tI2cStatus SoftI2CReadByte(uint8_t *data, bool ack)
{
	*data=0x00;
	uint8_t i;
	
	for(i=0;i<8;i++)
	{
		
		SOFT_I2C_SCL_LOW;
		H_DEL;
		SOFT_I2C_SCL_HIGH;
		H_DEL;
		
		while((SCLPIN & (1<<SCL))==0);
		
		if(SDAPIN &(1<<SDA))
		*data|=(0x80>>i);
		
	}
	
	SOFT_I2C_SCL_LOW;
	Q_DEL;						//Soft_I2C_Put_Ack
	
	if(ack)
	{
		SOFT_I2C_SDA_LOW;
	}
	else
	{
		SOFT_I2C_SDA_HIGH;
	}
	H_DEL;
	
	SOFT_I2C_SCL_HIGH;
	H_DEL;
	
	SOFT_I2C_SCL_LOW;
	H_DEL;
	
//	m_i2cStatus = I2cReadOk;
	return I2cOk;
}

void SoftI2CError(tI2cStatus error)
{
	if(m_i2cStatus == I2cIdle || m_i2cStatus == I2cOk){
		if(error == I2cOk){
			m_i2cStatus = I2cOk;
			return;
		}
		m_i2cStatus = error;
	}
}

tI2cStatus SoftI2CStatus()
{
	tI2cStatus tmp = m_i2cStatus;
	m_i2cStatus = I2cIdle;
	return tmp;
	//return I2cIdle;
}
