/*******************************************************************************
*                                  INCLUDES                                    *
*******************************************************************************/
#include <avr/io.h>
#include <util/twi.h>

#include "i2c_master.h"

// #define PRTWI	PRTWI1
// #define TWBR	TWBR1
// #define TWCR	TWCR1
// #define TWSR	TWSR1
// #define TWDR	TWDR1

/*******************************************************************************
*								   VARIABLES								   *
*******************************************************************************/
static tI2cStatus m_i2cMasterStatus = I2cOk;
/*******************************************************************************
*                                    CODE                                      *
*******************************************************************************/
void i2c_init(void)
{
//	PRR0 &= ~(1<<PRTWI);
	TWBR = 4;//(uint8_t)TWBR_val_STD;
	m_i2cMasterStatus = I2cIdle;
}

tI2cStatus i2c_start(uint8_t address)
{
	// reset TWI control register
	TWCR = 0;
	// transmit START condition
	TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);
	// wait for end of transmission
	while( !(TWCR & (1<<TWINT)) );
	
	// check if the start condition was successfully transmitted
	if((TWSR & 0xF8) != TW_START){
		m_i2cMasterStatus=I2cStartFail;
		i2c_stop();
		return I2cNok;
	}
	
	// load slave address into data register
	TWDR = address;
	// start transmission of address
	TWCR = (1<<TWINT) | (1<<TWEN);
	// wait for end of transmission
	while( !(TWCR & (1<<TWINT)) );
	
	// check if the device has acknowledged the READ / WRITE mode
	uint8_t twst = TW_STATUS & 0xF8;
	if ( (twst != TW_MT_SLA_ACK) && (twst != TW_MR_SLA_ACK) ){
		m_i2cMasterStatus=I2cAddrFail;
		i2c_stop();
		return I2cAddrFail;
	}
	m_i2cMasterStatus=I2cStartOk; 
	return I2cOk;
}

tI2cStatus i2c_write(uint8_t data)
{
	// load data into data register
	TWDR = data;
	// start transmission of data
	TWCR = (1<<TWINT) | (1<<TWEN);
	// wait for end of transmission
	while( !(TWCR & (1<<TWINT)) );
	
// 	for(char i=0; ((TWSR & 0xF8) != TW_MT_DATA_ACK) && (i<100); i++ ){
// 		_delay_us(1);
// 	}
	if((TWSR & 0xF8) != TW_MT_DATA_ACK){
		m_i2cMasterStatus = I2cWriteFail;
		i2c_stop();
		return I2cNok;
	}
	
	m_i2cMasterStatus = I2cWriteOk;
	return I2cOk;
}

uint8_t i2c_read_ack(void)
{
	
	// start TWI module and acknowledge data after reception
	TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWEA);
	// wait for end of transmission
	while( !(TWCR & (1<<TWINT)) );
	// return received data from TWDR
	return TWDR;
}

uint8_t i2c_read_nack(void)
{
	
	// start receiving without acknowledging reception
	TWCR = (1<<TWINT) | (1<<TWEN);
	// wait for end of transmission
	while( !(TWCR & (1<<TWINT)) );
	// return received data from TWDR
	return TWDR;
}

tI2cStatus i2c_transmit(uint8_t address, uint8_t* data, uint16_t length)
{
	if (i2c_start(address | I2C_WRITE)){
		return I2cNok;
	}
	for (uint16_t i = 0; i < length; i++)
	{
		if (i2c_write(data[i])) return I2cNok;
	}
	i2c_stop();
	m_i2cMasterStatus = I2cWriteOk;
	return I2cOk;
}

tI2cStatus i2c_receive(uint8_t address, uint8_t* data, uint16_t length)
{
	if (i2c_start(address | I2C_READ)) return I2cNok;
	
	for (uint16_t i = 0; i < (length-1); i++)
	{
		data[i] = i2c_read_ack();
	}
	data[(length-1)] = i2c_read_nack();
	
	i2c_stop();
	m_i2cMasterStatus = I2cReadOk;
	return I2cOk;
}

tI2cStatus i2c_writeReg(uint8_t devaddr, uint8_t regaddr, uint8_t* data, uint16_t length)
{
//debugPrint("Start\n");
	if (i2c_start(devaddr | I2C_WRITE)) return I2cNok;

//debugPrint("Addr\n");
	i2c_write(regaddr);

	for (uint16_t i = 0; i < length; i++)
	{
//debugPrint("Data\n");
		if (i2c_write(data[i])) return I2cNok;
	}

//debugPrint("Stop\n");
	i2c_stop();
	
	m_i2cMasterStatus = I2cWriteOk;
	return I2cOk;
}

tI2cStatus i2c_readReg(uint8_t devaddr, uint8_t regaddr, uint8_t* data, uint16_t length)
{
	if (i2c_start(devaddr | I2C_WRITE)) return I2cNok;

	if (i2c_write(regaddr)) return I2cNok;

	if (i2c_start(devaddr | I2C_READ)) return I2cNok;

	for (uint16_t i = 0; i < (length-1); i++)
	{
		data[i] = i2c_read_ack();
	}
	data[(length-1)] = i2c_read_nack();

	i2c_stop();
	m_i2cMasterStatus = I2cReadOk;
	return I2cOk;
}

void i2c_stop(void)
{
	// transmit STOP condition
	TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO);
}

tI2cStatus i2c_getStatus()
{
	tI2cStatus result = m_i2cMasterStatus;
	m_i2cMasterStatus = I2cIdle;
	return result;
}