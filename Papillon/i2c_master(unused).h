#ifndef I2C_MASTER_H_
#define I2C_MASTER_H_
/*******************************************************************************
*                                  INCLUDES                                    *
*******************************************************************************/
#include <stdint.h>
/*******************************************************************************
*								   DEFINES									   *
*******************************************************************************/
#define F_SCL_STD 400000UL // SCL frequency
#define F_SCL_FST 400000UL // SCL frequency
#define Prescaler 1
#define TWBR_val_STD ((((F_CPU / F_SCL_STD) / Prescaler) - 16 ) / 2)
#define TWBR_val_FST ((((F_CPU / F_SCL_FST) / Prescaler) - 16 ) / 2)

#define I2C_READ 0x01
#define I2C_WRITE 0x00

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
void i2c_init(void);
tI2cStatus i2c_start(uint8_t address);
tI2cStatus i2c_write(uint8_t data);
uint8_t i2c_read_ack(void);
uint8_t i2c_read_nack(void);
tI2cStatus i2c_transmit(uint8_t address, uint8_t* data, uint16_t length);
tI2cStatus i2c_receive(uint8_t address, uint8_t* data, uint16_t length);
tI2cStatus i2c_writeReg(uint8_t devaddr, uint8_t regaddr, uint8_t* data, uint16_t length);
tI2cStatus i2c_readReg(uint8_t devaddr, uint8_t regaddr, uint8_t* data, uint16_t length);
void i2c_stop(void);
tI2cStatus i2c_getStatus();

#endif /* I2C_MASTER_H_ */