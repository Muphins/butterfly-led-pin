#ifndef MMA8453_H_
#define MMA8453_H_
/*******************************************************************************
*                                  INCLUDES                                    *
*******************************************************************************/
//#include "TinyWire.h"
#include "i2cSoft.h"

/*******************************************************************************
*								   DEFINES									   *
*******************************************************************************/
#define I2C_MMA8453_ADDR	(0b00111010)
#define I2C_MMA8453_READ	(I2C_MMA8453_ADDR | I2C_READ)
#define I2C_MMA8453_WRITE	(I2C_MMA8453_ADDR | I2C_WRITE)

#define MMA_STATUS			0x00
#define MMA_OUT_X_MSB		0x01
#define MMA_OUT_X_LSB		0x02
#define MMA_OUT_Y_MSB		0x03
#define MMA_OUT_Y_LSB		0x04
#define MMA_OUT_Z_MSB		0x05
#define MMA_OUT_Z_LSB		0x06
#define MMA_INT_SOURCE		0x0C
#define MMA_XYZ_DATA_CFG	0x0E
#define MMA_HP_CUTOFF		0x0F

#define MMA_TRANSIENT_CFG	0x1D
#define MMA_TRANSIENT_SRC	0x1E
#define MMA_TRANSIENT_THS	0x1F
#define MMA_TRANSIENT_COUNT	0x20

#define MMA_ASLP_COUNT		0x29
#define MMA_CTRL_REG1		0x2A
#define MMA_CTRL_REG2		0x2B
#define MMA_CTRL_REG3		0x2C
#define MMA_CTRL_REG4		0x2D
#define MMA_CTRL_REG5		0x2E

namespace accel{
/*******************************************************************************
*								  TYPEDEFS									   *
*******************************************************************************/

/*******************************************************************************
*								 VARIABLES									   *
*******************************************************************************/

/*******************************************************************************
*								 Prototypes									   *
*******************************************************************************/
void init();
tI2cStatus checkIntSource();
tI2cStatus enableTransientIntLatch();
tI2cStatus disableTransientIntLatch();
tI2cStatus move(uint8_t *x, uint8_t *y, uint8_t *z);

}
#endif /* MMA8453_H_ */