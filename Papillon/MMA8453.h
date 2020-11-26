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
//============================================ i2c addresses ====
#define I2C_MMA8453_ADDR	(0b00111010)
#define I2C_MMA8453_READ	(I2C_MMA8453_ADDR | I2C_READ)
#define I2C_MMA8453_WRITE	(I2C_MMA8453_ADDR | I2C_WRITE)

//============================================ Registers ====
/* Status & data */
#define MMA_STATUS			0x00
#define MMA_OUT_X_MSB		0x01
#define MMA_OUT_X_LSB		0x02
#define MMA_OUT_Y_MSB		0x03
#define MMA_OUT_Y_LSB		0x04
#define MMA_OUT_Z_MSB		0x05
#define MMA_OUT_Z_LSB		0x06
#define MMA_SYSMOD			0x0B
#define MMA_INT_SOURCE		0x0C
#define MMA_XYZ_DATA_CFG	0x0E
#define MMA_HP_CUTOFF		0x0F
/* Transient detection */
#define MMA_TRANSIENT_CFG	0x1D
#define MMA_TRANSIENT_SRC	0x1E
#define MMA_TRANSIENT_THS	0x1F
#define MMA_TRANSIENT_COUNT	0x20
/* Configuration registers & fields */
#define MMA_ASLP_COUNT		0x29
#define MMA_CTRL_REG1		0x2A
	#define ASPL_RATE1			7
	#define ASPL_RATE0			6
	#define DATA_RATE2			5
	#define DATA_RATE1			4
	#define DATA_RATE0			3
	#define LOW_NOISE			2
	#define FAST_READ			1
	#define ACTIVE_MODE			0
	#define ASPL_50HZ			(0)
	#define ASPL_12_5HZ			(1)
	#define ASPL_6_25HZ			(2)
	#define ASPL_1_56HZ			(3)
	#define WAKE_800HZ			(0)
	#define WAKE_400HZ			(1)
	#define WAKE_200HZ			(2)
	#define WAKE_100HZ			(3)
	#define WAKE_50HZ			(4)
	#define WAKE_12_5HZ			(5)
	#define WAKE_6_25HZ			(6)
	#define WAKE_1_56HZ			(7)
#define MMA_CTRL_REG2		0x2B
#define MMA_CTRL_REG3		0x2C
#define MMA_CTRL_REG4		0x2D
#define MMA_CTRL_REG5		0x2E
/* Registers predefined values */
#define MMA_CTRL_REG1_STB	(ASPL_12_5HZ<<ASPL_RATE0 | WAKE_400HZ<<DATA_RATE0 | 1<<LOW_NOISE | 1<<FAST_READ)	// set 12.5Hz sleep, 800Hz wake, active mode
#define MMA_CTRL_REG1_ACT	(MMA_CTRL_REG1_STB | 1<<ACTIVE_MODE)
namespace accel{
/*******************************************************************************
*								  TYPEDEFS									   *
*******************************************************************************/

/*******************************************************************************
*								 VARIABLES									   *
*******************************************************************************/
extern bool autoSleep;
/*******************************************************************************
*								 Prototypes									   *
*******************************************************************************/
void init();
tI2cStatus checkIntSource();
tI2cStatus enableTransientIntLatch();
tI2cStatus disableTransientIntLatch();
tI2cStatus enableAutoSleep();
tI2cStatus disableAutoSleep();
tI2cStatus move(uint8_t *x, uint8_t *y, uint8_t *z);

}
#endif /* MMA8453_H_ */