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
	#define TRANS_EV_LATCH		4
	#define TRANS_Z_EVFLAG		3
	#define TRANS_Y_EVFLAG		2
	#define TRANS_X_EVFLAG		1
	#define TRANS_HPF_BYP		0
	
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
	/* Values */
		/* Auto sleep sample rates */
	#define ASPL_50HZ			(0)
	#define ASPL_12_5HZ			(1)
	#define ASPL_6_25HZ			(2)
	#define ASPL_1_56HZ			(3)
		/* Awake sample rates */
	#define WAKE_800HZ			(0)
	#define WAKE_400HZ			(1)
	#define WAKE_200HZ			(2)
	#define WAKE_100HZ			(3)
	#define WAKE_50HZ			(4)
	#define WAKE_12_5HZ			(5)
	#define WAKE_6_25HZ			(6)
	#define WAKE_1_56HZ			(7)
	
#define MMA_CTRL_REG2		0x2B
	#define SELF_TEST			7
	#define SOFT_RESET			6
	#define SLEEP_MODS1			4
	#define SLEEP_MODS0			3
	#define AUTO_SLEEP			2
	#define WAKE_MODS1			1
	#define WAKE_MODS0			0
	/* Values */
	#define MODS_NORMAL			(0)
	#define MODS_LOW_NOISE		(1)
	#define MODS_HI_RES			(2)
	#define MODS_LOW_POWER		(3)
	
#define MMA_CTRL_REG3		0x2C		// On interrupt wake config
	#define INT_WAKE_TRANS			6
	#define INT_WAKE_ORIENT			5
	#define INT_WAKE_PULSE			4
	#define INT_WAKE_MOTION			3
	#define INT_WAKE_POLARITY		1
	#define INT_WAKE_PP_OD			0
	
#define MMA_CTRL_REG4		0x2D		// Interrupt source enable
	#define INT_EN_SLEEP			7
	#define INT_EN_TRANS			5
	#define INT_EN_ORIENT			4
	#define INT_EN_PULSE			3
	#define INT_EN_MOTION			2
	#define INT_EN_DATA_RDY			0
	
#define MMA_CTRL_REG5		0x2E		// Route interrupt to INT2 (0) or INT1 (1)
/* Registers predefined values */
#define MMA_CTRL_REG1_STB			(ASPL_12_5HZ<<ASPL_RATE0 | WAKE_200HZ<<DATA_RATE0 | 1<<LOW_NOISE | 1<<FAST_READ)
#define MMA_CTRL_REG1_ACT			(MMA_CTRL_REG1_STB | 1<<ACTIVE_MODE)

#define MMA_CTRL_REG2_DEFAULT		(MODS_LOW_NOISE<<SLEEP_MODS0 | 0<<AUTO_SLEEP | MODS_HI_RES<<WAKE_MODS0)
#define MMA_CTRL_REG3_DEFAULT		(1<<INT_WAKE_TRANS)
#define MMA_CTRL_REG4_DEFAULT		(1<<INT_EN_TRANS | 1<<INT_EN_DATA_RDY)

#define MMA_TRANSIENT_CFG_DEFAULT	(1<<TRANS_Z_EVFLAG | 1<<TRANS_Y_EVFLAG | 1<<TRANS_X_EVFLAG)
#define MMA_TRANSIENT_CFG_EVLATCH	(MMA_TRANSIENT_CFG_DEFAULT | 1<<TRANS_EV_LATCH)
namespace accel{
/*******************************************************************************
*								  TYPEDEFS									   *
*******************************************************************************/
enum tIntSource{	// to be used with register MMA_INT_SOURCE as masking bits
	IntASPL		= 1<<7,
	IntTRANS	= 1<<5,
	IntORIENT	= 1<<4,
	IntPULSE	= 1<<3,
	IntMOTION	= 1<<2,
	IntDRDY		= 1<<0
	};
/*******************************************************************************
*								 VARIABLES									   *
*******************************************************************************/
extern bool autoSleep;
/*******************************************************************************
*								 Prototypes									   *
*******************************************************************************/
void init();
tI2cStatus enableTransientIntLatch();
tI2cStatus disableTransientIntLatch();
tI2cStatus checkIntSource(uint8_t *intSource);
tI2cStatus readIntTransient();
tI2cStatus enableAutoSleep();
tI2cStatus disableAutoSleep();
tI2cStatus activeMode();
tI2cStatus standbyMode();
tI2cStatus getAcc(int8_t *x, int8_t *y, int8_t *z);

}
#endif /* MMA8453_H_ */