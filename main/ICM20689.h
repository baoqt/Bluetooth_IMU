#ifndef ICM20689_H
#define ICM20689_H

#define FIFO_DEPTH			32

#define Kp 2.0f * 5.0f								// Free parameters in the Mahony filter and fusion scheme
#define Ki 0.0f

#include <stdlib.h>
#include <stdint.h>
#include <float.h>
#include <math.h>

#ifndef __FREERTOS__
#define __FREERTOS__
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#endif

#include "driver/spi_master.h"

typedef struct measurement
{
	volatile float AXoff;										    // Accelerometer offset values
	volatile float AYoff;
	volatile float AZoff;

	volatile float GXoff;										    // Gyroscope offset values
	volatile float GYoff;
	volatile float GZoff;

	volatile int16_t CAX;
	volatile int16_t CAY;
	volatile int16_t CAZ;

	volatile int16_t CGX;
	volatile int16_t CGY;
	volatile int16_t CGZ;

	volatile uint8_t* FIFO;
}measurement;

#define ICM20689_SELF_TEST_X_GYRO_REG					0x00
#define ICM20689_SELF_TEST_Y_GYRO_REG					0x01
#define ICM20689_SELF_TEST_Z_GYRO_REG					0x02

#define ICM20689_SELF_TEST_X_ACCEL_REG					0x0D
#define ICM20689_SELF_TEST_Y_ACCEL_REG					0x0E
#define ICM20689_SELF_TEST_Z_ACCEL_REG					0x0F

#define ICM20689_XG_OFFS_USRH_REG						0x13
#define ICM20689_XG_OFFS_USRL_REG						0x14
#define ICM20689_YG_OFFS_USRH_REG						0x15
#define ICM20689_YG_OFFS_USRL_REG						0x16
#define ICM20689_ZG_OFFS_USRH_REG						0x17
#define ICM20689_ZG_OFFS_USRL_REG						0x18
#define ICM20689_SMPLRT_DIV_REG							0x19
/**
 * @brief 	General config register
 * 
 * @param	FIFO_MODE:		Set for FIFO snapshot, clear for FIFO stream
 * @param	EXT_SYNC_SET:	Use for FSYNC bit selection
 * @param	DLPF_CFG:		Low pass filter selection
 * 
 */
#define ICM20689_CONFIG_REG								0x1A
#define ICM20689_CONFIG_FIFO_MODE_MASK					0x40
#define ICM20689_CONFIG_EXT_SYNC_SET_MASK				0x38
#define ICM20689_CONFIG_DLPF_CFG_MASK					0x07
/**
 * @brief 	Gyro config register
 * 
 * @param	XG_ST:				X Gyro self-test
 * @param	YG_ST:				Y Gyro self-test
 * @param	ZG_ST:				Z Gyro self-test
 * @param	FS_SEL:				Gyro full scale select
 * @param	FCHOICE_B:			Used to bypass DLPF
 * 
 */
#define ICM20689_GYRO_CONFIG_REG						0x1B
#define ICM20689_GYRO_CONFIG_XG_ST_MASK					0x80
#define ICM20689_GYRO_CONFIG_YG_ST_MASK					0x40
#define ICM20689_GYRO_CONFIG_ZG_ST_MASK					0x20
/**
 * @brief 	Gyro full scale select
 * 
 * @param	00:					+/-250DPS
 * @param	01:					+/-500DPS
 * @param	10:					+/-1000DPS
 * @param	11:					+/-2000DPS
 * 
 */
#define ICM20689_GYRO_CONFIG_FS_SEL_MASK				0x18
#define ICM20689_GYRO_CONFIG_FCHOICE_B_MASK				0x03
/**
 * @brief 	Accel config register
 * 
 * @param	XA_ST:				X Accel self-test
 * @param	YA_ST:				Y Accel self-test
 * @param	ZA_ST:				Z Accel self-test
 * @param	ACCEL_FS_SEL:		Accel full scale select
 * 
 */
#define ICM20689_ACCEL_CONFIG_REG						0x1C
#define ICM20689_ACCEL_CONFIG_XA_ST_MASK				0x80
#define ICM20689_ACCEL_CONFIG_YA_ST_MASK				0x40
#define ICM20689_ACCEL_CONFIG_ZA_ST_MASK				0x20
/**
 * @brief 	Accel full scale select
 * 
 * @param	00:					+/-2G
 * @param	01:					+/-4G
 * @param	10:					+/-8G
 * @param	11:					+/-16G
 * 
 */
#define ICM20689_ACCEL_CONFIG_ACCEL_FS_SEL_MASK			0x18
/**
 * @brief 	Accel config register 2
 * 
 * @param	DEC2_CFG:			Averaging filter settings for Low Power Accelerometer mode
 * @param	ACCEL_FCHOICE_B:	Used to bypass DLPF
 * @param	A_DLPF_CFG:			Accelerometer low pass filter setting
 * 
 */
#define ICM20689_ACCEL_CONFIG_2_REG						0x1D
/**
 * @brief 	DEC2_CFG Low Power Accelerometer mode averaging filter settings
 * 
 * @param	00:					Averaging 4 samples
 * @param	01:					Averaging 8 samples
 * @param	10:					Averaging 16 samples
 * @param	11:					Averaging 32 samples
 * 
 */
#define ICM20689_ACCEL_CONFIG_2_DEC2_CFG_MASK			0x30
#define ICM20689_ACCEL_CONFIG_2_ACCEL_FCHOICE_B_MASK	0x08
#define ICM20689_ACCEL_CONFIG_2_A_DLPF_CFG_MASK			0x07
/**
 * @brief 	Low power mode configuration register
 * 
 * @param	GYRO_CYCLE:			Set to enable low power gyro. Default setting is 0
 * @param	G_AVG_CFG:			Averaging filter setting for low power gyro. Default setting is 000
 * 
 */
#define ICM20689_LP_MODE_CFG_REG						0x1E
#define ICM20689_LP_MODE_CFG_GYRO_CYCLE_MASK			0x80
#define ICM20689_LP_MODE_CFG_G_AVGCFG_MASK				0x70
#define ICM20689_ACCEL_WOM_THR_REG						0x1F

/**
 * @brief 	FIFO enable register
 * 
 * @param	TEMP_FIFO_EN:		Set to write temperature data to FIFO, clear to disable
 * @param	XG_FIFO_EN:			Set to write X gyro data to FIFO, clear to disable
 * @param	YG_FIFO_EN:			Set to write Y gyro data to FIFO, clear to disable
 * @param	ZG_FIFO_EN:			Set to write Z gyro data to FIFO, clear to disable
 * @param	ACCEL_FIFO_EN:		Set to write accel data to FIFO, clear to disable
 * 
 */
#define ICM20689_FIFO_EN_REG							0x23
#define ICM20689_FIFO_EN_TEMP_FIFO_EN_MASK				0x80
#define ICM20689_FIFO_EN_XG_FIFO_EN_MASK				0x40
#define ICM20689_FIFO_EN_YG_FIFO_EN_MASK				0x20
#define ICM20689_FIFO_EN_ZG_FIFO_EN_MASK				0x10
#define ICM20689_FIFO_EN_ACCEL_FIFO_EN_MASK				0x08

#define ICM20689_FSYNC_INT_REG							0x36
#define ICM20689_INT_PIN_CFG_REG						0x37
#define ICM20689_INT_ENABLE_REG							0x38
#define ICM20689_DMP_INT_STATUS_REG						0x39
#define ICM20689_INT_STATUS_REG							0x3A
#define ICM20689_ACCEL_XOUT_H_REG						0x3B
#define ICM20689_ACCEL_XOUT_L_REG						0x3C
#define ICM20689_ACCEL_YOUT_H_REG						0x3D
#define ICM20689_ACCEL_YOUT_L_REG						0x3E
#define ICM20689_ACCEL_ZOUT_H_REG						0x3F
#define ICM20689_ACCEL_ZOUT_L_REG						0x40
#define ICM20689_TEMP_OUT_H_REG							0x41
#define ICM20689_TEMP_OUT_L_REG							0x42
#define ICM20689_GYRO_XOUT_H_REG						0x43
#define ICM20689_GYRO_XOUT_L_REG						0x44
#define ICM20689_GYRO_YOUT_H_REG						0x45
#define ICM20689_GYRO_YOUT_L_REG						0x46
#define ICM20689_GYRO_ZOUT_H_REG						0x47
#define ICM20689_GYRO_ZOUT_L_REG						0x48

#define ICM20689_SIGNAL_PATH_RESET_REG					0x68
#define ICM20689_ACCEL_INTEL_CTRL_REG					0x69
/**
 * @brief 	User control register
 * 
 * @param	DMP_EN:				Enables DMP
 * @param	FIFO_EN:			Set to enable FIFO operation, clear to disable FIFO access from serial interface
 * @param	I2C_IF_DIS:			Set to disable I2C slave module and put serial device in SPI mode only
 * @param 	DMP_RST:			Resets DMP
 * @param	FIFO_RST:			Set to reset FIFO, auto clears
 * @param	SIG_COND_RST:		Set to reset all digital signal paths, also clears all sensor registers
 * 
 */
#define ICM20689_USER_CTRL_REG							0x6A
#define	ICM20689_USER_CTRL_DMP_EN_MASK					0x80
#define	ICM20689_USER_CTRL_FIFO_EN_MASK					0x40
#define	ICM20689_USER_CTRL_I2C_IF_DIS_MASK				0x10
#define	ICM20689_USER_CTRL_DMP_RST_MASK					0x08
#define	ICM20689_USER_CTRL_FIFO_RST_MASK				0x04
#define	ICM20689_USER_CTRL_SIG_COND_RST_MASK			0x01
/**
 * @brief 	Power management register 1
 * 
 * @param	DEVICE_RESET:		Resets internal registers to default values, auto clears
 * @param	SLEEP:				Set to enter sleep mode, clear to disable, default setting is in sleep mode
 * @param	ACCEL_CYCLE:		When set, and if SLEEP and STANDBY are not set, chip will alternate between sleep and sampling gyro at SMPLRT_DIV
 * @param	GYRO_STANDBY:		When set, enables low power quick access of gyro circuitry
 * @param	TEMP_DIS:			When set, disables temperature sensor
 * @param	CLKSEL:				Selects clock source, should be set to 001 for full gyro performance
 * 
 */
#define ICM20689_PWR_MGMT_1_REG							0x6B
#define ICM20689_PWR_MGMT_1_DEVICE_RESET_MASK			0x80
#define ICM20689_PWR_MGMT_1_SLEEP_MASK					0x40
#define ICM20689_PWR_MGMT_1_ACCEL_CYCLE_MASK			0x20
#define ICM20689_PWR_MGMT_1_GYRO_STANDBY_MASK			0x10
#define ICM20689_PWR_MGMT_1_TEMP_DIS_MASK				0x08
#define ICM20689_PWR_MGMT_1_CLKSEL_MASK					0x07
#define ICM20689_PWR_MGMT_2_REG							0x6C
#define ICM20689_PWR_MGMT_2_FIFO_LP_EN_MASK				0x80
#define ICM20689_PWR_MGMT_2_DMP_LP_DIS_MASK				0x40
#define ICM20689_PWR_MGMT_2_SENSOR_STANDBY_MASK			0x3F

#define ICM20689_FIFO_COUNTH_REG						0x72
#define ICM20689_FIFO_COUNTL_REG						0x73
#define ICM20689_FIFO_R_W_REG							0x74
#define ICM20689_WHO_AM_I_REG							0x75

#define ICM20689_XA_OFFSET_H_REG						0x77
#define ICM20689_XA_OFFSET_L_REG						0x78

#define ICM20689_YA_OFFSET_H_REG						0x7A
#define ICM20689_YA_OFFSET_L_REG						0x7B

#define ICM20689_ZA_OFFSET_H_REG						0x7D
#define ICM20689_ZA_OFFSET_L_REG						0x7E



void ICM20689_getMeas(volatile measurement* meas);

void ICM20689_calibrate(volatile measurement* meas, int trials);
void ICM20689_SelfTest(volatile measurement* meas);

void ICM20689_setGyroSelfTest();
void ICM20689_setAccelSelfTest();
void ICM20689_clearGyroSelfTest();
void ICM20689_clearAccelSelfTest();
void ICM20689_setFullScaleGyroRange(uint8_t range);
void ICM20689_setFullScaleAccelRange(uint8_t range);
void ICM20689_setSampleRate(uint16_t rate);

void ICM20689_toggleFIFO(uint8_t state);
void ICM20689_toggleSensors(uint8_t state);
int ICM20689_readFullFIFO(volatile measurement* meas);

void ICM20689_ConfigureLowPassFilter();
void ICM20689_setSleepDisabled();
void ICM20689_whoAmI();

void ICM20689_init(volatile measurement* meas);

#endif