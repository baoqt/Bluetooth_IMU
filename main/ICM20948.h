#ifndef ICM20948_H
#define ICM20948_H

#define FIFO_DEPTH			32

#define Kp 2.0f * 5.0f								// Free parameters in the Mahony filter and fusion scheme
#define Ki 0.0f

#include <stdlib.h>
#include <stdint.h>
#include <float.h>
#include <math.h>

#include "I2C.h"

typedef struct measurement
{
	float AXoff;										// Accelerometer offset values
	float AYoff;
	float AZoff;

	float GXoff;										// Gyroscope offset values
	float GYoff;
	float GZoff;

	volatile int16_t CAX;
	volatile int16_t CAY;
	volatile int16_t CAZ;

	volatile int16_t CGX;
	volatile int16_t CGY;
	volatile int16_t CGZ;

	volatile float AX[FIFO_DEPTH];						// IMU measurement FIFO
	volatile float AY[FIFO_DEPTH];						// Accelerometer processed data
	volatile float AZ[FIFO_DEPTH];

	volatile float GX[FIFO_DEPTH];						// Gyroscope processed data
	volatile float GY[FIFO_DEPTH];
	volatile float GZ[FIFO_DEPTH];
	
	volatile int head;									// Entry of FIFO
	volatile int tail;									// Exit of FIFO
	volatile int size;									// Size of FIFO
}measurement;

typedef struct magnetometer
{
	int16_t CMX;										// Current magnometer values
	int16_t CMY;
	int16_t CMZ;

	float MXoff;										// Magnetometer offset values
	float MYoff;
	float MZoff;

	float MX;											// Magnetometer float values
	float MY;
	float MZ;
}magnetometer;

#define ICM20948_DEFAULT_ADDRESS		    		0x68
#define ICM20948_ALT_DEFAULT_ADDRESS	    		0x69

// Use to select user bank
#define ICM20948_REG_BANK_SEL             			0x7F
#define ICM20948_USER_BANK_0						0x00
#define ICM20948_USER_BANK_1						0x10
#define ICM20948_USER_BANK_2						0x20
#define ICM20948_USER_BANK_3						0x30

// User Bank 0 Register Map
#define ICM20948_WHO_AM_I				    		0x00

#define ICM20948_USER_CTRL                  		0x03
#define ICM20948_LP_CONFIG                  		0x05

#define ICM20948_PWR_MGMT_1                 		0x06
#define ICM20948_PWR_MGMT_1_SLEEP_MASK				0x40
#define ICM20948_PWR_MGMT_2                 		0x07

#define ICM20948_INT_PIN_CFG                		0x0F
#define ICM20948_INT_ENABLE                 		0x10
#define ICM20948_INT_ENABLE_1               		0x11
#define ICM20948_INT_ENABLE_2               		0x12
#define ICM20948_INT_ENABLE_3               		0x13

#define ICM20948_I2C_MST_STATUS             		0x17

#define ICM20948_INT_STATUS                 		0x19
#define ICM20948_INT_STATUS_1               		0x1A
#define ICM20948_INT_STATUS_2               		0x1B
#define ICM20948_INT_STATUS_3               		0x1C

#define ICM20948_DELAY_TIMEH                		0x28
#define ICM20948_DELAY_TIMEL                		0x29

#define ICM20948_ACCEL_XOUT_H               		0x2D
#define ICM20948_ACCEL_XOUT_L               		0x2E
#define ICM20948_ACCEL_YOUT_H               		0x2F
#define ICM20948_ACCEL_YOUT_L               		0x30
#define ICM20948_ACCEL_ZOUT_H               		0x31
#define ICM20948_ACCEL_ZOUT_L               		0x32

#define ICM20948_GYRO_XOUT_H                		0x33
#define ICM20948_GYRO_XOUT_L                		0x34
#define ICM20948_GYRO_YOUT_H                		0x35
#define ICM20948_GYRO_YOUT_L                		0x36
#define ICM20948_GYRO_ZOUT_H                		0x37
#define ICM20948_GYRO_ZOUT_L                		0x38

#define ICM20948_TEMP_OUT_H                 		0x39
#define ICM20948_TEMP_OUT_L                 		0x3A

#define ICM20948_EXP_SLV_SENS_DATA_00       		0x3B
#define ICM20948_EXP_SLV_SENS_DATA_01       		0x3C
#define ICM20948_EXP_SLV_SENS_DATA_02       		0x3D
#define ICM20948_EXP_SLV_SENS_DATA_03       		0x3E
#define ICM20948_EXP_SLV_SENS_DATA_04       		0x3F
#define ICM20948_EXP_SLV_SENS_DATA_05       		0x40
#define ICM20948_EXP_SLV_SENS_DATA_06       		0x41
#define ICM20948_EXP_SLV_SENS_DATA_07       		0x42
#define ICM20948_EXP_SLV_SENS_DATA_08       		0x43
#define ICM20948_EXP_SLV_SENS_DATA_09       		0x44
#define ICM20948_EXP_SLV_SENS_DATA_10       		0x45
#define ICM20948_EXP_SLV_SENS_DATA_11       		0x46
#define ICM20948_EXP_SLV_SENS_DATA_12       		0x47
#define ICM20948_EXP_SLV_SENS_DATA_13       		0x48
#define ICM20948_EXP_SLV_SENS_DATA_14       		0x49
#define ICM20948_EXP_SLV_SENS_DATA_15       		0x4A
#define ICM20948_EXP_SLV_SENS_DATA_16       		0x4B
#define ICM20948_EXP_SLV_SENS_DATA_17       		0x4C
#define ICM20948_EXP_SLV_SENS_DATA_18       		0x4D
#define ICM20948_EXP_SLV_SENS_DATA_19       		0x4E
#define ICM20948_EXP_SLV_SENS_DATA_20       		0x4F
#define ICM20948_EXP_SLV_SENS_DATA_21       		0x50
#define ICM20948_EXP_SLV_SENS_DATA_22       		0x51
#define ICM20948_EXP_SLV_SENS_DATA_23       		0x52

#define ICM20948_FIFO_EN_1                  		0x66
#define ICM20948_FIFO_EN_2                  		0x67
#define ICM20948_FIFO_RST                   		0x68
#define ICM20948_FIFO_MODE                  		0x69
#define ICM20948_FIFO_COUNT_H               		0x70
#define ICM20948_FIFO_COUNT_L               		0x71
#define ICM20948_FIFO_R_W                   		0x72

#define ICM20948_DATA_RDY_STATUS            		0x74

#define ICM20948_FIFO_CFG                   		0x76

// User Bank 1 Register Map
#define ICM20948_SELF_TEST_X_GYRO           		0x02
#define ICM20948_SELF_TEST_Y_GYRO           		0x03
#define ICM20948_SELF_TEST_Z_GYRO           		0x04

#define ICM20948_SELF_TEST_X_ACCEL          		0x0E
#define ICM20948_SELF_TEST_Y_ACCEL          		0x0F
#define ICM20948_SELF_TEST_Z_ACCEL          		0x10

#define ICM20948_XA_OFFS_H                  		0x14
#define ICM20948_XA_OFFS_L                  		0x15
#define ICM20948_YA_OFFS_H                  		0x17
#define ICM20948_YA_OFFS_L                  		0x18
#define ICM20948_ZA_OFFS_H                  		0x1A
#define ICM20948_ZA_OFFS_L                  		0x1B

#define ICM20948_TIMEBASE_CORRECTION_PLL    		0x28

// User Bank 2 Register Map
#define ICM20948_GYRO_SMPLRT_DIV            		0x00
#define ICM20948_GYRO_CONFIG_1              		0x01
#define ICM20948_GYRO_CONFIG_1_FS_SEL_MASK			0x06
#define ICM20948_GYRO_CONFIG_1_FS_250_DPS			0x00
#define ICM20948_GYRO_CONFIG_1_FS_500_DPS			0x01
#define ICM20948_GYRO_CONFIG_1_FS_1000_DPS			0x02
#define ICM20948_GYRO_CONFIG_1_FS_2000_DPS			0x03
#define ICM20948_GYRO_CONFIG_2              		0x02
#define ICM20948_GYRO_CONFIG_2_GX_ST_EN_REG_MASK	0x20
#define ICM20948_GYRO_CONFIG_2_GY_ST_EN_REG_MASK	0x10
#define ICM20948_GYRO_CONFIG_2_GZ_ST_EN_REG_MASK	0x08

#define ICM20948_XG_OFFS_USR_H              		0x03
#define ICM20948_XG_OFFS_USR_L              		0x04
#define ICM20948_YG_OFFS_USR_H             		 	0x05
#define ICM20948_YG_OFFS_USR_L              		0x06
#define ICM20948_ZG_OFFS_USR_H              		0x07
#define ICM20948_ZG_OFFS_USR_L              		0x08

#define ICM20948_ODR_ALIGN_EN               		0x09

#define ICM20948_ACCEL_SMPLRT_DIV_1         		0x10
#define ICM20948_ACCEL_SMPLRT_DIV_2         		0x11
#define ICM20948_ACCEL_INTEL_CTRL           		0x12
#define ICM20948_ACCEL_WOM_THR              		0x13
#define ICM20948_ACCEL_CONFIG_1             		0x14
#define ICM20948_ACCEL_CONFIG_1_DLPFCFG_MASK		0x38
#define ICM20948_ACCEL_CONFIG_1_FS_SEL_MASK			0x06
#define ICM20948_ACCEL_CONFIG_1_FS_2G				0x00
#define ICM20948_ACCEL_CONFIG_1_FS_4G				0x01
#define ICM20948_ACCEL_CONFIG_1_FS_8G				0x02
#define ICM20948_ACCEL_CONFIG_1_FS_16G				0x03
#define ICM20948_ACCEL_CONFIG_1_FCHOICE_MASK		0x01
#define ICM20948_ACCEL_CONFIG_2             		0x15
#define ICM20948_ACCEL_CONFIG_2_DEC3_CFG_MASK		0x03
#define ICM20948_ACCEL_CONFIG_2_AX_ST_EN_REG_MASK	0x10
#define ICM20948_ACCEL_CONFIG_2_AY_ST_EN_REG_MASK	0x08
#define ICM20948_ACCEL_CONFIG_2_AZ_ST_EN_REG_MASK	0x04


#define ICM20948_FSYNC_CONFIG               		0x52

#define ICM20948_TEMP_CONFIG                		0x53

#define ICM20948_MOD_CTRL_USR               		0x54

// User Bank 3 Register Map
#define ICM20948_I2C_MST_ODR_CONFIG         		0x00
#define ICM20948_I2C_MST_CTRL               		0x01
#define ICM20948_I2C_MST_DELAY_CTRL         		0x02
#define ICM20948_I2C_SLV_0_ADDR             		0x03
#define ICM20948_I2C_SLV_0_REG              		0x04
#define ICM20948_I2C_SLV_0_CTRL             		0x05
#define ICM20948_I2C_SLV_0_DO               		0x06
#define ICM20948_I2C_SLV_1_ADDR             		0x07
#define ICM20948_I2C_SLV_1_REG              		0x08
#define ICM20948_I2C_SLV_1_CTRL             		0x09
#define ICM20948_I2C_SLV_1_DO               		0x0A
#define ICM20948_I2C_SLV_2_ADDR             		0x0B
#define ICM20948_I2C_SLV_2_REG              		0x0C
#define ICM20948_I2C_SLV_2_CTRL             		0x0D
#define ICM20948_I2C_SLV_2_DO               		0x0E
#define ICM20948_I2C_SLV_3_ADDR             		0x0F
#define ICM20948_I2C_SLV_3_REG              		0x10
#define ICM20948_I2C_SLV_3_CTRL             		0x11
#define ICM20948_I2C_SLV_3_DO               		0x12
#define ICM20948_I2C_SLV_4_ADDR             		0x13
#define ICM20948_I2C_SLV_4_REG              		0x14
#define ICM20948_I2C_SLV_4_CTRL             		0x15
#define ICM20948_I2C_SLV_4_DO               		0x16
#define ICM20948_I2C_SLV_4_DI               		0x17

// AK09916 Magnetometer Register Bank
#define AK09916_DEFAULT_ADDRESS						0x0C

#define AK09916_DEVICE_ID                   		0x01
#define AK09916_STATUS_1                    		0x10

#define AK09916_MAG_XOUT_L                  		0x11
#define AK09916_MAG_XOUT_H                  		0x12
#define AK09916_MAG_YOUT_L                  		0x13
#define AK09916_MAG_YOUT_H                  		0x14
#define AK09916_MAG_ZOUT_L                  		0x15
#define AK09916_MAG_ZOUT_H                  		0x16

#define AK09916_STATUS_2                    		0x18
#define AK09916_CONTROL_2                   		0x31
#define AK09916_CONTROL_2_MODE_MASK					0x1F
#define AK09916_CONTROL_2_POWER_DOWN_MODE			0x00
#define AK09916_CONTROL_2_CONT_MEAS_MODE_1			0x02
#define AK09916_CONTROL_2_CONT_MEAS_MODE_2			0x04
#define AK09916_CONTROL_2_CONT_MEAS_MODE_3			0x05
#define AK09916_CONTROL_2_CONT_MEAS_MODE_4			0x08
#define AK09916_CONTROL_2_SELF_TEST_MODE			0x10
#define AK09916_CONTROL_3                   		0x32

void ICM20948_getMeas(volatile measurement* meas);
void ICM20948_getComp(magnetometer* magno);

void ICM20948_calibrate(volatile measurement* meas, magnetometer* magno, int trials);
void ICM20948_SelfTest(volatile measurement* meas);

void ICM20948_setGyroSelfTest();
void ICM20948_setAccelSelfTest();
void ICM20948_clearGyroSelfTest();
void ICM20948_clearAccelSelfTest();
void ICM20948_setFullScaleGyroRange(uint8_t range);
void ICM20948_setFullScaleAccelRange(uint8_t range);
uint8_t ICM20948_getFullScaleAccelRange();

void ICM20948_ConfigureAccelerometerLowPassFilter();
void ICM20948_setMagContinuousMeasurementMode();
void ICM20948_setMagPowerDown();
void ICM20948_setSleepDisabled();

void ICM20948_init(volatile measurement* meas, magnetometer* magno);

#endif