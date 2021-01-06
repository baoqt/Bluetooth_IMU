#include "ICM20948.h"

I2C_msg_t msg = {.device_addr = ICM20948_DEFAULT_ADDRESS,
				 .register_addr = ICM20948_WHO_AM_I,
				 .data = NULL,
				 .length = 1};

I2C_msg_t get = {.device_addr = ICM20948_DEFAULT_ADDRESS,
				 .register_addr = ICM20948_ACCEL_XOUT_H,
				 .data = NULL,
				 .length = 12};

void ICM20948_getMeas(volatile measurement* meas)
{
	I2C0_read(&get);

	meas->CAX = ((int16_t) get.data[0] << 8) | get.data[1];
	meas->CAY = ((int16_t) get.data[2] << 8) | get.data[3];
	meas->CAZ = ((int16_t) get.data[4] << 8) | get.data[5];
	meas->CGX = ((int16_t) get.data[6] << 8) | get.data[7]; 
	meas->CGY = ((int16_t) get.data[8] << 8) | get.data[9];
	meas->CGZ = ((int16_t) get.data[10] << 8) | get.data[11];
}

void ICM20948_getComp(magnetometer* magno)
{
//	msg.device_addr = AK09916_DEFAULT_ADDRESS;
//	msg.register_addr = AK09916_MAG_XOUT_L;
//	msg.data = malloc(6);
//
//	msg.register_addr = AK09916_MAG_XOUT_L;
//	msg.length = 6;
//
//	I2C0_read(&msg);
//
//	magno->CMX = (int16_t) ((((int16_t) msg.data[1]) << 8) | msg.data[0]);
//	magno->CMY = (int16_t) ((((int16_t) msg.data[3]) << 8) | msg.data[2]);
//	magno->CMZ = (int16_t) ((((int16_t) msg.data[5]) << 8) | msg.data[4]);
//
//	free(msg.data);
}
// TODO - add magno
void ICM20948_calibrate(volatile measurement* meas, magnetometer* magno, int trials)
{
	printf("\n---------------\nCalibrating values from the sensor...\n");
	
	msg.device_addr = ICM20948_DEFAULT_ADDRESS;
	msg.register_addr = ICM20948_REG_BANK_SEL;
	msg.data = malloc(2);
	msg.length = 1;

	*msg.data = ICM20948_USER_BANK_0;

	I2C0_write(&msg);

	for (int i = 0; i < trials; i++)
	{	
		ICM20948_getMeas(meas);
		ICM20948_getComp(magno);
		
		meas->AXoff += meas->CAX;
		meas->AYoff += meas->CAY;
		meas->AZoff += meas->CAZ;
		meas->GXoff += meas->CGX;
		meas->GYoff += meas->CGY;
		meas->GZoff += meas->CGZ;
		magno->MXoff += magno->CMX;
		magno->MYoff += magno->CMY;
		magno->MZoff += magno->CMZ;
	}
	
	meas->AXoff /= trials;
	meas->AYoff /= trials;
	meas->AZoff /= trials;
	meas->GXoff /= trials;
	meas->GYoff /= trials;
	meas->GZoff /= trials;
	magno->MXoff /= trials;
	magno->MYoff /= trials;
	magno->MZoff /= trials;
	
	printf("Calibration completed:\nAXoffset: %f		GXoffset: %f		MXoffset: %f\nAYoffset: %f		GYoffset: %f		MYoffset: %f\nAZoffset: %f		GZoffset: %f		MZoffset: %f\n", 
		meas->AXoff, meas->GXoff, magno->MXoff,
		meas->AYoff, meas->GYoff, magno->MYoff,
		meas->AZoff, meas->GZoff, magno->MZoff);	
	printf("---------------\n\n");

	msg.length = 1;

	*msg.data = ICM20948_USER_BANK_1;

	I2C0_write(&msg);

	msg.register_addr = ICM20948_XA_OFFS_H;
	msg.length = 2;

	printf("Factory calibration values:\n");
	I2C0_read(&msg);
	printf("XA_OFFS: 0x%04X\n", *((uint16_t*) msg.data));

	msg.register_addr = ICM20948_YA_OFFS_H;

	I2C0_read(&msg);
	printf("YA_OFFS: 0x%04X\n", *((uint16_t*) msg.data));

	msg.register_addr = ICM20948_ZA_OFFS_H;

	I2C0_read(&msg);
	printf("ZA_OFFS: 0x%04X\n", *((uint16_t*) msg.data));

	msg.register_addr = ICM20948_XG_OFFS_USR_H;

	I2C0_read(&msg);
	printf("XG_OFFS: 0x%04X\n", *((uint16_t*) msg.data));

	msg.register_addr = ICM20948_YG_OFFS_USR_H;

	I2C0_read(&msg);
	printf("YG_OFFS: 0x%04X\n", *((uint16_t*) msg.data));

	msg.register_addr = ICM20948_ZG_OFFS_USR_H;

	I2C0_read(&msg);
	printf("ZG_OFFS: 0x%04X\n", *((uint16_t*) msg.data));

	msg.register_addr = ICM20948_REG_BANK_SEL;
	msg.length = 1;

	*msg.data = ICM20948_USER_BANK_0;

	I2C0_write(&msg);

	free(msg.data);
}

void ICM20948_SelfTest(volatile measurement* meas)
{
	int gAvg[3] = {};
	int aAvg[3] = {};
	int aSTAvg[3] = {};
	int gSTAvg[3] = {};
	
	ICM20948_setGyroSelfTest();
	ICM20948_setAccelSelfTest();

	msg.device_addr = ICM20948_DEFAULT_ADDRESS;
	msg.register_addr = ICM20948_REG_BANK_SEL;
	msg.data = malloc(1);
	msg.length = 1;

	*msg.data = ICM20948_USER_BANK_0;

	I2C0_write(&msg);

	free(msg.data);

	for (int index = 0; index < 200; index++)
	{
		ICM20948_getMeas(meas);
		aSTAvg[0] += meas->CAX;
		aSTAvg[1] += meas->CAY;
		aSTAvg[2] += meas->CAZ;
		
		gSTAvg[0] += meas->CGX;
		gSTAvg[1] += meas->CGY;
		gSTAvg[2] += meas->CGZ;
	}

	for (int index = 0; index < 3; index++)
	{
		aSTAvg[index] /= 200;
		gSTAvg[index] /= 200;
	}

	ICM20948_clearGyroSelfTest();
	ICM20948_clearAccelSelfTest();

	msg.device_addr = ICM20948_DEFAULT_ADDRESS;
	msg.register_addr = ICM20948_REG_BANK_SEL;
	msg.data = malloc(1);
	msg.length = 1;

	*msg.data = ICM20948_USER_BANK_0;

	I2C0_write(&msg);

	free(msg.data);

	for (int index = 0; index < 200; index++)
	{
		ICM20948_getMeas(meas);
		aAvg[0] += meas->CAX;
		aAvg[1] += meas->CAY;
		aAvg[2] += meas->CAZ;
		
		gAvg[0] += meas->CGX;
		gAvg[1] += meas->CGY;
		gAvg[2] += meas->CGZ;
	}

	for (int index = 0; index < 3; index++)
	{
		aAvg[index] /= 200;
		gAvg[index] /= 200;
	}

	printf("Self test averages:\nGX - 0x%04X, GY - 0x%04X, GZ - 0x%04X\nAX - 0x%04X, AY - 0x%04X, AZ - 0x%04X\n", 
		gSTAvg[0], gSTAvg[1], gSTAvg[2], aSTAvg[0], aSTAvg[1], aSTAvg[2]);
	printf("Measured averages:\nGX - 0x%04X, GY - 0x%04X, GZ - 0x%04X\nAX - 0x%04X, AY - 0x%04X, AZ - 0x%04X\n", 
		gAvg[0], gAvg[1], gAvg[2], aAvg[0], aAvg[1], aAvg[2]);
}

void ICM20948_setGyroSelfTest()
{
	msg.device_addr = ICM20948_DEFAULT_ADDRESS;
	msg.register_addr = ICM20948_REG_BANK_SEL;
	msg.data = malloc(1);
	msg.length = 1;

	*msg.data = ICM20948_USER_BANK_2;

	I2C0_write(&msg);

	msg.register_addr = ICM20948_GYRO_CONFIG_2;
	
	I2C0_read(&msg);
	*msg.data |= ICM20948_GYRO_CONFIG_2_GX_ST_EN_REG_MASK | ICM20948_GYRO_CONFIG_2_GY_ST_EN_REG_MASK 
					| ICM20948_GYRO_CONFIG_2_GZ_ST_EN_REG_MASK;
	I2C0_write(&msg);

	free(msg.data);
}

void ICM20948_setAccelSelfTest()
{
	msg.device_addr = ICM20948_DEFAULT_ADDRESS;
	msg.register_addr = ICM20948_REG_BANK_SEL;
	msg.data = malloc(1);
	msg.length = 1;

	*msg.data = ICM20948_USER_BANK_2;

	I2C0_write(&msg);

	msg.register_addr = ICM20948_ACCEL_CONFIG_2;
	
	I2C0_read(&msg);
	*msg.data |= ICM20948_ACCEL_CONFIG_2_AX_ST_EN_REG_MASK | ICM20948_ACCEL_CONFIG_2_AY_ST_EN_REG_MASK 
					| ICM20948_ACCEL_CONFIG_2_AZ_ST_EN_REG_MASK;
	I2C0_write(&msg);

	free(msg.data);
}

void ICM20948_clearGyroSelfTest()
{
	msg.device_addr = ICM20948_DEFAULT_ADDRESS;
	msg.register_addr = ICM20948_REG_BANK_SEL;
	msg.data = malloc(1);
	msg.length = 1;

	*msg.data = ICM20948_USER_BANK_2;

	I2C0_write(&msg);

	msg.register_addr = ICM20948_GYRO_CONFIG_2;
	
	I2C0_read(&msg);
	*msg.data &= ~(ICM20948_GYRO_CONFIG_2_GX_ST_EN_REG_MASK & ICM20948_GYRO_CONFIG_2_GY_ST_EN_REG_MASK 
						& ICM20948_GYRO_CONFIG_2_GZ_ST_EN_REG_MASK);
	I2C0_write(&msg);

	free(msg.data);
}

void ICM20948_clearAccelSelfTest()
{
	msg.device_addr = ICM20948_DEFAULT_ADDRESS;
	msg.register_addr = ICM20948_REG_BANK_SEL;
	msg.data = malloc(1);
	msg.length = 1;

	*msg.data = ICM20948_USER_BANK_2;

	I2C0_write(&msg);

	msg.register_addr = ICM20948_ACCEL_CONFIG_2;
	
	I2C0_read(&msg);
	*msg.data &= ~(ICM20948_ACCEL_CONFIG_2_AX_ST_EN_REG_MASK & ICM20948_ACCEL_CONFIG_2_AY_ST_EN_REG_MASK 
						& ICM20948_ACCEL_CONFIG_2_AZ_ST_EN_REG_MASK);
	I2C0_write(&msg);

	free(msg.data);
}

void ICM20948_setFullScaleGyroRange(uint8_t range)
{
	msg.device_addr = ICM20948_DEFAULT_ADDRESS;
	msg.register_addr = ICM20948_REG_BANK_SEL;
	msg.data = malloc(1);
	msg.length = 1;

	*msg.data = ICM20948_USER_BANK_2;

	I2C0_write(&msg);

	msg.register_addr = ICM20948_GYRO_CONFIG_2;
	
	I2C0_read(&msg);
	
	switch (range)
	{
		case ICM20948_GYRO_CONFIG_1_FS_250_DPS:
		{
			*msg.data &= ~(ICM20948_GYRO_CONFIG_1_FS_250_DPS << 1);
			break;
		}
		case ICM20948_GYRO_CONFIG_1_FS_500_DPS:
		{
			*msg.data &= ~(ICM20948_GYRO_CONFIG_1_FS_1000_DPS << 1);
			*msg.data |= ICM20948_GYRO_CONFIG_1_FS_500_DPS << 1;
			break;
		}
		case ICM20948_GYRO_CONFIG_1_FS_1000_DPS:
		{
			*msg.data &= ~(ICM20948_GYRO_CONFIG_1_FS_500_DPS << 1);
			*msg.data |= ICM20948_GYRO_CONFIG_1_FS_500_DPS << 1;
			break;
		}
		case ICM20948_GYRO_CONFIG_1_FS_2000_DPS:
		{
			*msg.data |= ICM20948_GYRO_CONFIG_1_FS_2000_DPS << 1;
			break;
		}
	}
	
	I2C0_write(&msg);

	free(msg.data);
}

void ICM20948_setFullScaleAccelRange(uint8_t range)
{
	msg.device_addr = ICM20948_DEFAULT_ADDRESS;
	msg.register_addr = ICM20948_REG_BANK_SEL;
	msg.data = malloc(1);
	msg.length = 1;

	*msg.data = ICM20948_USER_BANK_2;

	I2C0_write(&msg);

	msg.register_addr = ICM20948_ACCEL_CONFIG_2;
	
	I2C0_read(&msg);

	switch (range)
	{
		case ICM20948_ACCEL_CONFIG_1_FS_2G:
		{
			*msg.data &= ~(ICM20948_ACCEL_CONFIG_1_FS_2G << 1); 
			break;
		}
		case ICM20948_ACCEL_CONFIG_1_FS_4G:
		{
			*msg.data &= ~(ICM20948_ACCEL_CONFIG_1_FS_8G << 1);
			*msg.data |= ICM20948_ACCEL_CONFIG_1_FS_4G << 1;
			break;
		}
		case ICM20948_ACCEL_CONFIG_1_FS_8G:
		{
			*msg.data &= ~(ICM20948_ACCEL_CONFIG_1_FS_4G << 1);
			*msg.data |= ICM20948_ACCEL_CONFIG_1_FS_8G << 1;
			break;
		}
		case ICM20948_ACCEL_CONFIG_1_FS_16G:
		{
			*msg.data |= ICM20948_ACCEL_CONFIG_1_FS_16G << 1;
			break;
		}
	}
	
	I2C0_write(&msg);

	free(msg.data);
}

uint8_t ICM20948_getFullScaleAccelRange()
{
	msg.device_addr = ICM20948_DEFAULT_ADDRESS;
	uint8_t range;
	
	msg.register_addr = ICM20948_REG_BANK_SEL;
	msg.data = malloc(1);
	msg.length = 1;

	*msg.data = ICM20948_USER_BANK_2;

	I2C0_write(&msg);

	msg.register_addr = ICM20948_ACCEL_CONFIG_2;
	
	I2C0_read(&msg);
	range = *msg.data;
	free(msg.data);

	return range; 
}

void ICM20948_ConfigureAccelerometerLowPassFilter()
{
	msg.device_addr = ICM20948_DEFAULT_ADDRESS;
	msg.register_addr = ICM20948_REG_BANK_SEL;
	msg.data = malloc(1);
	msg.length = 1;

	*msg.data = ICM20948_USER_BANK_2;

	I2C0_write(&msg);

	msg.register_addr = ICM20948_ACCEL_CONFIG_2;
	
	I2C0_read(&msg);
	*msg.data |= ICM20948_ACCEL_CONFIG_1_FCHOICE_MASK;
	I2C0_write(&msg);

	free(msg.data);	
}

void ICM20948_setMagContinuousMeasurementMode()
{
	msg.device_addr = AK09916_DEFAULT_ADDRESS;
	msg.register_addr = AK09916_CONTROL_2,
	msg.data = malloc(1);
	msg.length = 1;
	
	I2C0_read(&msg);
	*msg.data &= ~(AK09916_CONTROL_2_MODE_MASK);
	*msg.data |= AK09916_CONTROL_2_CONT_MEAS_MODE_4;
	I2C0_write(&msg);

	free(msg.data);
}

void ICM20948_setMagPowerDown()
{
	msg.device_addr = AK09916_DEFAULT_ADDRESS;
	msg.register_addr = AK09916_CONTROL_2,
	msg.data = malloc(1);
	msg.length = 1;
	
	I2C0_read(&msg);
	*msg.data &= ~(AK09916_CONTROL_2_MODE_MASK);
	I2C0_write(&msg);

	free(msg.data);
}

void ICM20948_setSleepDisabled()
{
	msg.device_addr = ICM20948_DEFAULT_ADDRESS;
	msg.register_addr = ICM20948_REG_BANK_SEL;
	msg.data = malloc(1);
	msg.length = 1;

	*msg.data = ICM20948_USER_BANK_0;

	I2C0_write(&msg);

	msg.register_addr = ICM20948_PWR_MGMT_1,
	msg.length = 1;
	
	I2C0_read(&msg);
	*msg.data &= ~(ICM20948_PWR_MGMT_1_SLEEP_MASK);
	I2C0_write(&msg);

	free(msg.data);
}

void ICM20948_init(volatile measurement* meas, magnetometer* magno)
{
	get.data = malloc(12);
	ICM20948_setSleepDisabled();
	ICM20948_setFullScaleGyroRange(ICM20948_GYRO_CONFIG_1_FS_500_DPS);
	ICM20948_setFullScaleAccelRange(ICM20948_ACCEL_CONFIG_1_FS_4G);
	ICM20948_ConfigureAccelerometerLowPassFilter();

	// ICM enable interrupt
	// Enable ODR_ALIGN_EN
	// GYRO sample rate div = 10
	// ACCEL sample rate div = 10
	// GYRO enable DLPF
	// ACCEL enable DLPF

	ICM20948_SelfTest(meas);
	ICM20948_setMagContinuousMeasurementMode();
}