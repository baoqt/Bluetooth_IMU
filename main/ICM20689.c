#include "ICM20689.h"

#define READ                                            0x80
#define WRITE                                           0x00

spi_bus_config_t buscfg = 
{
    .miso_io_num = 12,
    .mosi_io_num = 13,
    .sclk_io_num = 14,
    .quadwp_io_num=-1,
    .quadhd_io_num=-1,
    .max_transfer_sz = 512
};

spi_device_interface_config_t devcfg =
{
    .command_bits = 0,
    .address_bits = 8,
    .mode = 3,
    .cs_ena_pretrans = 1,
    .cs_ena_posttrans = 1,
	.clock_speed_hz = 1*1000*1000,						// 10MHz CLK
	.spics_io_num = 15,
    .flags = SPI_DEVICE_HALFDUPLEX,
	.queue_size = 12,
};

spi_transaction_t trans =
{
    .flags = SPI_TRANS_USE_TXDATA,
	.addr = ICM20689_WHO_AM_I_REG,
	.length = 8,
	.rxlength = 0,
	.tx_buffer = NULL,
    .rx_buffer = NULL
};

spi_device_handle_t spi;

void ICM20689_getMeas(volatile measurement* meas)
{
    trans.flags = 0;
    trans.addr = ICM20689_ACCEL_XOUT_H_REG | READ;
    trans.length = 48;
    trans.rxlength = 48;
    trans.tx_buffer = NULL;
    trans.rx_buffer = malloc(6);
    spi_device_transmit(spi, &trans);

    meas->CAX = (((uint8_t*) trans.rx_buffer)[0] << 8) | ((uint8_t*) trans.rx_buffer)[1];
    meas->CAY = (((uint8_t*) trans.rx_buffer)[2] << 8) | ((uint8_t*) trans.rx_buffer)[3];
    meas->CAZ = (((uint8_t*) trans.rx_buffer)[4] << 8) | ((uint8_t*) trans.rx_buffer)[5];

    trans.addr = ICM20689_GYRO_XOUT_H_REG | READ;
    spi_device_transmit(spi, &trans);

    meas->CGX = (((uint8_t*) trans.rx_buffer)[0] << 8) | ((uint8_t*) trans.rx_buffer)[1];
    meas->CGY = (((uint8_t*) trans.rx_buffer)[2] << 8) | ((uint8_t*) trans.rx_buffer)[3];
    meas->CGZ = (((uint8_t*) trans.rx_buffer)[4] << 8) | ((uint8_t*) trans.rx_buffer)[5];
    free(trans.rx_buffer);
    trans.rx_buffer = NULL;
}

void ICM20689_calibrate(volatile measurement* meas, int trials)
{
    printf("\n---------------\nCalibrating values from the sensor...\n\n");

	for (int i = 0; i < trials; i++)
	{	
		ICM20689_getMeas(meas);
		
		meas->AXoff += meas->CAX;
		meas->AYoff += meas->CAY;
		meas->AZoff += meas->CAZ;
		meas->GXoff += meas->CGX;
		meas->GYoff += meas->CGY;
		meas->GZoff += meas->CGZ;
	}
	
	meas->AXoff /= trials;
	meas->AYoff /= trials;
	meas->AZoff /= trials;
	meas->GXoff /= trials;
	meas->GYoff /= trials;
	meas->GZoff /= trials;
	
	printf("Calibration completed:\n    AXoffset: 0x%04X		GXoffset: 0x%04X\n    AYoffset: 0x%04X		GYoffset: 0x%04X\n    AZoffset: 0x%04X		GZoffset: 0x%04X\n", 
		(uint16_t) meas->AXoff, (uint16_t) meas->GXoff,
		(uint16_t) meas->AYoff, (uint16_t) meas->GYoff,
		(uint16_t) meas->AZoff, (uint16_t) meas->GZoff);	
	printf("---------------\n\n");

	/* trans.flags = SPI_TRANS_USE_RXDATA;
    trans.addr = ICM20689_XA_OFFSET_H_REG;
	trans.length = 21;

	printf("Factory calibration values:\n");
	spi_device_transmit(spi, &trans);
	printf("XA_OFFS: 0x%04X\n", (((uint16_t) trans.rx_data[0]) << 8) | trans.rx_data[1]);

	msg.register_addr = ICM20689_YA_OFFSET_H_REG;

	spi_device_transmit(spi, &trans);
	printf("YA_OFFS: 0x%04X\n", (((uint16_t) trans.rx_data[0]) << 8) | trans.rx_data[1]);

	msg.register_addr = ICM20689_ZA_OFFSET_H_REG;

	spi_device_transmit(spi, &trans);
	printf("ZA_OFFS: 0x%04X\n", (((uint16_t) trans.rx_data[0]) << 8) | trans.rx_data[1]);

	msg.register_addr = ICM20689_XG_OFFS_USRH_REG;

	spi_device_transmit(spi, &trans);
	printf("XG_OFFS: 0x%04X\n", (((uint16_t) trans.rx_data[0]) << 8) | trans.rx_data[1]);

	msg.register_addr = ICM20689_YG_OFFS_USR_H;

	spi_device_transmit(spi, &trans);
	printf("YG_OFFS: 0x%04X\n", (((uint16_t) trans.rx_data[0]) << 8) | trans.rx_data[1]);

	msg.register_addr = ICM20689_ZG_OFFS_USR_H;

	spi_device_transmit(spi, &trans);
	printf("ZG_OFFS: 0x%04X\n", (((uint16_t) trans.rx_data[0]) << 8) | trans.rx_data[1]); */
}

void ICM20689_SelfTest(volatile measurement* meas)
{
    int gAvg[3] = {};
	int aAvg[3] = {};
	int aSTAvg[3] = {};
	int gSTAvg[3] = {};
	
	ICM20689_setGyroSelfTest();
	ICM20689_setAccelSelfTest();

	for (int index = 0; index < 200; index++)
	{
		ICM20689_getMeas(meas);
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

	ICM20689_clearGyroSelfTest();
	ICM20689_clearAccelSelfTest();

	for (int index = 0; index < 200; index++)
	{
		ICM20689_getMeas(meas);
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

	printf("\n---------------\nSelf test averages:\n    GX - 0x%04X, GY - 0x%04X, GZ - 0x%04X\n    AX - 0x%04X, AY - 0x%04X, AZ - 0x%04X\n\n", 
		(uint16_t) gSTAvg[0], (uint16_t) gSTAvg[1], (uint16_t) gSTAvg[2], (uint16_t) aSTAvg[0], (uint16_t) aSTAvg[1], (uint16_t) aSTAvg[2]);
	printf("Measured averages:\n    GX - 0x%04X, GY - 0x%04X, GZ - 0x%04X\n    AX - 0x%04X, AY - 0x%04X, AZ - 0x%04X\n---------------\n\n", 
		(uint16_t) gAvg[0], (uint16_t) gAvg[1], (uint16_t) gAvg[2], (uint16_t) aAvg[0], (uint16_t) aAvg[1], (uint16_t) aAvg[2]);
}

void ICM20689_setGyroSelfTest()
{
    trans.flags = SPI_TRANS_USE_RXDATA;
    trans.addr = ICM20689_GYRO_CONFIG_REG | READ;
    trans.length = 8;
    trans.rxlength = 8;
    trans.rx_buffer = NULL;
    trans.tx_buffer = NULL;
    spi_device_transmit(spi, &trans);
    trans.tx_data[0] = trans.rx_data[0];

    trans.flags = SPI_TRANS_USE_TXDATA;
    trans.addr = ICM20689_GYRO_CONFIG_REG | WRITE;
    trans.rxlength = 0;
    trans.rx_buffer = NULL;
    trans.tx_buffer = NULL;
    trans.tx_data[0] |= ICM20689_GYRO_CONFIG_XG_ST_MASK 
                    |  ICM20689_GYRO_CONFIG_YG_ST_MASK 
                    |  ICM20689_GYRO_CONFIG_ZG_ST_MASK;
    spi_device_transmit(spi, &trans);
}

void ICM20689_setAccelSelfTest()
{
    trans.flags = SPI_TRANS_USE_RXDATA;
    trans.addr = ICM20689_ACCEL_CONFIG_REG | READ;
    trans.length = 8;
    trans.rxlength = 8;
    trans.rx_buffer = NULL;
    trans.tx_buffer = NULL;
    spi_device_transmit(spi, &trans);
    trans.tx_data[0] = trans.rx_data[0];

    trans.flags = SPI_TRANS_USE_TXDATA;
    trans.addr = ICM20689_ACCEL_CONFIG_REG | WRITE;
    trans.rxlength = 0;
    trans.rx_buffer = NULL;
    trans.tx_buffer = NULL;
    trans.tx_data[0] |= ICM20689_ACCEL_CONFIG_XA_ST_MASK 
                    |  ICM20689_ACCEL_CONFIG_YA_ST_MASK 
                    |  ICM20689_ACCEL_CONFIG_ZA_ST_MASK;
    spi_device_transmit(spi, &trans);
}

void ICM20689_clearGyroSelfTest()
{
    trans.flags = SPI_TRANS_USE_RXDATA;
    trans.addr = ICM20689_GYRO_CONFIG_REG | READ;
    trans.length = 8;
    trans.rxlength = 8;
    trans.rx_buffer = NULL;
    trans.tx_buffer = NULL;
    spi_device_transmit(spi, &trans);
    trans.tx_data[0] = trans.rx_data[0];

    trans.flags = SPI_TRANS_USE_TXDATA;
    trans.addr = ICM20689_GYRO_CONFIG_REG | WRITE;
    trans.rxlength = 0;
    trans.rx_buffer = NULL;
    trans.tx_buffer = NULL;
    trans.tx_data[0] &= ~(ICM20689_GYRO_CONFIG_XG_ST_MASK) 
                    &  ~(ICM20689_GYRO_CONFIG_YG_ST_MASK) 
                    &  ~(ICM20689_GYRO_CONFIG_ZG_ST_MASK);
    spi_device_transmit(spi, &trans);
}

void ICM20689_clearAccelSelfTest()
{
    trans.flags = SPI_TRANS_USE_RXDATA;
    trans.addr = ICM20689_ACCEL_CONFIG_REG | READ;
    trans.length = 8;
    trans.rxlength = 8;
    trans.rx_buffer = NULL;
    trans.tx_buffer = NULL;
    spi_device_transmit(spi, &trans);
    trans.tx_data[0] = trans.rx_data[0];

    trans.flags = SPI_TRANS_USE_TXDATA;
    trans.addr = ICM20689_ACCEL_CONFIG_REG | WRITE;
    trans.rxlength = 0;
    trans.rx_buffer = NULL;
    trans.tx_buffer = NULL;
    trans.tx_data[0] &= ~(ICM20689_ACCEL_CONFIG_XA_ST_MASK) 
                    &  ~(ICM20689_ACCEL_CONFIG_YA_ST_MASK) 
                    &  ~(ICM20689_ACCEL_CONFIG_ZA_ST_MASK);
    spi_device_transmit(spi, &trans);
}

void ICM20689_setFullScaleGyroRange(uint8_t range)
{
    trans.flags = SPI_TRANS_USE_RXDATA;
    trans.addr = ICM20689_GYRO_CONFIG_REG | READ;
    trans.length = 8;
    trans.rxlength = 8;
    trans.rx_buffer = NULL;
    trans.tx_buffer = NULL;
    spi_device_transmit(spi, &trans);
    trans.tx_data[0] = trans.rx_data[0];

    trans.flags = SPI_TRANS_USE_TXDATA;
    trans.addr = ICM20689_GYRO_CONFIG_REG | WRITE;
    trans.rxlength = 0;
    trans.rx_buffer = NULL;
    trans.tx_buffer = NULL;
    trans.tx_data[0] &= ~(ICM20689_GYRO_CONFIG_FS_SEL_MASK);
    trans.tx_data[0] |= range;
    spi_device_transmit(spi, &trans);
}

void ICM20689_setFullScaleAccelRange(uint8_t range)
{
    trans.flags = SPI_TRANS_USE_RXDATA;
    trans.addr = ICM20689_ACCEL_CONFIG_REG | READ;
    trans.length = 8;
    trans.rxlength = 8;
    trans.rx_buffer = NULL;
    trans.tx_buffer = NULL;
    spi_device_transmit(spi, &trans);
    trans.tx_data[0] = trans.rx_data[0];
    
    trans.flags = SPI_TRANS_USE_TXDATA;
    trans.addr = ICM20689_ACCEL_CONFIG_REG | WRITE;
    trans.rxlength = 0;
    trans.rx_buffer = NULL;
    trans.tx_buffer = NULL;
    trans.tx_data[0] &= ~(ICM20689_ACCEL_CONFIG_ACCEL_FS_SEL_MASK);
    trans.tx_data[0] |= range;
    spi_device_transmit(spi, &trans);
}

void ICM20689_setSampleRate(uint16_t rate)
{
    uint8_t div = (1000 / rate) - 1;

    trans.flags = SPI_TRANS_USE_TXDATA;
    trans.addr = ICM20689_SMPLRT_DIV_REG | WRITE;
    trans.length = 8;
    trans.rxlength = 0;
    trans.rx_buffer = NULL;
    trans.tx_buffer = NULL;
    trans.tx_data[0] = div;
    spi_device_transmit(spi, &trans);
}

void ICM20689_toggleSensors(uint8_t state)
{
    trans.flags = SPI_TRANS_USE_RXDATA;
    trans.addr = ICM20689_PWR_MGMT_2_REG | READ;
    trans.length = 8;
    trans.rxlength = 8;
    trans.rx_buffer = NULL;
    trans.tx_buffer = NULL;
    spi_device_transmit(spi, &trans);
    trans.tx_data[0] = trans.rx_data[0];

    trans.flags = SPI_TRANS_USE_TXDATA;
    trans.addr = ICM20689_PWR_MGMT_2_REG | WRITE;
    trans.rxlength = 0;
    trans.rx_buffer = NULL;
    trans.tx_buffer = NULL;

    if (state > 0)                                  // Turn accel and gyro sensors on
    {
        trans.tx_data[0] &= ~(ICM20689_PWR_MGMT_2_SENSOR_STANDBY_MASK);
    }
    else                                            // Turn accel and gyro sensors off
    {
        trans.tx_data[0] |= ICM20689_PWR_MGMT_2_SENSOR_STANDBY_MASK;
    }

    spi_device_transmit(spi, &trans);
}

void ICM20689_toggleFIFO(uint8_t state)
{
    trans.flags = SPI_TRANS_USE_TXDATA;
    trans.addr = ICM20689_USER_CTRL_REG | WRITE;
    trans.length = 8;
    trans.rxlength = 0;
    trans.rx_buffer = NULL;
    trans.tx_buffer = NULL;
    trans.tx_data[0] |= ICM20689_USER_CTRL_FIFO_RST_MASK;
    spi_device_transmit(spi, &trans);

    vTaskDelay(100 / portTICK_PERIOD_MS);

    trans.flags = SPI_TRANS_USE_RXDATA;
    trans.addr = ICM20689_USER_CTRL_REG | READ;
    trans.rxlength = 8;
    trans.rx_buffer = NULL;
    trans.tx_buffer = NULL;
    spi_device_transmit(spi, &trans);
    trans.tx_data[0] = trans.rx_data[0];

    trans.flags = SPI_TRANS_USE_TXDATA;
    trans.addr = ICM20689_USER_CTRL_REG | WRITE;
    trans.rxlength = 0;
    trans.rx_buffer = NULL;
    trans.tx_buffer = NULL;

    if (state > 0)                                  // Turn FIFO on
    {
        trans.tx_data[0] |= ICM20689_USER_CTRL_FIFO_EN_MASK;
    }
    else                                            // Turn FIFO off
    {
        trans.tx_data[0] &= ~(ICM20689_USER_CTRL_FIFO_EN_MASK);
    }

    spi_device_transmit(spi, &trans);

    trans.flags = SPI_TRANS_USE_RXDATA;
    trans.addr = ICM20689_FIFO_EN_REG | READ;
    trans.rxlength = 8;
    trans.rx_buffer = NULL;
    trans.tx_buffer = NULL;
    spi_device_transmit(spi, &trans);
    trans.tx_data[0] = trans.rx_data[0];

    trans.flags = SPI_TRANS_USE_TXDATA;
    trans.addr = ICM20689_FIFO_EN_REG | WRITE;
    trans.rxlength = 0;
    trans.rx_buffer = NULL;
    trans.tx_buffer = NULL;

    if (state > 0)
    {
        trans.tx_data[0] |= ICM20689_FIFO_EN_XG_FIFO_EN_MASK 
                         |  ICM20689_FIFO_EN_YG_FIFO_EN_MASK 
                         |  ICM20689_FIFO_EN_ZG_FIFO_EN_MASK 
                         |  ICM20689_FIFO_EN_ACCEL_FIFO_EN_MASK;
    }
    else
    {
        trans.tx_data[0] &= ~(ICM20689_FIFO_EN_XG_FIFO_EN_MASK)
                         &  ~(ICM20689_FIFO_EN_YG_FIFO_EN_MASK)
                         &  ~(ICM20689_FIFO_EN_ZG_FIFO_EN_MASK)
                         &  ~(ICM20689_FIFO_EN_ACCEL_FIFO_EN_MASK);
    }
    spi_device_transmit(spi, &trans);
}

int ICM20689_readFullFIFO(volatile measurement* meas)
{
    uint16_t fifoCount = 0;
    
    trans.flags = SPI_TRANS_USE_RXDATA;
    trans.addr = ICM20689_FIFO_COUNTH_REG | READ;
    trans.length = 8;
    trans.rxlength = 8;
    trans.tx_buffer = NULL;
    trans.rx_buffer = NULL;

    spi_device_transmit(spi, &trans);

    fifoCount = ((uint16_t) trans.rx_data[0] << 8);

    trans.flags = SPI_TRANS_USE_RXDATA;
    trans.addr = ICM20689_FIFO_COUNTL_REG | READ;
    trans.length = 8;
    trans.rxlength = 8;
    trans.tx_buffer = NULL;
    trans.rx_buffer = NULL;

    spi_device_transmit(spi, &trans);

    fifoCount |= (uint16_t) trans.rx_data[0];

    //printf("FIFO Count: %d\n", fifoCount);

    if (fifoCount > 0)
    {
        trans.flags = 0;
        trans.addr = ICM20689_FIFO_R_W_REG | READ;
        trans.length = fifoCount * 8;
        trans.rxlength = fifoCount * 8;
        trans.tx_buffer = NULL;
        trans.rx_buffer = meas->FIFO;

        spi_device_transmit(spi, &trans);

        /* meas->CAX = (((uint8_t*) trans.rx_buffer)[0] << 8) | ((uint8_t*) trans.rx_buffer)[1];
        meas->CAY = (((uint8_t*) trans.rx_buffer)[2] << 8) | ((uint8_t*) trans.rx_buffer)[3];
        meas->CAZ = (((uint8_t*) trans.rx_buffer)[4] << 8) | ((uint8_t*) trans.rx_buffer)[5];
        meas->CGX = (((uint8_t*) trans.rx_buffer)[6] << 8) | ((uint8_t*) trans.rx_buffer)[7];
        meas->CGY = (((uint8_t*) trans.rx_buffer)[8] << 8) | ((uint8_t*) trans.rx_buffer)[9];
        meas->CGZ = (((uint8_t*) trans.rx_buffer)[10] << 8) | ((uint8_t*) trans.rx_buffer)[11]; */

        trans.rx_buffer = NULL;
    }

    return fifoCount;
}

void ICM20689_ConfigureLowPassFilter()
{
    trans.flags = SPI_TRANS_USE_RXDATA;
    trans.addr = ICM20689_ACCEL_CONFIG_2_REG | READ;
    trans.length = 8;
    trans.rxlength = 8;
    trans.rx_buffer = NULL;
    trans.tx_buffer = NULL;
    spi_device_transmit(spi, &trans);
    trans.tx_data[0] = trans.rx_data[0];

    trans.flags = SPI_TRANS_USE_TXDATA;
    trans.addr = ICM20689_ACCEL_CONFIG_2_REG | WRITE;
    trans.rxlength = 0;
    trans.rx_buffer = NULL;
    trans.tx_buffer = NULL;
    trans.tx_data[0] &= ~(ICM20689_ACCEL_CONFIG_2_ACCEL_FCHOICE_B_MASK) 
                     &  ~(ICM20689_ACCEL_CONFIG_2_A_DLPF_CFG_MASK);
    trans.tx_data[0] |= (ICM20689_ACCEL_CONFIG_2_A_DLPF_CFG_MASK & 0x6);
    spi_device_transmit(spi, &trans);

    trans.flags = SPI_TRANS_USE_RXDATA;
    trans.addr = ICM20689_CONFIG_REG | READ;
    trans.length = 8;
    trans.rxlength = 8;
    trans.rx_buffer = NULL;
    trans.tx_buffer = NULL;
    spi_device_transmit(spi, &trans);
    trans.tx_data[0] = trans.rx_data[0];

    trans.flags = SPI_TRANS_USE_TXDATA;
    trans.addr = ICM20689_CONFIG_REG | WRITE;
    trans.rxlength = 0;
    trans.rx_buffer = NULL;
    trans.tx_buffer = NULL;
    trans.tx_data[0] &= ~(ICM20689_CONFIG_DLPF_CFG_MASK);
    trans.tx_data[0] |= (ICM20689_CONFIG_DLPF_CFG_MASK & 0x6);
    spi_device_transmit(spi, &trans);
}

void ICM20689_setSleepDisabled()
{
    trans.flags = SPI_TRANS_USE_TXDATA;
    trans.addr = ICM20689_PWR_MGMT_1_REG | WRITE;
    trans.rxlength = 0;
    trans.rx_buffer = NULL;
    trans.tx_buffer = NULL;
    trans.tx_data[0] = ICM20689_PWR_MGMT_1_DEVICE_RESET_MASK;
    spi_device_transmit(spi, &trans);

    vTaskDelay(100 / portTICK_PERIOD_MS);

    trans.flags = SPI_TRANS_USE_RXDATA;
    trans.addr = ICM20689_PWR_MGMT_1_REG | READ;
    trans.length = 8;
    trans.rxlength = 8;
    trans.rx_buffer = NULL;
    trans.tx_buffer = NULL;
    spi_device_transmit(spi, &trans);

    trans.tx_data[0] = trans.rx_data[0];

    trans.flags = SPI_TRANS_USE_TXDATA;
    trans.addr = ICM20689_PWR_MGMT_1_REG | WRITE;
    trans.rxlength = 0;
    trans.rx_buffer = NULL;
    trans.tx_buffer = NULL;
    trans.tx_data[0] &= ~(ICM20689_PWR_MGMT_1_CLKSEL_MASK) & ~(ICM20689_PWR_MGMT_1_SLEEP_MASK);
    trans.tx_data[0] |= ICM20689_PWR_MGMT_1_TEMP_DIS_MASK | (ICM20689_PWR_MGMT_1_CLKSEL_MASK & 0x1);
    spi_device_transmit(spi, &trans);

/*     trans.flags = SPI_TRANS_USE_RXDATA;
    trans.addr = ICM20689_LP_MODE_CFG_REG | READ;
    trans.length = 8;
    trans.rxlength = 8;
    trans.rx_buffer = NULL;
    trans.tx_buffer = NULL;
    spi_device_transmit(spi, &trans);

    trans.flags = SPI_TRANS_USE_TXDATA;
    trans.addr = ICM20689_LP_MODE_CFG_REG | WRITE;
    trans.rxlength = 0;
    trans.rx_buffer = NULL;
    trans.tx_buffer = NULL;
    trans.tx_data[0] |= (ICM20689_LP_MODE_CFG_G_AVGCFG_MASK & (0x4 << 4));
    spi_device_transmit(spi, &trans); */

    ICM20689_whoAmI();
}

void ICM20689_whoAmI()
{
    trans.flags = SPI_TRANS_USE_RXDATA;
    trans.addr = ICM20689_WHO_AM_I_REG | READ;
    trans.length = 8;
    trans.rxlength = 8;
    trans.rx_buffer = NULL;
    trans.tx_buffer = NULL;
    spi_device_transmit(spi, &trans);
    printf("\nWHO_AM_I: 0x%02X\n\n", trans.rx_data[0]);
}

void ICM20689_init(volatile measurement* meas)
{
    meas->FIFO = malloc(512);

    spi_bus_initialize(HSPI_HOST, &buscfg, 1);
	spi_bus_add_device(HSPI_HOST, &devcfg, &spi);

    ICM20689_setSleepDisabled();
    ICM20689_setSampleRate(100);

    ICM20689_setFullScaleAccelRange(ICM20689_ACCEL_CONFIG_ACCEL_FS_SEL_MASK & (0x01 << 3));
    ICM20689_setFullScaleGyroRange(ICM20689_GYRO_CONFIG_FS_SEL_MASK & (0x01 << 3));
    ICM20689_ConfigureLowPassFilter();

    ICM20689_SelfTest(meas);
    ICM20689_calibrate(meas, 100);
    
    ICM20689_toggleSensors(0);
}