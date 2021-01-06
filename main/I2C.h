#ifndef I2C_H
#define I2C_H

#include <stdint.h>
#include "driver/i2c.h"

#define ACK_CHECK_EN		0x01
#define ACK_CHECK_DIS		0x00
#define ACK_VAL 			0x00
#define NACK_VAL 			0x01

typedef struct I2C_msg_t
{
	uint8_t device_addr;
	uint8_t register_addr;
	uint8_t* data;
	uint8_t length;
}I2C_msg_t;

/**
 * @brief		This function sets up I2C0 block to interface with the ICM20948 IMU
 *
 * @return
 *				- ESP_OK: success
 *				- other: failed
 */
esp_err_t I2C0_init();

/**
 * @brief		This function formats and feeds a read message to the I2C block to be transmitted 
 * 
 * @param[in]	handle: The message handle
 */
void I2C0_read(struct I2C_msg_t* handle);

/**
 * @brief		This function formats and feeds a write message to the I2C block to be transmitted 
 * 
 * @param[in]	handle: The message handle
 */
void I2C0_write(struct I2C_msg_t* handle);

#endif