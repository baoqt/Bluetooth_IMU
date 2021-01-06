#include "I2C.h"

esp_err_t I2C0_init()
{
	i2c_config_t I2C0;
	I2C0.mode = I2C_MODE_MASTER;
	I2C0.sda_io_num = GPIO_NUM_17;
	I2C0.sda_pullup_en = GPIO_PULLUP_DISABLE;
	I2C0.scl_io_num = GPIO_NUM_16;
	I2C0.scl_pullup_en = GPIO_PULLUP_DISABLE;
	I2C0.master.clk_speed = 400000;
	
	i2c_param_config(I2C_NUM_0, &I2C0);
	esp_err_t err_val = i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);
	
	return err_val;
}

void I2C0_write(I2C_msg_t* msg)
{
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, msg->device_addr << 1, ACK_CHECK_EN);
	i2c_master_write_byte(cmd, msg->register_addr, ACK_CHECK_EN);
	i2c_master_write(cmd, msg->data, msg->length, ACK_CHECK_EN);
	i2c_master_stop(cmd);
	i2c_master_cmd_begin(I2C_NUM_0, cmd, 0x0);
	
	i2c_cmd_link_delete(cmd);
}

void I2C0_read(I2C_msg_t* msg)
{
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();

	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (msg->device_addr << 1), ACK_CHECK_EN);
	i2c_master_write_byte(cmd, msg->register_addr, ACK_CHECK_EN);
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (msg->device_addr << 1) | 1, ACK_CHECK_EN);
	if (msg->length > 1)
	{
		i2c_master_read(cmd, msg->data, msg->length - 1, ACK_VAL);
	}
	i2c_master_read_byte(cmd, msg->data + msg->length - 1, NACK_VAL);
	i2c_master_stop(cmd);
	i2c_master_cmd_begin(I2C_NUM_0, cmd, 0x0);

	i2c_cmd_link_delete(cmd);
}