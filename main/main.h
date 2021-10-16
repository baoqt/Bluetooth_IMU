#ifndef MAIN_H
#define MAIN_H

#include <unistd.h>
#include <stdio.h>
#include <stdint.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <math.h>
#include <float.h>
#include <time.h>

#include "nvs.h"
#include "nvs_flash.h"

#ifndef __FREERTOS__
#define __FREERTOS__
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#endif

#include "esp_system.h"
#include "esp_types.h"
#include "esp_intr_alloc.h"
#include "esp_spi_flash.h"
#include "sdkconfig.h"
#include "esp_log.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_bt_device.h"
#include "esp_spp_api.h"

#include "driver/periph_ctrl.h"
#include "driver/timer.h"
#include "driver/gpio.h"
#include "soc/soc.h"
#include "soc/timer_group_struct.h"

//#include "I2C.h"
//#include "ICM20948.h"
#include "ICM20689.h"

///////////////////////////////////////////////////////////////////
//	Bluetooth SPP defines
///////////////////////////////////////////////////////////////////
#define SPP_TAG "Bluetooth_IMU"
#define SPP_SERVER_NAME "SPP_SERVER"
#define DEVICE_NAME "Bluetooth_IMU_A"

///////////////////////////////////////////////////////////////////
//	Timer group 0, timer 0 configuration and control registers
//
//	TIMG0_T0CONFIG_REG			Timer 0 configuration registers
//	TIMG0_T0LOADLO_REG			Timer 0 current value, low 32 bits
//	TIMG0_T0LOADLO_REG			Timer 0 current value, high 32 bits
//	TIMG0_T0UPDATE_REG			Write to copy current timer value into LO/HI registers
//	TIMG0_T0ALARMLO_REG			Timer 0 alarm value, low 32 bits
//	TIMG0_T0ALARMHI_REG			Timer 0 alarm value, high 32 bits
//	TIMG0_T0LOADLO_REG			Timer 0 reload value, low 32 bits
//	TIMG0_T0LOADHI_REG			Timer 0 reload value, high 32 bits
//	TIMG0_T0LOAD_REG			Write to reload timer from LO/HI registers
///////////////////////////////////////////////////////////////////

#define TIMG0_T0CONFIG_REG		(*((volatile uint32_t*) 0x3FF5F000))
#define TIMG0_T0LO_REG			(*((volatile uint32_t*) 0x3FF5F004))
#define TIMG0_T0HI_REG			(*((volatile uint32_t*) 0x3FF5F008))
#define TIMG0_T0UPDATE_REG		(*((volatile uint32_t*) 0x3FF5F00C))
#define TIMG0_T0ALARMLO_REG		(*((volatile uint32_t*) 0x3FF5F010))
#define TIMG0_T0ALARMHI_REG		(*((volatile uint32_t*) 0x3FF5F014))
#define TIMG0_T0LOADLO_REG		(*((volatile uint32_t*) 0x3FF5F018))
#define TIMG0_T0LOADHI_REG		(*((volatile uint32_t*) 0x3FF5F01C))
#define TIMG0_T0LOAD_REG		(*((volatile uint32_t*) 0x3FF5F020))

typedef struct {
    int type;  // the type of timer's event
    int timer_group;
    int timer_idx;
    uint64_t timer_counter_value;
} timer_event_t;

/* I2C_msg_t testMsg = {.device_addr = ICM20948_DEFAULT_ADDRESS,
				 .register_addr = ICM20948_WHO_AM_I,
				 .data = NULL,
				 .length = 0}; */

xQueueHandle timer_queue_0;
xQueueHandle timer_queue_1;

///////////////////////////////////////////////////////////////////
//	Timer group 0, timer 0 interrupt registers
//
//	TIMG0_T0_INT_ENA_REG		Interrupt enable bits
//	TIMG0_T0_INT_RAW_REG		Raw interrupt status
//	TIMG0_T0_INT_ST_REG			Masked interrupt status
//	TIMG0_T0_INT_CLR_REG		Interrupt clear bits
///////////////////////////////////////////////////////////////////

#define TIMG0_T0_INT_ENA_REG	(*((volatile uint32_t*) 0x3FF5F098))
#define TIMG0_T0_INT_RAW_REG	(*((volatile uint32_t*) 0x3FF5F09C))
#define TIMG0_T0_INT_ST_REG		(*((volatile uint32_t*) 0x3FF5F0A0))
#define TIMG0_T0_INT_CLR_REG	(*((volatile uint32_t*) 0x3FF5F0A4))

#define TIMER_DIVIDER           16                                  // Hardware timer clock divider
#define TIMER_SCALE             (TIMER_BASE_CLK / TIMER_DIVIDER)    // convert counter value to seconds
#define TIMER_GROUP_0_MEAS_SEC  (0.33)                              // IMU measurement period
#define TIMER_GROUP_1_WAIT_SEC  (3.0)                               // Connection status LED period

volatile measurement meas = 
{
    .AXoff = 0, .AYoff = 0, .AZoff = 0,
    .GXoff = 0, .GYoff = 0, .GZoff = 0
};

static char measurement_buffer[100] = "\0";
static char message_buffer[3000] = "\0";

/* volatile int j;
volatile float AXavg;
volatile float AYavg;
volatile float AZavg;
volatile float GXavg;
volatile float GYavg;
volatile float GZavg; */

#define MEASUREMENT_BIT_LENGTH  16
#define ACCEL_FULL_SCALE        4
#define GYRO_FULL_SCALE         500
#define GRAVITY_CONSTANT        9.81f

static float accel_multiplier = (ACCEL_FULL_SCALE * GRAVITY_CONSTANT) / pow(2, MEASUREMENT_BIT_LENGTH - 1);
static float gyro_multiplier = GYRO_FULL_SCALE / pow(2, MEASUREMENT_BIT_LENGTH - 1);

void TIMG0_T0_init(int timer_idx, bool auto_reload, double timer_interval_sec);
void TIMG1_T0_init(int timer_idx, bool auto_reload, double timer_interval_sec);
void IRAM_ATTR TIMG0_T0_HANDLER(void *para);
void IRAM_ATTR TIMG1_T0_HANDLER(void *para);
static void TG0T0_task(void *arg);
static void TG1T0_task(void *arg);
void GPIO_init();

static const esp_spp_mode_t esp_spp_mode = ESP_SPP_MODE_CB;
static const esp_spp_sec_t sec_mask = ESP_SPP_SEC_AUTHENTICATE;
static const esp_spp_role_t role_slave = ESP_SPP_ROLE_SLAVE;

void BL_SPP_init(void);
void esp_bt_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param);
static void esp_spp_cb(esp_spp_cb_event_t event, esp_spp_cb_param_t *param);

uint32_t HANDLER;

#endif