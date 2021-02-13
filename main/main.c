#include "main.h"

void app_main()
{
    printf("\nBootup complete\n\n");
	GPIO_init();
	printf("GPIO intialized\n");
	I2C0_init();
	printf("I2C0 initialized\n");
	ICM20948_init(&meas, &magno);
	printf("ICM20948 initialized\n");
	ICM20948_calibrate(&meas, &magno, 100);
	printf("ICM20948 calibrated\n");
	BL_SPP_init();
	printf("Bluetooth SPP initialized\n");
	TIMG0_T0_init(TIMER_0, true, TIMER_GROUP_0_MEAS_SEC);
	TIMG1_T0_init(TIMER_0, true, TIMER_GROUP_1_WAIT_SEC);
	printf("Timers initialized\n");
	printf("\n");

//	testMsg.device_addr = ICM20948_DEFAULT_ADDRESS;
//	testMsg.register_addr = ICM20948_REG_BANK_SEL;
//	testMsg.data = malloc(1);
//	testMsg.length = 1;
//
//	*testMsg.data = ICM20948_USER_BANK_0;
//
//	I2C0_write(&testMsg);
//
//	testMsg.register_addr = ICM20948_WHO_AM_I;
//	
//	printf("Test read on reg WHO_AM_I (0x00)\nSending:\nDevice addr: 0x%02X, register addr: 0x%02X, data[0]: 0x%02X\n", 
//		testMsg.device_addr, testMsg.register_addr, testMsg.data[0]);
//	
//	I2C0_read(&testMsg);
//	
//	printf("Received:\nDevice addr: 0x%02X, register addr: 0x%02X, data[0]: 0x%02X\n",
//		testMsg.device_addr, testMsg.register_addr, testMsg.data[0]);
//	if (*testMsg.data == 0xEA)
//	{
//		printf("Test read successful\n\n");
//	} 
//	else
//	{
//		printf("Test read failed\n\n");
//	}
//
//	free(testMsg.data);

    xTaskCreate(TG0T0_task, "timer_evt_task", 2048, NULL, 7, NULL);
	xTaskCreate(TG1T0_task, "timer_evt_task", 2048, NULL, 5, NULL);
}

///////////////////////////////////////////////////////////////////
//	Initializes TG0T0
//	Based on an 80MHz clock with a prescaler value between 2 (40MHz) to 65536 (1.22kHz)
///////////////////////////////////////////////////////////////////

void TIMG0_T0_init(int timer_idx, bool auto_reload, double timer_interval_sec)
{
	timer_queue_0 = xQueueCreate(10, sizeof(timer_event_t));
	 /* Select and initialize basic parameters of the timer */
    timer_config_t config = {
        .divider = TIMER_DIVIDER,
        .counter_dir = TIMER_COUNT_UP,
        .counter_en = TIMER_PAUSE,
        .alarm_en = TIMER_ALARM_EN,
        .auto_reload = auto_reload,
    }; // default clock source is APB
    timer_init(TIMER_GROUP_0, timer_idx, &config);

    /* Timer's counter will initially start from value below.
       Also, if auto_reload is set, this value will be automatically reload on alarm */
    timer_set_counter_value(TIMER_GROUP_0, timer_idx, 0x00000000ULL);

    /* Configure the alarm value and the interrupt on alarm. */
    timer_set_alarm_value(TIMER_GROUP_0, timer_idx, timer_interval_sec * TIMER_SCALE);
    timer_enable_intr(TIMER_GROUP_0, timer_idx);
    timer_isr_register(TIMER_GROUP_0, timer_idx, TIMG0_T0_HANDLER, (void *) timer_idx, ESP_INTR_FLAG_IRAM, NULL);
}

void TIMG1_T0_init(int timer_idx, bool auto_reload, double timer_interval_sec)
{
	timer_queue_1 = xQueueCreate(10, sizeof(timer_event_t));
	 /* Select and initialize basic parameters of the timer */
    timer_config_t config = {
        .divider = TIMER_DIVIDER,
        .counter_dir = TIMER_COUNT_UP,
        .counter_en = TIMER_PAUSE,
        .alarm_en = TIMER_ALARM_EN,
        .auto_reload = auto_reload,
    }; // default clock source is APB
    timer_init(TIMER_GROUP_1, timer_idx, &config);

    /* Timer's counter will initially start from value below.
       Also, if auto_reload is set, this value will be automatically reload on alarm */
    timer_set_counter_value(TIMER_GROUP_1, timer_idx, 0x00000000ULL);

    /* Configure the alarm value and the interrupt on alarm. */
    timer_set_alarm_value(TIMER_GROUP_1, timer_idx, timer_interval_sec * TIMER_SCALE);
    timer_enable_intr(TIMER_GROUP_1, timer_idx);
    timer_isr_register(TIMER_GROUP_1, timer_idx, TIMG1_T0_HANDLER, (void *) timer_idx, ESP_INTR_FLAG_IRAM, NULL);
	timer_start(TIMER_GROUP_1, TIMER_0);
}

static void TG0T0_task(void *arg)
{
    while (1) 
	{
        timer_event_t evt;
        xQueueReceive(timer_queue_0, &evt, portMAX_DELAY);
		ICM20948_getMeas(&meas);
		//ICM20948_getComp(&magno);

		//magno.MX = (float) magno.CMX * 0.0000001500f;
		//magno.MY = (float) magno.CMY * 0.0000001500f;
		//magno.MZ = (float) magno.CMZ * 0.0000001500f;

		//magno.MX = ((float) magno.CMX) * magCalibrationX * 4800.0f * 2.0f * 10.0f / 65536.0f - mag_bias[0];
		//magno.MY = ((float) magno.CMY) * magCalibrationY * 4800.0f * 2.0f * 10.0f / 65536.0f - mag_bias[1];
		//magno.MZ = ((float) magno.CMZ) * magCalibrationZ * 4800.0f * 2.0f * 10.0f / 65536.0f - mag_bias[2];
		//meas.time[meas.head] = clock();
		
//		printf("Task 0 - measure:\nHead @ %d\n\n", meas.head);
		meas.AX[meas.head] = ((float) meas.CAX - meas.AXoff) * accel_multiplier/* 0.0011975f */;
		meas.AY[meas.head] = ((float) meas.CAY - meas.AYoff) * accel_multiplier/* 0.0011975f */;
		meas.AZ[meas.head] = ((float) meas.CAZ - meas.AZoff) * accel_multiplier/* 0.0011975f */;

		meas.GX[meas.head] = ((float) meas.CGX - meas.GXoff) * gyro_multiplier/* 0.0152590219f */;
		meas.GY[meas.head] = ((float) meas.CGY - meas.GYoff) * gyro_multiplier/* 0.0152590219f */;
		meas.GZ[meas.head] = ((float) meas.CGZ - meas.GZoff) * gyro_multiplier/* 0.0152590219f */;
		
		meas.head = (meas.head + 1) & (FIFO_DEPTH - 1);
		
		// Values rinted here for diagnostics but can be removed if not needed.
		//char buf1[100];
		//sprintf(buf1, "%f, %f, %f, %f, %f, %f, %ld\n", accel.AX, accel.AY, accel.AZ, gyro.GX, gyro.GY, gyro.GZ, clock());
	}
}

static void TG1T0_task(void *arg)
{
	while (1)
	{
		timer_event_t evt;
		xQueueReceive(timer_queue_1, &evt, portMAX_DELAY);
		
		if (connected)
		{
//			printf("Task 1 - forward:\nTail @ %d\n\n", meas.tail);
			int snap = meas.head;
			while (meas.tail != snap)									// Take items off the FIFO & format message
			{
				forward = true;
//				printf("Task 1A - while loop:\n");
				sprintf(measurement_buffer, ", %f, %f, %f, %f, %f, %f\n", 
					meas.AX[meas.tail], meas.AY[meas.tail], meas.AZ[meas.tail],
					meas.GX[meas.tail], meas.GY[meas.tail], meas.GZ[meas.tail]);
//				printf("Task 1B - while loop:\n");
				strcat(message_buffer, measurement_buffer);
//				printf("Task 1C - while loop:\n");
				meas.tail = (meas.tail + 1) & (FIFO_DEPTH - 1);
			}
			
			if (forward)
			{
//				printf("Task 1 - forward finished:\nTail @ %d\n\n", meas.tail);
				esp_spp_write(HANDLER, strlen(message_buffer), (uint8_t *)message_buffer);
				memcpy(message_buffer, "\0", 1);
				forward = false;
			}
		}
		else															// Blink the LED until connection established
		{
//			printf("Task 1 - blink:\n");
			gpio_set_level(GPIO_NUM_2, 1);
			vTaskDelay(100 / portTICK_PERIOD_MS);
			gpio_set_level(GPIO_NUM_2, 0);
			vTaskDelay(100 / portTICK_PERIOD_MS);
			gpio_set_level(GPIO_NUM_2, 1);
			vTaskDelay(100 / portTICK_PERIOD_MS);
			gpio_set_level(GPIO_NUM_2, 0);
		}
	}
}

void IRAM_ATTR TIMG0_T0_HANDLER(void *para)
{
    timer_spinlock_take(TIMER_GROUP_0);
    int timer_idx = (int) para;

    /* Retrieve the interrupt status and the counter value
       from the timer that reported the interrupt */
    uint32_t timer_intr = timer_group_get_intr_status_in_isr(TIMER_GROUP_0);
    uint64_t timer_counter_value = timer_group_get_counter_value_in_isr(TIMER_GROUP_0, timer_idx);

    /* Prepare basic event data
       that will be then sent back to the main program task */
    timer_event_t evt;
    evt.timer_group = 0;
    evt.timer_idx = timer_idx;
    evt.timer_counter_value = timer_counter_value;

    /* Clear the interrupt
       and update the alarm time for the timer with without reload */
    if (timer_intr & TIMER_INTR_T0) {
        evt.type = true;
        timer_group_clr_intr_status_in_isr(TIMER_GROUP_0, TIMER_0);
    } else {
        evt.type = -1; // not supported even type
    }

    /* After the alarm has been triggered
      we need enable it again, so it is triggered the next time */
    timer_group_enable_alarm_in_isr(TIMER_GROUP_0, timer_idx);

    /* Now just send the event data back to the main program task */
    xQueueSendFromISR(timer_queue_0, &evt, NULL);
    timer_spinlock_give(TIMER_GROUP_0);
}

void IRAM_ATTR TIMG1_T0_HANDLER(void *para)
{
    timer_spinlock_take(TIMER_GROUP_1);
    int timer_idx = (int) para;

    /* Retrieve the interrupt status and the counter value
       from the timer that reported the interrupt */
    uint32_t timer_intr = timer_group_get_intr_status_in_isr(TIMER_GROUP_1);
    uint64_t timer_counter_value = timer_group_get_counter_value_in_isr(TIMER_GROUP_1, timer_idx);

    /* Prepare basic event data
       that will be then sent back to the main program task */
    timer_event_t evt;
    evt.timer_group = 1;
    evt.timer_idx = timer_idx;
    evt.timer_counter_value = timer_counter_value;

    /* Clear the interrupt
       and update the alarm time for the timer with without reload */
    if (timer_intr & TIMER_INTR_T0) {
        evt.type = true;
        timer_group_clr_intr_status_in_isr(TIMER_GROUP_1, TIMER_0);
    } else {
        evt.type = -1; // not supported even type
    }

    /* After the alarm has been triggered
      we need enable it again, so it is triggered the next time */
    timer_group_enable_alarm_in_isr(TIMER_GROUP_1, timer_idx);

    /* Now just send the event data back to the main program task */
    xQueueSendFromISR(timer_queue_1, &evt, NULL);
    timer_spinlock_give(TIMER_GROUP_1);
}

void BL_SPP_init(void)
{
	esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_BLE));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    if ((ret = esp_bt_controller_init(&bt_cfg)) != ESP_OK) {
        ESP_LOGE(SPP_TAG, "%s initialize controller failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT)) != ESP_OK) {
        ESP_LOGE(SPP_TAG, "%s enable controller failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_bluedroid_init()) != ESP_OK) {
        ESP_LOGE(SPP_TAG, "%s initialize bluedroid failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_bluedroid_enable()) != ESP_OK) {
        ESP_LOGE(SPP_TAG, "%s enable bluedroid failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_bt_gap_register_callback(esp_bt_gap_cb)) != ESP_OK) {
        ESP_LOGE(SPP_TAG, "%s gap register failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_spp_register_callback(esp_spp_cb)) != ESP_OK) {
        ESP_LOGE(SPP_TAG, "%s spp register failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_spp_init(esp_spp_mode)) != ESP_OK) {
        ESP_LOGE(SPP_TAG, "%s spp init failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

#if (CONFIG_BT_SSP_ENABLED == true)
    /* Set default parameters for Secure Simple Pairing */
    esp_bt_sp_param_t param_type = ESP_BT_SP_IOCAP_MODE;
    esp_bt_io_cap_t iocap = ESP_BT_IO_CAP_IO;
    esp_bt_gap_set_security_param(param_type, &iocap, sizeof(uint8_t));
#endif

    /*
     * Set default parameters for Legacy Pairing
     * Use variable pin, input pin code when pairing
     */
    esp_bt_pin_type_t pin_type = ESP_BT_PIN_TYPE_VARIABLE;
    esp_bt_pin_code_t pin_code;
    esp_bt_gap_set_pin(pin_type, 0, pin_code);
}

static void esp_spp_cb(esp_spp_cb_event_t event, esp_spp_cb_param_t *param)
{
    switch (event) 
	{
		case ESP_SPP_INIT_EVT:
			ESP_LOGI(SPP_TAG, "ESP_SPP_INIT_EVT");
			esp_bt_dev_set_device_name(DEVICE_NAME);
			esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
			esp_spp_start_srv(sec_mask,role_slave, 0, SPP_SERVER_NAME);
			break;
		case ESP_SPP_DISCOVERY_COMP_EVT:
			ESP_LOGI(SPP_TAG, "ESP_SPP_DISCOVERY_COMP_EVT");
			break;
		case ESP_SPP_OPEN_EVT:
			ESP_LOGI(SPP_TAG, "ESP_SPP_OPEN_EVT");
			break;
		case ESP_SPP_CLOSE_EVT:
			ESP_LOGI(SPP_TAG, "ESP_SPP_CLOSE_EVT"); 
			timer_pause(TIMER_GROUP_0, TIMER_0);
			timer_pause(TIMER_GROUP_1, TIMER_0);
			connected = false;
			timer_set_alarm_value(TIMER_GROUP_1, TIMER_0, TIMER_GROUP_1_WAIT_SEC * TIMER_SCALE);
			timer_start(TIMER_GROUP_1, TIMER_0);
			break;
		case ESP_SPP_START_EVT:
			ESP_LOGI(SPP_TAG, "ESP_SPP_START_EVT");
			break;
		case ESP_SPP_CL_INIT_EVT:
			ESP_LOGI(SPP_TAG, "ESP_SPP_CL_INIT_EVT");
			break;
		case ESP_SPP_DATA_IND_EVT:
			ESP_LOGI(SPP_TAG, "ESP_SPP_DATA_IND_EVT");
			esp_log_buffer_char("",param->data_ind.data,param->data_ind.len);
			break;
		case ESP_SPP_CONG_EVT:
			ESP_LOGI(SPP_TAG, "ESP_SPP_CONG_EVT");
			break;
		case ESP_SPP_WRITE_EVT:
		//	ESP_LOGI(SPP_TAG, "ESP_SPP_WRITE_EVT");
			break;
		case ESP_SPP_SRV_OPEN_EVT:
			ESP_LOGI(SPP_TAG, "ESP_SPP_SRV_OPEN_EVT");
			HANDLER = param->srv_open.handle;
			timer_start(TIMER_GROUP_0, TIMER_0);
			timer_pause(TIMER_GROUP_1, TIMER_0);
			connected = true;
			timer_set_alarm_value(TIMER_GROUP_1, TIMER_0, TIMER_GROUP_1_FIFO_SEC * TIMER_SCALE);
			timer_start(TIMER_GROUP_1, TIMER_0);
			break;
		default:
			break;
	}
}

void esp_bt_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param)
{
    switch (event) 
	{
		case ESP_BT_GAP_AUTH_CMPL_EVT:{
			if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS) 
			{
				ESP_LOGI(SPP_TAG, "authentication success: %s", param->auth_cmpl.device_name);
				esp_log_buffer_hex(SPP_TAG, param->auth_cmpl.bda, ESP_BD_ADDR_LEN);
			} 
			else 
			{
            ESP_LOGE(SPP_TAG, "authentication failed, status:%d", param->auth_cmpl.stat);
			}
			break;
		}
		case ESP_BT_GAP_PIN_REQ_EVT:
		{
			ESP_LOGI(SPP_TAG, "ESP_BT_GAP_PIN_REQ_EVT min_16_digit:%d", param->pin_req.min_16_digit);
			if (param->pin_req.min_16_digit) 
			{
				ESP_LOGI(SPP_TAG, "Input pin code: 0000 0000 0000 0000");
				esp_bt_pin_code_t pin_code = {0};
				esp_bt_gap_pin_reply(param->pin_req.bda, true, 16, pin_code);
			} 
			else 
			{
				ESP_LOGI(SPP_TAG, "Input pin code: 1234");
				esp_bt_pin_code_t pin_code;
				pin_code[0] = '1';
				pin_code[1] = '2';
				pin_code[2] = '3';
				pin_code[3] = '4';
				esp_bt_gap_pin_reply(param->pin_req.bda, true, 4, pin_code);
			}
			break;
		}
		#if (CONFIG_BT_SSP_ENABLED == true)
		case ESP_BT_GAP_CFM_REQ_EVT:
			ESP_LOGI(SPP_TAG, "ESP_BT_GAP_CFM_REQ_EVT Please compare the numeric value: %d", param->cfm_req.num_val);
			esp_bt_gap_ssp_confirm_reply(param->cfm_req.bda, true);
			break;
		case ESP_BT_GAP_KEY_NOTIF_EVT:
			ESP_LOGI(SPP_TAG, "ESP_BT_GAP_KEY_NOTIF_EVT passkey:%d", param->key_notif.passkey);
			break;
		case ESP_BT_GAP_KEY_REQ_EVT:
			ESP_LOGI(SPP_TAG, "ESP_BT_GAP_KEY_REQ_EVT Please enter passkey!");
			break;
		#endif
		default: 
		{
			ESP_LOGI(SPP_TAG, "event: %d", event);
			break;
		}
    }
    return;
}

void GPIO_init()
{
	gpio_config_t io_conf;
	io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
	io_conf.mode = GPIO_MODE_OUTPUT;
	io_conf.pin_bit_mask = 0x00000004;
	io_conf.pull_down_en = 0;
	io_conf.pull_up_en = 0;
	gpio_config(&io_conf);

	gpio_set_level(GPIO_NUM_2, 0);
}