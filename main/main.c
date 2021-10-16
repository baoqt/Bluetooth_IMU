#include "main.h"

void app_main()
{
	//vTaskDelay(5000 / portTICK_PERIOD_MS);
    printf("\nBootup complete\n\n");
	GPIO_init();
	printf("GPIO intialized\n");
	ICM20689_init(&meas);
	printf("ICM20689 initialized\n");
	BL_SPP_init();
	printf("Bluetooth SPP initialized\n");
	TIMG0_T0_init(TIMER_0, true, TIMER_GROUP_0_MEAS_SEC);
	TIMG1_T0_init(TIMER_0, true, TIMER_GROUP_1_WAIT_SEC);
	printf("Timers initialized\n");
	printf("\n");

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
		uint16_t fifoCount = ICM20689_readFullFIFO(&meas);

		if (fifoCount > 0)
		{
			/* if (j == 0)
			{
				AXavg = 0;
				AYavg = 0;
				AZavg = 0;
				GXavg = 0;
				GYavg = 0;
				GZavg = 0;
			} */

			for (int i = 0; i < fifoCount; i += 12)
			{
				
				/* AXavg += (((float) ((int16_t) ((meas.FIFO[i + 0] << 8) | meas.FIFO[i + 1]))) - meas.AXoff) * accel_multiplier;
				AYavg += (((float) ((int16_t) ((meas.FIFO[i + 2] << 8) | meas.FIFO[i + 3]))) - meas.AYoff) * accel_multiplier;
				AZavg += (((float) ((int16_t) ((meas.FIFO[i + 4] << 8) | meas.FIFO[i + 5]))) - meas.AZoff) * accel_multiplier;
				GXavg += (((float) ((int16_t) ((meas.FIFO[i + 6] << 8) | meas.FIFO[i + 7]))) - meas.GXoff) * gyro_multiplier;
				GYavg += (((float) ((int16_t) ((meas.FIFO[i + 8] << 8) | meas.FIFO[i + 9]))) - meas.GYoff) * gyro_multiplier;
				GZavg += (((float) ((int16_t) ((meas.FIFO[i + 10] << 8) | meas.FIFO[i + 11]))) - meas.GZoff) * gyro_multiplier;
				
				j++;

				if (j > 3)
				{
					j = 0;

					AXavg /= (4);
					AYavg /= (4);
					AZavg /= (4);
					GXavg /= (4);
					GYavg /= (4);
					GZavg /= (4);

					sprintf(measurement_buffer, ", %f, %f, %f, %f, %f, %f\n", 
					AXavg, AYavg, AZavg, GXavg, GYavg, GZavg);
					strcat(message_buffer, measurement_buffer);

					AXavg = 0;
					AYavg = 0;
					AZavg = 0;
					GXavg = 0;
					GYavg = 0;
					GZavg = 0;
				} */
				sprintf(measurement_buffer, ", %f, %f, %f, %f, %f, %f\n", 
					(((float) (((int16_t) (meas.FIFO[i + 0] << 8) | meas.FIFO[i + 1]))) - meas.AXoff) * accel_multiplier,
					(((float) (((int16_t) (meas.FIFO[i + 2] << 8) | meas.FIFO[i + 3]))) - meas.AYoff) * accel_multiplier,
					(((float) (((int16_t) (meas.FIFO[i + 4] << 8) | meas.FIFO[i + 5]))) - meas.AZoff) * accel_multiplier,
					(((float) (((int16_t) (meas.FIFO[i + 6] << 8) | meas.FIFO[i + 7]))) - meas.GXoff) * gyro_multiplier,
					(((float) (((int16_t) (meas.FIFO[i + 8] << 8) | meas.FIFO[i + 9]))) - meas.GYoff) * gyro_multiplier,
					(((float) (((int16_t) (meas.FIFO[i + 10] << 8) | meas.FIFO[i + 11]))) - meas.GZoff) * gyro_multiplier);

				strcat(message_buffer, measurement_buffer);
			}

			esp_spp_write(HANDLER, strlen(message_buffer), (uint8_t *)message_buffer);
			memcpy(message_buffer, "\0", 1);
		}
	}
}

static void TG1T0_task(void *arg)
{
	while (1)
	{
		timer_event_t evt;
		xQueueReceive(timer_queue_1, &evt, portMAX_DELAY);
		
		gpio_set_level(GPIO_NUM_2, 1);
		vTaskDelay(100 / portTICK_PERIOD_MS);
		gpio_set_level(GPIO_NUM_2, 0);
		vTaskDelay(100 / portTICK_PERIOD_MS);
		gpio_set_level(GPIO_NUM_2, 1);
		vTaskDelay(100 / portTICK_PERIOD_MS);
		gpio_set_level(GPIO_NUM_2, 0);
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
		printf("X\n");
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
			ICM20689_toggleSensors(0);
			ICM20689_toggleFIFO(0);
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
			break;
		case ESP_SPP_SRV_OPEN_EVT:
			ESP_LOGI(SPP_TAG, "ESP_SPP_SRV_OPEN_EVT");
			HANDLER = param->srv_open.handle;
			ICM20689_toggleSensors(1);
			//ICM20689_calibrate(&meas, 300);
			ICM20689_toggleFIFO(1);
			timer_pause(TIMER_GROUP_1, TIMER_0);
			timer_start(TIMER_GROUP_0, TIMER_0);
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