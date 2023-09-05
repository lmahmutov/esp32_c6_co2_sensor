#include "esp_zigbee_co2.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "string.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "zcl/esp_zigbee_zcl_common.h"
#include "sensair_s8.h"
#include "driver/i2c.h"
#include "bmx280.h"
#include "ssd1306.h"
#include "zigbee_logo.h"
#include "zigbee_connected.h"
#include "zigbee_disconnected.h"
#include "zigbee_image.h"
#include "iot_button.h"

/*------ Clobal definitions -----------*/
uint16_t CO2_value = 0;
float temp = 0, pres = 0, hum = 0;
bool connected = false;
uint8_t zero_attr_value = 0;

static ssd1306_handle_t ssd1306_dev = NULL;
SemaphoreHandle_t i2c_semaphore = NULL;

static void button_single_click_cb(void *arg,void *usr_data)
{
    ESP_LOGI("Button boot", "Single click");
}

static void button_long_press_cb(void *arg,void *usr_data)
{
	ESP_LOGI("Button boot", "Long press, leave & reset");
	esp_zb_factory_reset();
}

void register_button()
{
	// create gpio button
	button_config_t gpio_btn_cfg = {
		.type = BUTTON_TYPE_GPIO,
		.long_press_time = CONFIG_BUTTON_LONG_PRESS_TIME_MS,
		.short_press_time = CONFIG_BUTTON_SHORT_PRESS_TIME_MS,
		.gpio_button_config = {
			.gpio_num = 9,
			.active_level = 0,
		},
	};

	button_handle_t gpio_btn = iot_button_create(&gpio_btn_cfg);
	if(NULL == gpio_btn) {
		ESP_LOGE("Button boot", "Button create failed");
	}

	iot_button_register_cb(gpio_btn, BUTTON_SINGLE_CLICK, button_single_click_cb,NULL);
	iot_button_register_cb(gpio_btn, BUTTON_LONG_PRESS_START, button_long_press_cb, NULL);

}

esp_err_t i2c_master_init()
{
	    // Don't initialize twice
    if (i2c_semaphore != NULL)
	{
		return ESP_FAIL;
	}
        
    i2c_semaphore = xSemaphoreCreateMutex();
    if (i2c_semaphore == NULL)
	{
		return ESP_FAIL;
	}
        
	i2c_config_t i2c_config = {
		.mode = I2C_MODE_MASTER,
		.sda_io_num = GPIO_NUM_6,
		.scl_io_num = GPIO_NUM_7,
		.sda_pullup_en = GPIO_PULLUP_ENABLE,
		.scl_pullup_en = GPIO_PULLUP_ENABLE,
		.master.clk_speed = 1000000
	};

	esp_err_t ret;

	ret = i2c_param_config(I2C_NUM_0, &i2c_config);
	if(ret != ESP_OK)
	{
		return ret;
	}
        
	ret = i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);
	if(ret != ESP_OK)
	{
		return ret;
	}
        
    return ESP_OK;
}

#if !defined CONFIG_ZB_ZCZR
#error Define ZB_ZCZR in idf.py menuconfig to compile light (Router) source code.
#endif

static const char *TAG = "ESP_HA_CO2_SENSOR";

/* --------- User task section -----------------*/
void reportAttribute(uint8_t endpoint, uint16_t clusterID, uint16_t attributeID, void *value, uint8_t value_length)
{
    esp_zb_zcl_report_attr_cmd_t cmd = {
        .zcl_basic_cmd = {
            .dst_addr_u.addr_short = 0x0000,
            .dst_endpoint = endpoint,
            .src_endpoint = endpoint,
        },
        .address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
        .clusterID = clusterID,
        .attributeID = attributeID,
        .cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
    };
    esp_zb_zcl_attr_t *value_r = esp_zb_zcl_get_attribute(endpoint, clusterID, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, attributeID);
    memcpy(value_r->data_p, value, value_length);
    esp_zb_zcl_report_attr_cmd_req(&cmd);
}

static void lcd_task(void *pvParameters)
{
	
	/* Start lcd */
	ssd1306_dev = ssd1306_create(I2C_NUM_0, SSD1306_I2C_ADDRESS);
    ssd1306_refresh_gram(ssd1306_dev);
    ssd1306_clear_screen(ssd1306_dev, 0x00);

    ssd1306_draw_bitmap(ssd1306_dev, 0, 16, zigbee, 128, 32);
    ssd1306_refresh_gram(ssd1306_dev);
	vTaskDelay(1500 / portTICK_PERIOD_MS);
	
	while (1)
	{	
		if (CO2_value != 0)
		{
			ESP_LOGI("LCD", "data updating");
			ssd1306_refresh_gram(ssd1306_dev);
			ssd1306_clear_screen(ssd1306_dev, 0x00);
			char co2_data_str[16] = {0};
			char temp_data_str[16] = {0};
			char pres_data_str[16] = {0};
			char hum_data_str[16] = {0};
 		   	sprintf(co2_data_str, "CO2 : %d", CO2_value);
			sprintf(temp_data_str, "Temp: %.2f", temp);
			sprintf(pres_data_str, "Pres: %.1f", pres/100);
			sprintf(hum_data_str, "Hum : %.1f", hum);
	    	ssd1306_draw_string(ssd1306_dev, 5, 0, (const uint8_t *)co2_data_str, 16, 1);
 		   	ssd1306_draw_string(ssd1306_dev, 5, 16, (const uint8_t *)temp_data_str, 16, 1);
	    	ssd1306_draw_string(ssd1306_dev, 5, 32, (const uint8_t *)pres_data_str, 16, 1);
	    	ssd1306_draw_string(ssd1306_dev, 5, 48, (const uint8_t *)hum_data_str, 16, 1);
			ssd1306_draw_bitmap(ssd1306_dev, 112, 0, zigbee_image, 16, 16);
			if (connected)
			{
				ssd1306_draw_bitmap(ssd1306_dev, 112, 18, zigbee_connected, 16, 16);
			}
	    	else
			{
				ssd1306_draw_bitmap(ssd1306_dev, 112, 18, zigbee_disconnected, 16, 16);
			}
			ssd1306_refresh_gram(ssd1306_dev);
		}
		else
		{
			ESP_LOGI("LCD", "Waiting data");
		}		
		vTaskDelay(1000 / portTICK_PERIOD_MS);
	}
}

static void bmx280_task(void *pvParameters)
{
	bmx280_t* bmx280 = bmx280_create(I2C_NUM_0);

    if (!bmx280) { 
        ESP_LOGE("BMX280", "Could not create bmx280 driver.");
        return;
    }

    ESP_ERROR_CHECK(bmx280_init(bmx280));
    bmx280_config_t bmx_cfg = BMX280_DEFAULT_CONFIG;
    ESP_ERROR_CHECK(bmx280_configure(bmx280, &bmx_cfg));

    while (1)
    {
        ESP_ERROR_CHECK(bmx280_setMode(bmx280, BMX280_MODE_FORCE));
        do {
            vTaskDelay(2000 / portTICK_PERIOD_MS);
        } while(bmx280_isSampling(bmx280));
        ESP_ERROR_CHECK(bmx280_readoutFloat(bmx280, &temp, &pres, &hum));
        ESP_LOGI("BMX280", "Read Values: temp = %.2f, pres = %.1f, hum = %.1f", temp, pres/100, hum);
        uint16_t temperature = (uint16_t)(temp * 100);
        uint16_t humidity = (uint16_t)(hum * 100);
        uint16_t pressure = (uint16_t)(pres/100);
        reportAttribute(SENSOR_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT, ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID, &temperature, 2);
        reportAttribute(SENSOR_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT, ESP_ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_VALUE_ID, &humidity, 2);
        reportAttribute(SENSOR_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_PRESSURE_MEASUREMENT, ESP_ZB_ZCL_ATTR_PRESSURE_MEASUREMENT_VALUE_ID, &pressure, 2);
    }
}
/*----------------------------------------*/

static void bdb_start_top_level_commissioning_cb(uint8_t mode_mask)
{
    ESP_ERROR_CHECK(esp_zb_bdb_start_top_level_commissioning(mode_mask));
}

static esp_err_t zb_attribute_handler(const esp_zb_zcl_set_attr_value_message_t *message)
{
    esp_err_t ret = ESP_OK;
    ESP_RETURN_ON_FALSE(message, ESP_FAIL, TAG, "Empty message");
    ESP_RETURN_ON_FALSE(message->info.status == ESP_ZB_ZCL_STATUS_SUCCESS, ESP_ERR_INVALID_ARG, TAG, "Received message: error status(%d)",
                        message->info.status);
    ESP_LOGI(TAG, "Received message: endpoint(%d), cluster(0x%x), attribute(0x%x), data size(%d)", message->info.dst_endpoint, message->info.cluster,
             message->attribute.id, message->attribute.data.size);
    if (message->info.dst_endpoint == SENSOR_ENDPOINT) {
        switch (message->info.cluster) {
        case ESP_ZB_ZCL_CLUSTER_ID_IDENTIFY:
            ESP_LOGI(TAG, "Identify pressed");
            break;
        default:
            ESP_LOGI(TAG, "Message data: cluster(0x%x), attribute(0x%x)  ", message->info.cluster, message->attribute.id);
        }
    }
    return ret;
}

static esp_err_t zb_action_handler(esp_zb_core_action_callback_id_t callback_id, const void *message)
{
    esp_err_t ret = ESP_OK;
    switch (callback_id) {
    case ESP_ZB_CORE_SET_ATTR_VALUE_CB_ID:
        ret = zb_attribute_handler((esp_zb_zcl_set_attr_value_message_t *)message);
        break;
    default:
        ESP_LOGW(TAG, "Receive Zigbee action(0x%x) callback", callback_id);
        break;
    }
    return ret;
}

void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct)
{
    uint32_t *p_sg_p       = signal_struct->p_app_signal;
    esp_err_t err_status = signal_struct->esp_err_status;
    esp_zb_app_signal_type_t sig_type = *p_sg_p;
    esp_zb_zdo_signal_leave_params_t *leave_params = NULL;
    switch (sig_type) {
    case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
    case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
    case ESP_ZB_BDB_SIGNAL_STEERING:
        if (err_status != ESP_OK) {
            connected = false;
            ESP_LOGW(TAG, "Stack %s failure with %s status, steering",esp_zb_zdo_signal_to_string(sig_type), esp_err_to_name(err_status));
            esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_top_level_commissioning_cb, ESP_ZB_BDB_MODE_NETWORK_STEERING, 1000);
        } else {
            /* device auto start successfully and on a formed network */
            connected = true;
            esp_zb_ieee_addr_t extended_pan_id;
            esp_zb_get_extended_pan_id(extended_pan_id);
            ESP_LOGI(TAG, "Joined network successfully (Extended PAN ID: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x, PAN ID: 0x%04hx, Channel:%d)",
                     extended_pan_id[7], extended_pan_id[6], extended_pan_id[5], extended_pan_id[4],
                     extended_pan_id[3], extended_pan_id[2], extended_pan_id[1], extended_pan_id[0],
                     esp_zb_get_pan_id(), esp_zb_get_current_channel());
        }
        break;
    case ESP_ZB_ZDO_SIGNAL_LEAVE:
        leave_params = (esp_zb_zdo_signal_leave_params_t *)esp_zb_app_signal_get_params(p_sg_p);
        if (leave_params->leave_type == ESP_ZB_NWK_LEAVE_TYPE_RESET) {
            ESP_LOGI(TAG, "Reset device");
            esp_zb_factory_reset();
        }
        break;
    default:
        ESP_LOGI(TAG, "ZDO signal: %s (0x%x), status: %s", esp_zb_zdo_signal_to_string(sig_type), sig_type,
                 esp_err_to_name(err_status));
        break;
    }
}

static void esp_zb_task(void *pvParameters)
{
    /* initialize Zigbee stack */
    esp_zb_cfg_t zb_nwk_cfg = ESP_ZB_ZR_CONFIG();
    esp_zb_init(&zb_nwk_cfg);

    /* basic cluster create with fully customized */
    uint32_t ApplicationVersion = 0x0001;
    uint32_t StackVersion = 0x0002;
    uint32_t HWVersion = 0x0002;
    uint8_t ManufacturerName[] = {8, 'L', 'm', 'a', 'h', 'u', 't', 'o', 'v'}; // warning: this is in format {length, 'string'} :
    uint8_t ModelIdentifier[] = {10, 'A', 'i', 'r', '-', 's', 'e', 'n', 's', 'o', 'r'};
    uint8_t DateCode[] = {8, '2', '0', '2', '3', '0', '9', '0', '5'};
    esp_zb_attribute_list_t *esp_zb_basic_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_BASIC);
    esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_APPLICATION_VERSION_ID, &ApplicationVersion);
    esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_STACK_VERSION_ID, &StackVersion);
    esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_HW_VERSION_ID, &HWVersion);
    esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID, ManufacturerName);
    esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID, ModelIdentifier);
    esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_DATE_CODE_ID, DateCode);

    /* identify cluster create with fully customized */

    esp_zb_attribute_list_t *esp_zb_identify_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_IDENTIFY);
    esp_zb_identify_cluster_add_attr(esp_zb_identify_cluster, ESP_ZB_ZCL_CMD_IDENTIFY_IDENTIFY_ID, &zero_attr_value);

    /* Temperature cluster */
    esp_zb_attribute_list_t *esp_zb_temperature_meas_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT);
    esp_zb_temperature_meas_cluster_add_attr(esp_zb_temperature_meas_cluster, ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID, &temp);

    /* Humidity cluster */
    esp_zb_attribute_list_t *esp_zb_humidity_meas_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT);
    esp_zb_humidity_meas_cluster_add_attr(esp_zb_humidity_meas_cluster, ESP_ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_VALUE_ID, &hum);

    /* Presure cluster */
    esp_zb_attribute_list_t *esp_zb_press_meas_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_PRESSURE_MEASUREMENT);
    esp_zb_pressure_meas_cluster_add_attr(esp_zb_press_meas_cluster, ESP_ZB_ZCL_ATTR_PRESSURE_MEASUREMENT_VALUE_ID, &pres);

    /* Carbon Dioxide cluster */
    //esp_zb_attribute_list_t *esp_zb_co2_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_CARBON_DIOXIDE_MEAS);
    //esp_zb_co2_meas_cluster_add_attr(esp_zb_co2_cluster, ESP_ZB_ZCL_ATTR_CO2_MEASUREMENT_VALUE_ID, &test_attr);
    
    /* Create full cluster list enabled on device */
    esp_zb_cluster_list_t *esp_zb_cluster_list = esp_zb_zcl_cluster_list_create();
    esp_zb_cluster_list_add_basic_cluster(esp_zb_cluster_list, esp_zb_basic_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_cluster_list_add_identify_cluster(esp_zb_cluster_list, esp_zb_identify_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_cluster_list_add_temperature_meas_cluster(esp_zb_cluster_list, esp_zb_temperature_meas_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_cluster_list_add_humidity_meas_cluster(esp_zb_cluster_list, esp_zb_humidity_meas_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_cluster_list_add_pressure_meas_cluster(esp_zb_cluster_list, esp_zb_press_meas_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    //esp_zb_cluster_list_add_co2_meas_cluster(esp_zb_cluster_list, esp_zb_co2_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    esp_zb_ep_list_t *esp_zb_ep_list = esp_zb_ep_list_create();
    esp_zb_ep_list_add_ep(esp_zb_ep_list, esp_zb_cluster_list, SENSOR_ENDPOINT, ESP_ZB_AF_HA_PROFILE_ID, ESP_ZB_HA_SIMPLE_SENSOR_DEVICE_ID);

    /* END */
    esp_zb_device_register(esp_zb_ep_list);
    esp_zb_core_action_handler_register(zb_action_handler);
    esp_zb_set_primary_network_channel_set(ESP_ZB_PRIMARY_CHANNEL_MASK);
    ESP_ERROR_CHECK(esp_zb_start(true));
    esp_zb_main_loop_iteration();
}

void app_main(void)
{
	register_button();
    ESP_ERROR_CHECK(i2c_master_init());
	uart_init();
    //sensair_get_info();
    esp_zb_platform_config_t config = {
        .radio_config = ESP_ZB_DEFAULT_RADIO_CONFIG(),
        .host_config = ESP_ZB_DEFAULT_HOST_CONFIG(),
    };
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_zb_platform_config(&config));
	xTaskCreate(lcd_task, "lcd_task", 4096, NULL, 1, NULL);
	xTaskCreate(bmx280_task, "bmx280_task",  4096, NULL, 2, NULL);
	xTaskCreate(sensair_tx_task, "sensor_tx_task", 4096, NULL, 3, NULL);
    xTaskCreate(sensair_rx_task, "sensor_rx_task", 4096, NULL, 4, NULL);
    xTaskCreate(esp_zb_task, "Zigbee_main", 4096, NULL, 5, NULL);
}
