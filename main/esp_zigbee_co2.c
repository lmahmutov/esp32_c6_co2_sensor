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

/*------ Clobal definitions -----------*/
uint16_t CO2_value = 0;
float temp = 0, pres = 0, hum = 0;

static ssd1306_handle_t ssd1306_dev = NULL;
SemaphoreHandle_t i2c_semaphore = NULL;

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
static void lcd_task(void *pvParameters)
{
	
	/* Start lcd */
	ssd1306_dev = ssd1306_create(I2C_NUM_0, SSD1306_I2C_ADDRESS);
    ssd1306_refresh_gram(ssd1306_dev);
    ssd1306_clear_screen(ssd1306_dev, 0x00);

    char data_str[15] = {0};
    sprintf(data_str, "ZigBee Sensor");
    ssd1306_draw_string(ssd1306_dev, 5, 16, (const uint8_t *)data_str, 16, 1);
    ssd1306_refresh_gram(ssd1306_dev);
	vTaskDelay(1000 / portTICK_PERIOD_MS);
	ssd1306_refresh_gram(ssd1306_dev);
    ssd1306_clear_screen(ssd1306_dev, 0x00);
	
	while (1)
	{	
		ESP_LOGI("LCD", "data updating");
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
    	ssd1306_refresh_gram(ssd1306_dev);
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
        ESP_LOGI("BMX280", "Read Values: temp = %f, pres = %f, hum = %f", temp, pres/100, hum);
    }
}
/*----------------------------------------*/

static void bdb_start_top_level_commissioning_cb(uint8_t mode_mask)
{
    ESP_ERROR_CHECK(esp_zb_bdb_start_top_level_commissioning(mode_mask));
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
            ESP_LOGW(TAG, "Stack %s failure with %s status, steering",esp_zb_zdo_signal_to_string(sig_type), esp_err_to_name(err_status));
            esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_top_level_commissioning_cb, ESP_ZB_BDB_MODE_NETWORK_STEERING, 1000);
        } else {
            /* device auto start successfully and on a formed network */
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

    uint8_t test_attr;
    test_attr = 0;
    /* basic cluster create with fully customized */
    esp_zb_attribute_list_t *esp_zb_basic_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_BASIC);
    esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_ZCL_VERSION_ID, &test_attr);
    esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_POWER_SOURCE_ID, &test_attr);
    esp_zb_cluster_update_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_ZCL_VERSION_ID, &test_attr);
    /* identify cluster create with fully customized */
    esp_zb_attribute_list_t *esp_zb_identify_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_IDENTIFY);
    esp_zb_identify_cluster_add_attr(esp_zb_identify_cluster, ESP_ZB_ZCL_ATTR_IDENTIFY_IDENTIFY_TIME_ID, &test_attr);

    //esp_zb_attribute_list_t *esp_zb_co2_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_CARBON_DIOXIDE_MEAS);
    //esp_zb_co2_meas_cluster_add_attr(esp_zb_co2_cluster, ESP_ZB_ZCL_ATTR_CO2_MEASUREMENT_VALUE_ID, &test_attr);
    
    esp_zb_cluster_list_t *esp_zb_cluster_list = esp_zb_zcl_cluster_list_create();
    esp_zb_cluster_list_add_basic_cluster(esp_zb_cluster_list, esp_zb_basic_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_cluster_list_add_identify_cluster(esp_zb_cluster_list, esp_zb_identify_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    //esp_zb_cluster_list_add_co2_meas_cluster(esp_zb_cluster_list, esp_zb_co2_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    esp_zb_ep_list_t *esp_zb_ep_list = esp_zb_ep_list_create();
    esp_zb_ep_list_add_ep(esp_zb_ep_list, esp_zb_cluster_list, CO2_ENDPOINT, ESP_ZB_AF_HA_PROFILE_ID, ESP_ZB_HA_SIMPLE_SENSOR_DEVICE_ID);

    /* END */
    esp_zb_device_register(esp_zb_ep_list);
    esp_zb_set_primary_network_channel_set(ESP_ZB_PRIMARY_CHANNEL_MASK);
    ESP_ERROR_CHECK(esp_zb_start(true));
    esp_zb_main_loop_iteration();
}

void app_main(void)
{
    ESP_ERROR_CHECK(i2c_master_init());
    uart_init();
    sensair_get_info();
    esp_zb_platform_config_t config = {
        .radio_config = ESP_ZB_DEFAULT_RADIO_CONFIG(),
        .host_config = ESP_ZB_DEFAULT_HOST_CONFIG(),
    };
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_zb_platform_config(&config));
    xTaskCreate(sensair_rx_task, "sensor_rx_task", 2048, NULL, 4, NULL);
    xTaskCreate(sensair_tx_task, "sensor_tx_task", 2048, NULL, 3, NULL);
    xTaskCreate(bmx280_task, "bmx280_task",  4096, NULL, 2, NULL);
	xTaskCreate(lcd_task, "lcd_task", 2048, NULL, 1, NULL);
    xTaskCreate(esp_zb_task, "Zigbee_main", 4096, NULL, 5, NULL);
}
