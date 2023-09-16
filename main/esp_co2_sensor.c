#include "string.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "zboss_api.h"
#include "esp_co2_sensor.h"
#include "esp_mac.h"
#include "sensair_s8.h"
#include "zcl/zb_zcl_reporting.h"

#if !defined CONFIG_ZB_ZCZR
#error Define ZB_ZCZR in idf.py menuconfig to compile light (Router) source code.
#endif

static const char *TAG = "ESP_ZB_CO2_ZBOSS";
/* ZBOSS*/
/**
 * Global variables definitions
 */
zb_uint16_t g_dst_addr;
zb_uint8_t g_addr_mode;
zb_uint8_t g_endpoint;
uint16_t CO2_value;

/**
 * Declaring attributes for each cluster
 */

/* Basic cluster attributes */
zb_uint8_t g_attr_hardware_version = ZB_ZCL_BASIC_HW_VERSION_DEFAULT_VALUE;
zb_uint8_t g_attr_manufacturer_name[]= {4, 'L', 'e', 'n', 'z'};
zb_uint8_t g_attr_model_id[] = {3, 'C', 'O', '2'};
zb_uint8_t g_attr_power_source = ZB_ZCL_BASIC_POWER_SOURCE_DC_SOURCE;
zb_uint8_t g_attr_sw_build_id = 0;
ZB_ZCL_DECLARE_BASIC_ATTRIB_LIST_CUSTOM(basic_attr_list, &g_attr_hardware_version,
       &g_attr_manufacturer_name, &g_attr_model_id, &g_attr_power_source, &g_attr_sw_build_id);

/* Identify cluster attributes */
zb_uint16_t g_attr_identify_time = ZB_ZCL_IDENTIFY_IDENTIFY_TIME_DEFAULT_VALUE;
ZB_ZCL_DECLARE_IDENTIFY_ATTRIB_LIST(identify_attr_list, &g_attr_identify_time);

 /* Carbon Dioxide measurement cluster attributes data */
zb_uint16_t co2_attr_measured_value       = ZB_ZCL_ATTR_CARBON_DIOXIDE_MEASUREMENT_VALUE_UNKNOWN;
ZB_ZCL_DECLARE_CARBON_DIOXIDE_MEASUREMENT_ATTRIB_LIST(carbon_dioxide_attr_list, &co2_attr_measured_value);

/* Declare cluster list for the device */
ZB_DECLARE_CUSTOM_CLUSTER_LIST(custom_clusters,
                               basic_attr_list,
                               identify_attr_list,
                               carbon_dioxide_attr_list);

/* Declare endpoint */
ZB_DECLARE_CUSTOM_EP(custom_ep, ENDPOINT_SERVER, custom_clusters);

/* Declare application's device context for single-endpoint device */
ZB_DECLARE_CUSTOM_CTX(custom_ctx, custom_ep);

void zboss_signal_handler(zb_uint8_t param)
{
  zb_zdo_app_signal_hdr_t *sg_p = NULL;
  zb_zdo_app_signal_t sig = zb_get_app_signal(param, &sg_p);

  //ESP_LOGI(TAG, ">> zboss_signal_handler: status %d signal ",(ZB_GET_APP_SIGNAL_STATUS(param));//, sig));

  if (ZB_GET_APP_SIGNAL_STATUS(param) == 0)
  {
    switch(sig)
    {
      case ZB_ZDO_SIGNAL_DEFAULT_START:
      case ZB_BDB_SIGNAL_DEVICE_FIRST_START:
      case ZB_BDB_SIGNAL_DEVICE_REBOOT:
        ESP_LOGI(TAG, "Device STARTED OK, signal = %d", sig);
        break;

      case ZB_ZDO_SIGNAL_LEAVE:
            zb_zdo_signal_leave_params_t *p_leave_params = ZB_ZDO_SIGNAL_GET_PARAMS(sg_p, zb_zdo_signal_leave_params_t);
            ESP_LOGI(TAG, "Network left. Leave type: %d", p_leave_params->leave_type);
            break;
      default:
        ESP_LOGI(TAG, "Unknown signal %hd", sig);
        break;
    }
  }
  else if (sig == ZB_ZDO_SIGNAL_PRODUCTION_CONFIG_READY)
  {
    ESP_LOGI(TAG, "Production config is not present or invalid");
  }
  else
  {
    ESP_LOGI(TAG,
              "Device started FAILED status %d",
              (ZB_GET_APP_SIGNAL_STATUS(param)));
  }

  if (param)
  {
    zb_buf_free(param);
  }

  ESP_LOGI(TAG, "<< zboss_signal_handler");
}

zb_uint8_t zcl_specific_cluster_cmd_handler(zb_uint8_t param)
{
  zb_zcl_parsed_hdr_t cmd_info;
  zb_uint8_t lqi = ZB_MAC_LQI_UNDEFINED;
  zb_int8_t rssi = ZB_MAC_RSSI_UNDEFINED;

  ESP_LOGI("ZCL", "> zcl_specific_cluster_cmd_handler");

  ZB_ZCL_COPY_PARSED_HEADER(param, &cmd_info);

  g_dst_addr = ZB_ZCL_PARSED_HDR_SHORT_DATA(&cmd_info).source.u.short_addr;
  g_endpoint = ZB_ZCL_PARSED_HDR_SHORT_DATA(&cmd_info).src_endpoint;
  g_addr_mode = ZB_APS_ADDR_MODE_16_ENDP_PRESENT;

  ZB_ZCL_DEBUG_DUMP_HEADER(&cmd_info);
  ESP_LOGI("ZCL",  "payload size: %i", param);

  zb_zdo_get_diag_data(g_dst_addr, &lqi, &rssi);
  ESP_LOGI("ZCL",  "lqi %hd rssi %d g_dst_addr %x g_endpoint %x", lqi, rssi, g_dst_addr, g_endpoint);

  if (cmd_info.cmd_direction == ZB_ZCL_FRAME_DIRECTION_TO_CLI)
  {
    ESP_LOGI("ZCL", 
        "Unsupported \"from server\" command direction");
  }

  ESP_LOGI("ZCL",  "< zcl_specific_cluster_cmd_handler");
  return ZB_FALSE;
}

static void update_attr_value(void *pvParameters)
{
  while(1)
  {
    float co2_fixed = (double)CO2_value/1000000;
    ESP_LOGI(TAG,"Set CO2 value. zcl_status: %f", co2_fixed);
    zb_zcl_status_t zcl_status;
    zcl_status = zb_zcl_set_attr_val(ENDPOINT_SERVER, 
                                     ZB_ZCL_CLUSTER_ID_CARBON_DIOXIDE_MEASUREMENT, 
                                     ZB_ZCL_CLUSTER_SERVER_ROLE, 
                                     ZB_ZCL_ATTR_CARBON_DIOXIDE_MEASUREMENT_VALUE_ID, 
                                     (zb_uint8_t *)&co2_fixed, 
                                     ZB_FALSE);
    if(zcl_status != ZB_ZCL_STATUS_SUCCESS)
    {
        ESP_LOGI(TAG,"Set CO2 value fail. zcl_status: %d", zcl_status);
    }
    /*-------------------------------*/
        zb_zcl_reporting_info_t cmd = {
        .ep = ENDPOINT_SERVER,
        .cluster_id = ZB_ZCL_CLUSTER_ID_CARBON_DIOXIDE_MEASUREMENT,
        .cluster_role = ZB_ZCL_CLUSTER_SERVER_ROLE,
        .attr_id = ZB_ZCL_ATTR_CARBON_DIOXIDE_MEASUREMENT_VALUE_ID, 
        .dst.short_addr = 0x0000,
        .dst.endpoint = ENDPOINT_SERVER,
        .dst.profile_id = ZB_AF_HA_PROFILE_ID,
        .manuf_code = ZB_ZCL_MANUFACTURER_WILDCARD_ID,
    };

    if (ZCL_CTX().reporting_ctx.buf_ref != ZB_UNDEFINED_BUFFER)
      {
        ESP_LOGW(TAG, "buffer is free, send report");
        zb_zcl_send_report_attr_command(&cmd, ZCL_CTX().reporting_ctx.buf_ref);;
        ZCL_CTX().reporting_ctx.buf_ref = ZB_UNDEFINED_BUFFER;
      }
    else
      {
        /* Report buffer is in use. Retry sending on cb */
        ESP_LOGW(TAG, "buffer is in use, skip");
      }
    /*--------------------------------------------------------------------*/
    vTaskDelay(pdMS_TO_TICKS(5000));
  }
}

static void esp_zb_task(void *pvParameters)
{
  /* Initiate the stack start with starting the commissioning */
  if (zboss_start() != RET_OK)
  {
    ESP_LOGI(TAG, "zdo_dev_start failed");
  }
  else
  {
    /* Call the main loop */
    zboss_main_loop();
  }

}

void app_main(void)
{
  ESP_ERROR_CHECK(nvs_flash_init());
     /* Trace disable */
  ZB_SET_TRAF_DUMP_OFF();
  /* Global ZBOSS initialization */
  ZB_INIT("Zigbee Co2 Sensor");

  esp_err_t ret = ESP_OK;
  uint8_t base_mac_addr[8];
  ret = esp_efuse_mac_get_default(base_mac_addr);
  if(ret != ESP_OK){
        ESP_LOGE(TAG, "Failed to get base MAC address from EFUSE BLK0. (%s)", esp_err_to_name(ret));
        ESP_LOGE(TAG, "Aborting");
        abort();
    } 
  
  zb_ieee_addr_t g_zr_addr = { base_mac_addr[7], base_mac_addr[6], base_mac_addr[5], base_mac_addr[4], base_mac_addr[3], base_mac_addr[2], base_mac_addr[1], base_mac_addr[0]};

    /* Set up defaults for the commissioning */
  zb_set_long_address(g_zr_addr);
  zb_set_network_router_role(ZB_CUSTOM_CHANNEL_MASK);
  zb_set_nvram_erase_at_start(ZB_FALSE);

  /* Register device ZCL context */
  ZB_AF_REGISTER_DEVICE_CTX(&custom_ctx);

   /* Register cluster commands handler for a specific endpoint */
  ZB_AF_SET_ENDPOINT_HANDLER(ENDPOINT_SERVER, zcl_specific_cluster_cmd_handler);

  /* Sensair S8 init start */
  uart_init();
  sensair_get_info();
  xTaskCreate(update_attr_value, "Update attr value", 4096, NULL, 2, NULL);
  xTaskCreate(sensair_tx_task, "sensor_tx_task", 4096, NULL, 3, NULL);
  xTaskCreate(sensair_rx_task, "sensor_rx_task", 4096, NULL, 4, NULL); 
  xTaskCreate(esp_zb_task, "Zigbee_main", 4096, NULL, 5, NULL);
}


void zb_zcl_carbon_dioxide_measurement_init_client(void)
{
  zb_zcl_add_cluster_handlers(ZB_ZCL_CLUSTER_ID_CARBON_DIOXIDE_MEASUREMENT,
                              ZB_ZCL_CLUSTER_CLIENT_ROLE,
                              (zb_zcl_cluster_check_value_t)NULL,
                              (zb_zcl_cluster_write_attr_hook_t)NULL,
                              (zb_zcl_cluster_handler_t)NULL);
}

zb_ret_t check_value_carbon_dioxide_attr(zb_uint16_t attr_id, zb_uint8_t endpoint, zb_uint8_t *value)
{
  ZVUNUSED(attr_id);
  ZVUNUSED(endpoint);
  ZVUNUSED(value); 
  ESP_LOGI("ZCL", "< check_value_temp_measurement ret");
  return RET_OK;
}

/* For correct workflow of ZCL command processing at least cluster_check_value function should be defined in zb_zcl_add_cluster_handlers*/
void zb_zcl_carbon_dioxide_measurement_init_server(void)
{
  ESP_LOGI(TAG, ">> zb_zcl_carbon_dioxide_measurement_init_server");

  zb_zcl_add_cluster_handlers(ZB_ZCL_CLUSTER_ID_CARBON_DIOXIDE_MEASUREMENT,
                              ZB_ZCL_CLUSTER_SERVER_ROLE,
                              (zb_zcl_cluster_check_value_t)check_value_carbon_dioxide_attr,
                              (zb_zcl_cluster_write_attr_hook_t)NULL,
                              (zb_zcl_cluster_handler_t)NULL);

  ESP_LOGI(TAG,  "<< zb_zcl_carbon_dioxide_measurement_init_server");
}

void zb_zcl_carbon_dioxide_measurement_write_attr_hook(
  zb_uint8_t endpoint, zb_uint16_t attr_id, zb_uint8_t *new_value)
{
	ZVUNUSED(new_value);
  ZVUNUSED(endpoint);

  ESP_LOGI("ZCL", ">> zb_zcl_carbon_dioxide_measurement_write_attr_hook endpoint %hd, attr_id %d",
            endpoint, attr_id);

  if (attr_id == ZB_ZCL_ATTR_CARBON_DIOXIDE_MEASUREMENT_VALUE_ID)
  {
	  /* TODO Change min/max CO2 by current are not agree
	   * spec/
	   * Need consult with customer !*/
  }

  ESP_LOGI("ZCL", "<< zb_zcl_carbon_dioxide_measurement_write_attr_hook");
}