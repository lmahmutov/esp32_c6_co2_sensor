#ifndef ZB_ZCL_CARBON_DIOXIDE_MEAS_H
#define ZB_ZCL_CARBON_DIOXIDE_MEAS_H 1

#include "zcl/zb_zcl_common.h"
#include "zcl/zb_zcl_commands.h"

#define ZB_ZCL_CLUSTER_ID_CARBON_DIOXIDE_MEASUREMENT 0x040dU

enum zb_zcl_carbon_dioxide_measurement_attr_e
{
  ZB_ZCL_ATTR_CARBON_DIOXIDE_MEASUREMENT_VALUE_ID = 0x0000,
};

#define ZB_ZCL_CARBON_DIOXIDE_MEASUREMENT_CLUSTER_REVISION_DEFAULT ((zb_uint16_t)0x0003u)

#define ZB_ZCL_ATTR_CARBON_DIOXIDE_MEASUREMENT_VALUE_UNKNOWN                  ((zb_uint32_t)0x8000)

#define ZB_SET_ATTR_DESCR_WITH_ZB_ZCL_ATTR_CARBON_DIOXIDE_MEASUREMENT_VALUE_ID(data_ptr) \
{                                                               \
  ZB_ZCL_ATTR_CARBON_DIOXIDE_MEASUREMENT_VALUE_ID,              \
  ZB_ZCL_ATTR_TYPE_SINGLE,                                      \
  ZB_ZCL_ATTR_ACCESS_READ_ONLY | ZB_ZCL_ATTR_ACCESS_REPORTING,  \
  (ZB_ZCL_NON_MANUFACTURER_SPECIFIC),                           \
  (void*) data_ptr                                              \
}

#define ZB_ZCL_CARBON_DIOXIDE_MEASUREMENT_REPORT_ATTR_COUNT 1

void zb_zcl_carbon_dioxide_measurement_write_attr_hook(
  zb_uint8_t endpoint, zb_uint16_t attr_id, zb_uint8_t *new_value);

#define ZB_ZCL_DECLARE_CARBON_DIOXIDE_MEASUREMENT_ATTRIB_LIST(attr_list,                          \
    value)                                                       \
  ZB_ZCL_START_DECLARE_ATTRIB_LIST_CLUSTER_REVISION(attr_list, ZB_ZCL_CARBON_DIOXIDE_MEASUREMENT) \
  ZB_ZCL_SET_ATTR_DESC(ZB_ZCL_ATTR_CARBON_DIOXIDE_MEASUREMENT_VALUE_ID, (value))                  \
  ZB_ZCL_FINISH_DECLARE_ATTRIB_LIST                                                               \

void zb_zcl_carbon_dioxide_measurement_init_server(void);
void zb_zcl_carbon_dioxide_measurement_init_client(void);
#define ZB_ZCL_CLUSTER_ID_CARBON_DIOXIDE_MEASUREMENT_SERVER_ROLE_INIT zb_zcl_carbon_dioxide_measurement_init_server
#define ZB_ZCL_CLUSTER_ID_CARBON_DIOXIDE_MEASUREMENT_CLIENT_ROLE_INIT zb_zcl_carbon_dioxide_measurement_init_client

#endif /* ZB_ZCL_CARBON_DIOXIDE_MEAS_H */
