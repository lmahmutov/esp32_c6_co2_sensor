#include "zb_zcl_carbon_dioxide_meas.h"

/* Used endpoint */
#define ENDPOINT_SERVER 1

/* Number of input clusters */
#define CUSTOM_IN_CLUSTER_NUM 3
/* Number of output clusters */
#define CUSTOM_OUT_CLUSTER_NUM 0

#define ZB_CUSTOM_DEVICE_VERSION 6

/* Default channel mask for the device */
#define ZB_CUSTOM_CHANNEL_MASK ZB_TRANSCEIVER_ALL_CHANNELS_MASK 

#define ZB_ZCL_DECLARE_BASIC_ATTRIB_LIST_CUSTOM(                                    \
  attr_list,                                                                        \
  hardware_version,                                                                 \
  manufacturer_name,                                                                \
  model_id,                                                                         \
  power_source,                                                                     \
  sw_build_id)                                                                      \
  zb_bool_t device_enable_##attr_list = ZB_TRUE;                                    \
    ZB_ZCL_START_DECLARE_ATTRIB_LIST_CLUSTER_REVISION(attr_list, ZB_ZCL_BASIC)      \
    ZB_ZCL_SET_ATTR_DESC(ZB_ZCL_ATTR_BASIC_HW_VERSION_ID, (hardware_version))       \
    ZB_ZCL_SET_ATTR_DESC(ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID, (manufacturer_name))\
    ZB_ZCL_SET_ATTR_DESC(ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID, (model_id))         \
    ZB_ZCL_SET_ATTR_DESC(ZB_ZCL_ATTR_BASIC_POWER_SOURCE_ID, (power_source))         \
    ZB_ZCL_SET_ATTR_DESC(ZB_ZCL_ATTR_BASIC_SW_BUILD_ID, (sw_build_id))              \
    ZB_ZCL_FINISH_DECLARE_ATTRIB_LIST

/**
 *  @brief Declare attribute list for Basic cluster (extended attribute set).
 *  @param attr_list [IN] - attribute list name.
 *  @param hardware_version [IN] - pointer to the variable storing hardware version.
 *  @param manufacturer_name [IN] - pointer to the variable storing manufacturer name.
 *  @param model_id [IN] - pointer to the variable storing model identifier.
 *  @param power_source [IN] - pointer to variable storing power source attribute value.
 *  @param sw_build_id [IN] - pointer to the variable storing software version reference.
 */

/*! @brief Declare cluster list for Custom device
    @param cluster_list_name - cluster list variable name
    @param basic_attr_list - attribute list for Basic cluster
    @param identify_attr_list - attribute list for Identify cluster
    @param thermostat_attr_list - attribute list for Thermostat cluster
 */
#define ZB_DECLARE_CUSTOM_CLUSTER_LIST(                                 \
  cluster_list_name,                                                    \
  basic_attr_list,                                                      \
  identify_attr_list,                                                   \
  carbon_dioxide_attr_list)                                             \
  zb_zcl_cluster_desc_t cluster_list_name[] =                           \
  {                                                                     \
    ZB_ZCL_CLUSTER_DESC(                                                \
      ZB_ZCL_CLUSTER_ID_BASIC,                                          \
      ZB_ZCL_ARRAY_SIZE(basic_attr_list, zb_zcl_attr_t),                \
      (basic_attr_list),                                                \
      ZB_ZCL_CLUSTER_SERVER_ROLE,                                       \
      ZB_ZCL_MANUF_CODE_INVALID                                         \
    ),                                                                  \
    ZB_ZCL_CLUSTER_DESC(                                                \
      ZB_ZCL_CLUSTER_ID_IDENTIFY,                                       \
      ZB_ZCL_ARRAY_SIZE(identify_attr_list, zb_zcl_attr_t),             \
      (identify_attr_list),                                             \
      ZB_ZCL_CLUSTER_SERVER_ROLE,                                       \
      ZB_ZCL_MANUF_CODE_INVALID                                         \
    ),                                                                  \
    ZB_ZCL_CLUSTER_DESC(                                                \
      ZB_ZCL_CLUSTER_ID_CARBON_DIOXIDE_MEASUREMENT,                     \
      ZB_ZCL_ARRAY_SIZE(carbon_dioxide_attr_list, zb_zcl_attr_t),       \
      (carbon_dioxide_attr_list),                                       \
      ZB_ZCL_CLUSTER_SERVER_ROLE,                                       \
      ZB_ZCL_MANUF_CODE_INVALID                                         \
    )                                                                   \
  }


/*! @brief Declare simple descriptor for Custom device
    @param ep_name - endpoint variable name
    @param ep_id - endpoint ID
    @param in_clust_num - number of supported input clusters
    @param out_clust_num - number of supported output clusters
 */
#define ZB_DECLARE_CUSTOM_SIMPLE_DESC(                                         \
  ep_name, ep_id, in_clust_num, out_clust_num)                                 \
  ZB_DECLARE_SIMPLE_DESC(in_clust_num, out_clust_num);                         \
  ZB_AF_SIMPLE_DESC_TYPE(in_clust_num, out_clust_num) simple_desc_##ep_name =  \
  {                                                                            \
    ep_id,                                                                     \
    ZB_AF_HA_PROFILE_ID,                                                       \
    ZB_HA_SIMPLE_SENSOR_DEVICE_ID,                                             \
    ZB_CUSTOM_DEVICE_VERSION,                                                  \
    0,                                                                         \
    in_clust_num,                                                              \
    out_clust_num,                                                             \
    {                                                                          \
      ZB_ZCL_CLUSTER_ID_BASIC,                                                 \
      ZB_ZCL_CLUSTER_ID_IDENTIFY,                                              \
      ZB_ZCL_CLUSTER_ID_CARBON_DIOXIDE_MEASUREMENT                             \
    }                                                                          \
  }


/*! @brief Declare endpoint for Custom device
    @param ep_name - endpoint variable name
    @param ep_id - endpoint ID
    @param cluster_list - endpoint cluster list
 */
#define ZB_DECLARE_CUSTOM_EP(ep_name, ep_id, cluster_list)     \
  ZB_DECLARE_CUSTOM_SIMPLE_DESC(                               \
    ep_name,                                                   \
    ep_id,                                                     \
    CUSTOM_IN_CLUSTER_NUM,                                     \
    CUSTOM_OUT_CLUSTER_NUM);                                   \
  ZB_AF_DECLARE_ENDPOINT_DESC(                                 \
    ep_name,                                                   \
    ep_id,                                                     \
    ZB_AF_HA_PROFILE_ID,                                       \
    0,                                                         \
    NULL,                                                      \
    ZB_ZCL_ARRAY_SIZE(cluster_list, zb_zcl_cluster_desc_t),    \
    cluster_list,                                              \
    (zb_af_simple_desc_1_1_t*)&simple_desc_##ep_name,          \
    0, NULL,                                                   \
    0, NULL)

/*! @brief Declare application's device context for Custom device
    @param device_ctx - device context variable
    @param ep_name - endpoint variable name
*/
#define ZB_DECLARE_CUSTOM_CTX(device_ctx, ep_name) \
  ZBOSS_DECLARE_DEVICE_CTX_1_EP(device_ctx, ep_name)
