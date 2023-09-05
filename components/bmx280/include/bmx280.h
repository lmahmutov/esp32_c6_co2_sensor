/**
 * BMX280 - BME280 & BMP280 Driver for Esspressif ESP-32.
 *
 * MIT License
 *
 * Copyright (C) 2020 Halit Utku Maden
 * Please contact at <utkumaden@hotmail.com>
 */

#ifndef _BMX280_H_
#define _BMX280_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <limits.h>
#include "driver/i2c.h"
#include "sdkconfig.h"

#define BMXAPI extern
extern SemaphoreHandle_t i2c_semaphore;

/**
 * Anonymous structure to driver settings.
 */
typedef struct bmx280_t bmx280_t;

#include "bmx280_bits.h"

/**
 * Create an instance of the BMX280 driver.
 * @param port The I2C port to use.
 * @return A non-null pointer to the driver structure on success.
 */
BMXAPI bmx280_t* bmx280_create(i2c_port_t port);
/**
 * Destroy your the instance.
 * @param bmx280 The instance to destroy.
 */
BMXAPI void bmx280_close(bmx280_t* bmx280);

/**
 * Probe for the sensor and read calibration data.
 * @param bmx280 Driver structure.
 */
BMXAPI esp_err_t bmx280_init(bmx280_t* bmx280);
/**
 * Configure the sensor with the given parameters.
 * @param bmx280 Driver structure.
 * @param configuration The configuration to use.
 */
BMXAPI esp_err_t bmx280_configure(bmx280_t* bmx280, bmx280_config_t *cfg);

/**
 * Set the sensor mode of operation.
 * @param bmx280 Driver structure.
 * @param mode The mode to set the sensor to.
 */
BMXAPI esp_err_t bmx280_setMode(bmx280_t* bmx280, bmx280_mode_t mode);
/**
 * Get the sensor current mode of operation.
 * @param bmx280 Driver structure.
 * @param mode Pointer to write current mode to.
 */
BMXAPI esp_err_t bmx280_getMode(bmx280_t* bmx280, bmx280_mode_t* mode);

/**
 * Returns true if sensor is currently sampling environment conditions.
 * @param bmx280 Driver structure.
 */
BMXAPI bool bmx280_isSampling(bmx280_t* bmx280);

/**
 * Read sensor values as fixed point numbers.
 * @param bmx280 Driver structure.
 * @param temperature The temperature in C (0.01 degree C increments)
 * @param pressure The pressure in Pa (1/256 Pa increments)
 * @param humidity The humidity in %RH (1/1024 %RH increments) (UINT32_MAX when invlaid.)
 */
BMXAPI esp_err_t bmx280_readout(bmx280_t *bmx280, int32_t *temperature, uint32_t *pressure, uint32_t *humidity);

/**
 * Convert sensor readout to floating point values.
 * @param tin Input temperature.
 * @param pin Input pressure.
 * @param hin Input humidity.
 * @param tout Output temperature. (C)
 * @param pout Output pressure. (Pa)
 * @param hout Output humidity. (%Rh)
 */
static inline void bmx280_readout2float(int32_t* tin, uint32_t *pin, uint32_t *hin, float *tout, float *pout, float *hout)
{
    if (tin && tout)
        *tout = (float)*tin * 0.01f;
    if (pin && pout)
        *pout = (float)*pin * (1.0f/256.0f);
    if (hin && hout)
        *hout = (*hin == UINT32_MAX) ? -1.0f : (float)*hin * (1.0f/1024.0f);
}

/**
 * Read sensor values as floating point numbers.
 * @param bmx280 Driver structure.
 * @param temperature The temperature in C.
 * @param pressure The pressure in Pa.
 * @param humidity The humidity in %RH.
 */
static inline esp_err_t bmx280_readoutFloat(bmx280_t *bmx280, float* temperature, float* pressure, float* humidity)
{
    int32_t t; uint32_t p, h;
    esp_err_t err = bmx280_readout(bmx280, &t, &p, &h);

    if (err == ESP_OK)
    {
        bmx280_readout2float(&t, &p, &h, temperature, pressure, humidity);
    }

    return err;
}

#ifdef __cplusplus
};
#endif

#endif
