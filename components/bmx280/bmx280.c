/**
 * BMX280 - BME280 & BMP280 Driver for Esspressif ESP-32.
 *
 * MIT License
 *
 * Copyright (C) 2020 Halit Utku Maden
 * Please contact at <utkumaden@hotmail.com>
 */

// LEGAL NOTE:
// Any code between below the caption "// HERE BE DRAGONS" and above the caption 
// "// END OF DRAGONS" contains modified versions of code owned by Bosch 
// Sensortec GmbH and it is not clearly licensed, therefore this code is not 
// covered by the MIT of this repository. Use at your own risk.

#include "bmx280.h"
#include "esp_log.h"

#include <stdlib.h>
#include <stdint.h>
#include <string.h>

// [BME280] Register address of humidity least significant byte.
#define BMX280_REG_HUMI_LSB 0xFE
// [BME280] Register address of humidity most significant byte.
#define BMX280_REG_HUMI_MSB 0xFD

// Register address of temperature fraction significant byte.
#define BMX280_REG_TEMP_XSB 0xFC
// Register address of temperature least significant byte.
#define BMX280_REG_TEMP_LSB 0xFB
// Register address of temperature most significant byte.
#define BMX280_REG_TEMP_MSB 0xFA

// Register address of pressure fraction significant byte.
#define BMX280_REG_PRES_XSB 0xF9
// Register address of pressure least significant byte.
#define BMX280_REG_PRES_LSB 0xF8
// Register address of pressure most significant byte.
#define BMX280_REG_PRES_MSB 0xF7

// Register address of sensor configuration.
#define BMX280_REG_CONFIG 0xF5
// Register address of sensor measurement control.
#define BMX280_REG_MESCTL 0xF4
// Register address of sensor status.
#define BMX280_REG_STATUS 0xF3
// [BME280] Register address of humidity control.
#define BMX280_REG_HUMCTL 0xF2

// [BME280] Register address of calibration constants. (high bank)
#define BMX280_REG_CAL_HI 0xE1
// Register address of calibration constants. (low bank)
#define BMX280_REG_CAL_LO 0x88

// Register address for sensor reset.
#define BMX280_REG_RESET 0xE0
// Chip reset vector.
#define BMX280_RESET_VEC 0xB6

// Register address for chip identification number.
#define BMX280_REG_CHPID 0xD0
// Value of REG_CHPID for BME280
#define BME280_ID  0x60
// Value of REG_CHPID for BMP280 (Engineering Sample 1)
#define BMP280_ID0 0x56
// Value of REG_CHPID for BMP280 (Engineering Sample 2)
#define BMP280_ID1 0x57
// Value of REG_CHPID for BMP280 (Production)
#define BMP280_ID2 0x58

struct bmx280_t{
    // I2C port.
    i2c_port_t i2c_port;
    // Slave Address of sensor.
    uint8_t slave;
    // Chip ID of sensor
    uint8_t chip_id;
    // Compensation data
    struct {
        uint16_t T1;
        int16_t T2;
        int16_t T3;
        uint16_t P1;
        int16_t P2;
        int16_t P3;
        int16_t P4;
        int16_t P5;
        int16_t P6;
        int16_t P7;
        int16_t P8;
        int16_t P9;
    #if !(CONFIG_BMX280_EXPECT_BMP280)
        uint8_t H1;
        int16_t H2;
        uint8_t H3;
        int16_t H4;
        int16_t H5;
        int8_t H6;
    #endif
    } cmps;
    // Storage for a variable proportional to temperature.
    int32_t t_fine;
};

/**
 * Macro that identifies a chip id as BME280 or BMP280
 * @note Only use when the chip is verified to be either a BME280 or BMP280.
 * @see bmx280_verify
 * @param chip_id The chip id.
 */
#define bmx280_isBME(chip_id) ((chip_id) == BME280_ID)
/**
 * Macro to verify a the chip id matches with the expected values.
 * @note Use when the chip needs to be verified as a BME280 or BME280.
 * @see bmx280_isBME
 * @param chip_id The chip id.
 */
#define bmx280_verify(chip_id) (((chip_id) == BME280_ID) || ((chip_id) == BMP280_ID2) || ((chip_id) == BMP280_ID1) || ((chip_id) == BMP280_ID0))

/**
 * Returns false if the sensor was not found.
 * @param bmx280 The driver structure.
 */
#define bmx280_validate(bmx280) (!(bmx280->slave == 0xDE && bmx280->chip_id == 0xAD))

/**
 * Read from sensor.
 * @param bmx280 Driver Sturcture.
 * @param addr Register address.
 * @param dout Data to read.
 * @param size The number of bytes to read.
 * @returns Error codes.
 */
static esp_err_t bmx280_read(bmx280_t *bmx280, uint8_t addr, uint8_t *dout, size_t size)
{
    esp_err_t err;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    if (cmd)
    {
        // Write register address
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, bmx280->slave | I2C_MASTER_WRITE, true);
        i2c_master_write_byte(cmd, addr, true);

        // Read Registers
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, bmx280->slave | I2C_MASTER_READ, true);
        i2c_master_read(cmd, dout, size, I2C_MASTER_LAST_NACK);
        i2c_master_stop(cmd);
        
        xSemaphoreTake(i2c_semaphore, portMAX_DELAY);
        err = i2c_master_cmd_begin(bmx280->i2c_port, cmd, CONFIG_BMX280_TIMEOUT);
        xSemaphoreGive(i2c_semaphore);
        i2c_cmd_link_delete(cmd);
        return err;
    }
    else
    {
        return ESP_ERR_NO_MEM;
    }
}

static esp_err_t bmx280_write(bmx280_t* bmx280, uint8_t addr, const uint8_t *din, size_t size)
{
    esp_err_t err;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    if (cmd)
    {
        for (int i = 0; i < size; i++)
        {
            i2c_master_start(cmd);
            i2c_master_write_byte(cmd, bmx280->slave | I2C_MASTER_WRITE, true);
            // Register
            i2c_master_write_byte(cmd, addr + i, true);
            //Data
            i2c_master_write_byte(cmd, din[i], true);
        }
        i2c_master_stop(cmd);

        xSemaphoreTake(i2c_semaphore, portMAX_DELAY);
        err = i2c_master_cmd_begin(bmx280->i2c_port, cmd, CONFIG_BMX280_TIMEOUT);
        xSemaphoreGive(i2c_semaphore);
        i2c_cmd_link_delete(cmd);
        return err;
    }
    else
    {
        return ESP_ERR_NO_MEM;
    }
}

static esp_err_t bmx280_probe_address(bmx280_t *bmx280)
{
    esp_err_t err = bmx280_read(bmx280, BMX280_REG_CHPID, &bmx280->chip_id, sizeof bmx280->chip_id);

    if (err == ESP_OK)
    {
        if (
        #if CONFIG_BMX280_EXPECT_BME280
            bmx280->chip_id == BME280_ID
        #elif CONFIG_BMX280_EXPECT_BMP280
            bmx280->chip_id == BMP280_ID2 || bmx280->chip_id == BMP280_ID1 || bmx280->chip_id == BMP280_ID0
        #else
            bmx280_verify(bmx280->chip_id)
        #endif
        )
        {
            ESP_LOGI("bmx280", "Probe success: address=%hhx, id=%hhx", bmx280->slave, bmx280->chip_id);
           return ESP_OK;
        }
        else
        {
            err = ESP_ERR_NOT_FOUND;
        }
    }

    ESP_LOGW("bmx280", "Probe failure: address=%hhx, id=%hhx, reason=%s", bmx280->slave, bmx280->chip_id, esp_err_to_name(err));
    return err;
}

static esp_err_t bmx280_probe(bmx280_t *bmx280)
{
    ESP_LOGI("bmx280", "Probing for BMP280/BME280 sensors on I2C %d", bmx280->i2c_port);

    #if CONFIG_BMX280_ADDRESS_HI
    bmx280->slave = 0xEE;
    return bmx280_probe_address(bmx280);
    #elif CONFIG_BMX280_ADDRESS_LO
    bmx280->slave = 0xEC;
    return bmx280_probe_address(bmx280);
    #else
    esp_err_t err;
    bmx280->slave = 0xEC;
    if ((err = bmx280_probe_address(bmx280)) != ESP_OK)
    {
        bmx280->slave = 0xEE;
        if ((err = bmx280_probe_address(bmx280)) != ESP_OK)
        {
            ESP_LOGE("bmx280", "Sensor not found.");
            bmx280->slave = 0xDE;
            bmx280->chip_id = 0xAD;
        }
    }
    return err;
    #endif
}

static esp_err_t bmx280_reset(bmx280_t *bmx280)
{
    const static uint8_t din[] = { BMX280_RESET_VEC };
    return bmx280_write(bmx280, BMX280_REG_RESET, din, sizeof din);
}

static esp_err_t bmx280_calibrate(bmx280_t *bmx280)
{
    // Honestly, the best course of action is to read the high and low banks
    // into a buffer, then put them in the calibration values. Makes code
    // endian agnostic, and overcomes struct packing issues.
    // Also the BME280 high bank is weird.
    //
    // Write and pray to optimizations is my new motto.

    ESP_LOGI("bmx280", "Reading out calibration values...");

    esp_err_t err;
    uint8_t buf[26];

    // Low Bank
    err = bmx280_read(bmx280, BMX280_REG_CAL_LO, buf, sizeof buf);

    if (err != ESP_OK) return err;

    ESP_LOGI("bmx280", "Read Low Bank.");

    bmx280->cmps.T1 = buf[0] | (buf[1] << 8);
    bmx280->cmps.T2 = buf[2] | (buf[3] << 8);
    bmx280->cmps.T3 = buf[4] | (buf[5] << 8);
    bmx280->cmps.P1 = buf[6] | (buf[7] << 8);
    bmx280->cmps.P2 = buf[8] | (buf[9] << 8);
    bmx280->cmps.P3 = buf[10] | (buf[11] << 8);
    bmx280->cmps.P4 = buf[12] | (buf[13] << 8);
    bmx280->cmps.P5 = buf[14] | (buf[15] << 8);
    bmx280->cmps.P6 = buf[16] | (buf[17] << 8);
    bmx280->cmps.P7 = buf[18] | (buf[19] << 8);
    bmx280->cmps.P8 = buf[20] | (buf[21] << 8);
    bmx280->cmps.P9 = buf[22] | (buf[23] << 8);

    #if !(CONFIG_BMX280_EXPECT_BMP280)

    #if CONFIG_BMX280_EXPECT_DETECT
    if (bmx280_isBME(bmx280->chip_id)) // Only conditional for detect scenario.
    #endif
    {
        // First get H1 out of the way.
        bmx280->cmps.H1 = buf[23];

        err = bmx280_read(bmx280, BMX280_REG_CAL_HI, buf, 7);

        if (err != ESP_OK) return err;

        ESP_LOGI("bmx280", "Read High Bank.");

        bmx280->cmps.H2 = buf[0] | (buf[1] << 8);
        bmx280->cmps.H3 = buf[2];
        bmx280->cmps.H4 = (buf[3] << 4) | (buf[4] & 0x0F);
        bmx280->cmps.H5 = (buf[4] >> 4) | (buf[5] << 4);
        bmx280->cmps.H6 = buf[6];
    }

    #endif

    return ESP_OK;
}

bmx280_t* bmx280_create(i2c_port_t port)
{
    bmx280_t* bmx280 = malloc(sizeof(bmx280_t));
    if (bmx280)
    {
        memset(bmx280, 0, sizeof(bmx280_t));

        bmx280->i2c_port = port;
        bmx280->slave = 0xDE;
        bmx280->chip_id = 0xAD;
    }
    return bmx280;
}

void bmx280_close(bmx280_t *bmx280)
{
    free(bmx280);
}

esp_err_t bmx280_init(bmx280_t* bmx280)
{
    if (bmx280 == NULL) return ESP_ERR_INVALID_ARG;

    esp_err_t error = bmx280_probe(bmx280) || bmx280_reset(bmx280);

    if (error == ESP_OK)
    {
        // Give the sensor 10 ms delay to reset.
        vTaskDelay(pdMS_TO_TICKS(10));

        // Read calibration data.
        bmx280_calibrate(bmx280);

        ESP_LOGI("bmx280", "Dumping calibration...");
        ESP_LOG_BUFFER_HEX("bmx280", &bmx280->cmps, sizeof(bmx280->cmps));
    }

    return error;
}

esp_err_t bmx280_configure(bmx280_t* bmx280, bmx280_config_t *cfg)
{
    if (bmx280 == NULL || cfg == NULL) return ESP_ERR_INVALID_ARG;
    if (!bmx280_validate(bmx280)) return ESP_ERR_INVALID_STATE;

    // Always set ctrl_meas first.
    uint8_t num = (cfg->t_sampling << 5) | (cfg->p_sampling << 2) | BMX280_MODE_SLEEP;
    esp_err_t err = bmx280_write(bmx280, BMX280_REG_MESCTL, &num, sizeof num);

    if (err) return err;

    // We can set cfg now.
    num = (cfg->t_standby << 5) | (cfg->iir_filter << 2);
    err = bmx280_write(bmx280, BMX280_REG_CONFIG, &num, sizeof num);

    if (err) return err;

    #if !(CONFIG_BMX280_EXPECT_BMP280)
    #if CONFIG_BMX280_EXPECT_DETECT
    if (bmx280_isBME(bmx280->chip_id))
    #elif CONFIG_BMX280_EXPECT_BME280
    #endif
    {
        num = cfg->h_sampling;
        err = bmx280_write(bmx280, BMX280_REG_HUMCTL, &num, sizeof(num));

        if (err) return err;
    }
    #endif

    // f = 0; 
    return ESP_OK;
}

esp_err_t bmx280_setMode(bmx280_t* bmx280, bmx280_mode_t mode)
{
    uint8_t ctrl_mes;
    esp_err_t err;

    if ((err = bmx280_read(bmx280, BMX280_REG_MESCTL, &ctrl_mes, 1)) != ESP_OK)
        return err;

    ctrl_mes = (ctrl_mes & (~3)) | mode;

    return bmx280_write(bmx280, BMX280_REG_MESCTL, &ctrl_mes, 1);
}

esp_err_t bmx280_getMode(bmx280_t* bmx280, bmx280_mode_t* mode)
{
    uint8_t ctrl_mes;
    esp_err_t err;

    if ((err = bmx280_read(bmx280, BMX280_REG_MESCTL, &ctrl_mes, 1)) != ESP_OK)
        return err;

    ctrl_mes &= 3;

    switch (ctrl_mes)
    {
    default:
        *mode = ctrl_mes; break;
    case (BMX280_MODE_FORCE + 1):
        *mode = BMX280_MODE_FORCE; break;
    }

    return ESP_OK;
}

bool bmx280_isSampling(bmx280_t* bmx280)
{
    uint8_t status;
    if (bmx280_read(bmx280, BMX280_REG_STATUS, &status, 1) == ESP_OK)
        return (status & (1 << 3)) != 0;
    else
        return false;
}


// LEGAL NOTE:
// Any code between below the caption "// HERE BE DRAGONS" and above the caption 
// "// END OF DRAGONS" contains modified versions of code owned by Bosch 
// Sensortec GmbH and it is not clearly licensed, therefore this code is not 
// covered by the MIT of this repository. Use at your own risk.

// HERE BE DRAGONS
// This code is revised from the Bosch code within the datasheet of the BME280.
// I do not understand it enough to tell you what it does.
// No touchies.

// Returns temperature in DegC, resolution is 0.01 DegC. Output value of “5123” equals 51.23 DegC.
// t_fine carries fine temperature as global value
int32_t BME280_compensate_T_int32(bmx280_t *bmx280, int32_t adc_T)
{
    int32_t var1, var2, T; 
    var1 = ((((adc_T>>3) -((int32_t)bmx280->cmps.T1<<1))) * ((int32_t)bmx280->cmps.T2)) >> 11;
    var2  =(((((adc_T>>4) -((int32_t)bmx280->cmps.T1)) * ((adc_T>>4) -((int32_t)bmx280->cmps.T1))) >> 12) * ((int32_t)bmx280->cmps.T3)) >> 14;
    bmx280->t_fine = var1 + var2;
    T  = (bmx280->t_fine * 5 + 128) >> 8;
    return T;
}

// Returns pressure in Pa as unsigned 32 bit integer in Q24.8 format (24 integer bits and 8 fractional bits).
// Output value of “24674867” represents 24674867/256 = 96386.2 Pa = 963.862 hPa
uint32_t BME280_compensate_P_int64(bmx280_t *bmx280, int32_t adc_P)
{
    int64_t var1, var2, p;
    var1 = ((int64_t)bmx280->t_fine) -128000;
    var2 = var1 * var1 * (int64_t)bmx280->cmps.P6;
    var2 = var2 + ((var1*(int64_t)bmx280->cmps.P5)<<17);
    var2 = var2 + (((int64_t)bmx280->cmps.P4)<<35);
    var1 = ((var1 * var1 * (int64_t)bmx280->cmps.P3)>>8) + ((var1 * (int64_t)bmx280->cmps.P2)<<12);
    var1 = (((((int64_t)1)<<47)+var1))*((int64_t)bmx280->cmps.P1)>>33;
    if(var1 == 0){ 
        return 0; // avoid exception caused by division by zero
    }
    p = 1048576-adc_P;
    p = (((p<<31)-var2)*3125)/var1;
    var1 = (((int64_t)bmx280->cmps.P9) * (p>>13) * (p>>13)) >> 25;
    var2 =(((int64_t)bmx280->cmps.P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t)bmx280->cmps.P7)<<4);
    return (uint32_t)p;
}

#if !CONFIG_BMX280_EXPECT_BMP280

// Returns humidity in %RH as unsigned 32 bit integer in Q22.10 format (22 integer and 10 fractional bits).
// Output value of “47445” represents 47445/1024 = 46.333 %RH
uint32_t bme280_compensate_H_int32(bmx280_t *bmx280, int32_t adc_H)
{
    int32_t v_x1_u32r;
    v_x1_u32r = (bmx280->t_fine -((int32_t)76800));
    v_x1_u32r = (((((adc_H << 14) -(((int32_t)bmx280->cmps.H4) << 20) -(((int32_t)bmx280->cmps.H5) * v_x1_u32r)) + ((int32_t)16384)) >> 15) * (((((((v_x1_u32r * ((int32_t)bmx280->cmps.H6)) >> 10) * (((v_x1_u32r * ((int32_t)bmx280->cmps.H3)) >> 11) + ((int32_t)32768))) >> 10) + ((int32_t)2097152)) * ((int32_t)bmx280->cmps.H2) + 8192) >> 14));
    v_x1_u32r = (v_x1_u32r -(((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * ((int32_t)bmx280->cmps.H1)) >> 4));
    v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
    v_x1_u32r = (v_x1_u32r > 419430400? 419430400: v_x1_u32r);
    return(uint32_t)(v_x1_u32r>>12);
}

#endif

// END OF DRAGONS

esp_err_t bmx280_readout(bmx280_t *bmx280, int32_t *temperature, uint32_t *pressure, uint32_t *humidity)
{
    if (bmx280 == NULL) return ESP_ERR_INVALID_ARG;
    if (!bmx280_validate(bmx280)) return ESP_ERR_INVALID_STATE;

    uint8_t buffer[3];
    esp_err_t error;

    if (temperature)
    {
        if ((error = bmx280_read(bmx280, BMX280_REG_TEMP_MSB, buffer, 3)) != ESP_OK)
            return error;

        *temperature = BME280_compensate_T_int32(bmx280,
                        (buffer[0] << 12) | (buffer[1] << 4) | (buffer[0] >> 4)
                    );
    }

    if (pressure)
    {
        if ((error = bmx280_read(bmx280, BMX280_REG_PRES_MSB, buffer, 3)) != ESP_OK)
            return error;

        *pressure = BME280_compensate_P_int64(bmx280, 
                        (buffer[0] << 12) | (buffer[1] << 4) | (buffer[0] >> 4)
                    );
    }

    #if !(CONFIG_BMX280_EXPECT_BMP280)
    #if CONFIG_BMX280_EXPECT_DETECT
    if (bmx280_isBME(bmx280->chip_id))
    #elif CONFIG_BMX280_EXPECT_BME280
    #endif
    {
        if (humidity)
        {
            if ((error = bmx280_read(bmx280, BMX280_REG_HUMI_MSB, buffer, 2)) != ESP_OK)
                return error;

            *humidity = bme280_compensate_H_int32(bmx280,
                            (buffer[0] << 8) | buffer[0]
                        );
        }
    }
    #if CONFIG_BMX280_EXPECT_DETECT
    else if (humidity)
        *humidity = UINT32_MAX;
    #endif
    #else
    if (humidity)
        *humidity = UINT32_MAX;
    #endif

    return ESP_OK;
}
