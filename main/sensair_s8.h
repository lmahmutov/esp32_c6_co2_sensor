/* 
    Sensair S8 CO2 Sensor driver
*/

#ifndef SENSAIR_S8_H_  
#define SENSAIR_S8_H_

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "string.h"
#include "driver/gpio.h"

extern uint16_t CO2_value;
void uart_init(void);
void sensair_get_info();
void sensair_tx_task();
void sensair_rx_task();

#endif