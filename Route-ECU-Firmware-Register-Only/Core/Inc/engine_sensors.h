/*
 * engine_sensors.h
 *
 * Created on: Aug 19, 2025
 */

#ifndef INC_ENGINE_SENSORS_H_
#define INC_ENGINE_SENSORS_H_

#include "stm32h7xx.h"  // For direct register access on STM32H7xx
#include "main.h"
#include "ecu_config.h"

// External ECU configuration structure
extern EcuConfig_t  g_ecuConfig;
// External engine data structure
extern EngineData_t g_engineData;
// External engine state structure
extern EngineState_t g_engineState;

// Maximum ADC value for 12-bit resolution
#define ADC_MAX_VALUE 4095.0f
// ADC reference voltage
#define ADC_VREF      3.3f
// Number of sensor channels
#define NUM_SENSOR_CHANNELS 12

// Battery voltage divider ratio
const float BATT_DIVIDER_RATIO = 18.81f / 3.3f;
// Sensor voltage divider ratio
const float SENSOR_DIVIDER_RATIO = 5.0f / 3.205f;

// DMA buffer for sensor ADC values
uint16_t g_adc_dma_buffer_sensors[NUM_SENSOR_CHANNELS];
// DMA buffer for battery ADC value
uint16_t g_adc_dma_buffer_battery[1];
// Flag for ADC conversion complete
volatile int g_adc_conversion_complete = 0;

// Raw ADC values array
uint16_t g_raw_adc_values[12];
// Sensor voltages array
float    g_sensor_voltages[12];
// Battery ADC voltage
float    g_battery_adc_voltage;

// Sensor voltages array (repeated in original, kept as is)
float g_sensor_voltages[12];
// Battery ADC voltage (repeated in original, kept as is)
float g_battery_adc_voltage;

// Function to initialize engine sensors
void EngineSensors_Init(void);
// RTOS task to read sensors
void Task_ReadSensors(void const * argument);

#endif /* INC_ENGINE_SENSORS_H_ */