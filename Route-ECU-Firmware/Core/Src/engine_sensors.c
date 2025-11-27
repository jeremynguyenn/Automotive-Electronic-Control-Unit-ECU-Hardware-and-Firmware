/*
 * engine_sensors.c
 *
<<<<<<< HEAD
 *  Created on: Aug 19, 2025
=======
>>>>>>> 6da437a2b5ecaab585351f10c0837700ec4ca60e
 */

#include "engine_sensors.h"
#include "cmsis_os.h"
#include <stdbool.h>

void EngineSensors_Init(void) {
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)g_adc_dma_buffer_sensors, NUM_SENSOR_CHANNELS);
    HAL_ADC_Start_DMA(&hadc2, (uint32_t*)g_adc_dma_buffer_battery, 1);
}

void ProcessSensorData(void) {
    if (!g_adc_conversion_complete) {
        return;
    }

    for (int i = 0; i < NUM_SENSOR_CHANNELS; i++) {
        g_sensor_voltages[i] = (g_adc_dma_buffer_sensors[i] / ADC_MAX_VALUE) * ADC_VREF;
    }
    g_battery_adc_voltage = (g_adc_dma_buffer_battery[0] / ADC_MAX_VALUE) * ADC_VREF;

    g_engineData.battery_voltage = g_battery_adc_voltage * BATT_DIVIDER_RATIO;

    for (int i = 0; i < NUM_SENSOR_CHANNELS; i++) {
        float sensor_voltage_5v_scale = g_sensor_voltages[i] * SENSOR_DIVIDER_RATIO;
        SensorType_e current_sensor_type = g_ecuConfig.sensor_channel_map[i];

        switch (current_sensor_type) {
            case SENSOR_TYPE_TPS:
                g_engineData.tps_percent = (sensor_voltage_5v_scale / 5.0f) * 100.0f;
                break;

            case SENSOR_TYPE_CLT:
                // g_engineData.clt_celsius = InterpolateTemp(sensor_voltage_5v_scale, CLT_TABLE);
                break;

            case SENSOR_TYPE_IAT:
                // g_engineData.iat_celsius = InterpolateTemp(sensor_voltage_5v_scale, IAT_TABLE);
                break;

            case SENSOR_TYPE_CKP:
                // g_engineData.iat_celsius = InterpolateTemp(sensor_voltage_5v_scale, IAT_TABLE);
                break;

            case SENSOR_TYPE_CMP:
                // g_engineData.iat_celsius = InterpolateTemp(sensor_voltage_5v_scale, IAT_TABLE);
                break;

            case SENSOR_TYPE_NONE:
            default:

                break;
        }
    }

    g_adc_conversion_complete = 0;
}

// This is the RTOS task that will read and process the sensors
void Task_ReadSensors(void const * argument) {
    uint32_t last_tick = osKernelSysTick();

    for(;;) {
        // This task runs every 20ms
        osDelayUntil(&last_tick, 20);

        if (g_adc_conversion_complete) {
            ProcessSensorData();
            g_adc_conversion_complete = 0;
        }
    }
}

// The DMA callback interrupt remains the same
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
    if (hadc->Instance == ADC1) { // Check if it is the sensor ADC
        g_adc_conversion_complete = 1;
    }
}