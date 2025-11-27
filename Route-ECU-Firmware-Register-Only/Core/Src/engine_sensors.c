/*
 * engine_sensors.c
 *
 * Created on: Aug 19, 2025
 */

#include "stm32h7xx.h"  // For direct register access on STM32H7xx
#include "engine_sensors.h"
#include "cmsis_os.h"
#include <stdbool.h>

// Function to initialize engine sensors
void EngineSensors_Init(void) {
    // Start ADC1 with DMA (assuming DMA configured, start conversion)
    ADC1->CR |= ADC_CR_ADSTART;
    // Start ADC2 with DMA
    ADC2->CR |= ADC_CR_ADSTART;
}

// Function to process sensor data
void ProcessSensorData(void) {
    // Check if ADC conversion is complete
    if (!g_adc_conversion_complete) {
        return;
    }

    // Convert ADC values to voltages for sensors
    for (int i = 0; i < NUM_SENSOR_CHANNELS; i++) {
        g_sensor_voltages[i] = (g_adc_dma_buffer_sensors[i] / ADC_MAX_VALUE) * ADC_VREF;
    }
    // Convert battery ADC value to voltage
    g_battery_adc_voltage = (g_adc_dma_buffer_battery[0] / ADC_MAX_VALUE) * ADC_VREF;

    // Calculate battery voltage using divider ratio
    g_engineData.battery_voltage = g_battery_adc_voltage * BATT_DIVIDER_RATIO;

    // Process each sensor channel
    for (int i = 0; i < NUM_SENSOR_CHANNELS; i++) {
        // Scale sensor voltage to 5V range
        float sensor_voltage_5v_scale = g_sensor_voltages[i] * SENSOR_DIVIDER_RATIO;
        // Get current sensor type from config
        SensorType_e current_sensor_type = g_ecuConfig.sensor_channel_map[i];

        // Switch based on sensor type
        switch (current_sensor_type) {
            case SENSOR_TYPE_TPS:
                // Calculate TPS percentage
                g_engineData.tps_percent = (sensor_voltage_5v_scale / 5.0f) * 100.0f;
                break;

            case SENSOR_TYPE_CLT:
                // g_engineData.clt_celsius = InterpolateTemp(sensor_voltage_5v_scale, CLT_TABLE); (commented in original)
                break;

            case SENSOR_TYPE_IAT:
                // g_engineData.iat_celsius = InterpolateTemp(sensor_voltage_5v_scale, IAT_TABLE); (commented in original)
                break;

            case SENSOR_TYPE_CKP:
                // g_engineData.iat_celsius = InterpolateTemp(sensor_voltage_5v_scale, IAT_TABLE); (commented in original, possible typo)
                break;

            case SENSOR_TYPE_CMP:
                // g_engineData.iat_celsius = InterpolateTemp(sensor_voltage_5v_scale, IAT_TABLE); (commented in original, possible typo)
                break;

            case SENSOR_TYPE_NONE:
            default:
                // No action for none or default
                break;
        }
    }

    // Reset conversion complete flag
    g_adc_conversion_complete = 0;
}

// This is the RTOS task that will read and process the sensors
void Task_ReadSensors(void const * argument) {
    // Get current tick
    uint32_t last_tick = osKernelSysTick();

    for(;;) {
        // Delay until next 20ms interval
        osDelayUntil(&last_tick, 20);

        // If conversion complete, process data
        if (g_adc_conversion_complete) {
            ProcessSensorData();
            g_adc_conversion_complete = 0;
        }
    }
}

// DMA IRQ handler for ADC1 completion (replaces HAL callback)
void DMA1_Stream0_IRQHandler(void) {  // Assuming DMA1 Stream0 for ADC1
    // Check if transfer complete
    if (DMA1_Stream0->ISR & DMA_ISR_TCIF0) {
        // Clear flag
        DMA1_Stream0->IFCR |= DMA_IFCR_CTCIF0;
        // Set conversion complete flag
        g_adc_conversion_complete = 1;
    }
}