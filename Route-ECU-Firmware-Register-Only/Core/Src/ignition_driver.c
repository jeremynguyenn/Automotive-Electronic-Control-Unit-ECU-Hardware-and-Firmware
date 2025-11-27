/*
 * ignition_driver.c
 *
 * Created on: Aug 20, 2025
 */

#include "stm32h7xx.h"  // For direct register access on STM32H7xx
#include "ignition_driver.h"

// --- Definition of Global Variables ---
// External ECU configuration structure
extern EcuConfig_t  g_ecuConfig;
// External engine data structure
extern EngineData_t g_engineData;
// External engine state structure
extern EngineState_t g_engineState;

// Function to initialize ignition system
void Ignition_Init(void) {
    // Enable PWM channels for all ignition coils
    TIM4->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E;
    // Set compare values to 0 for all channels
    TIM4->CCR1 = 0;
    TIM4->CCR2 = 0;
    TIM4->CCR3 = 0;
    TIM4->CCR4 = 0;
    // Enable the timer counter
    TIM4->CR1 |= TIM_CR1_CEN;
}

// ============================================================
// Schedule the spark trigger.
// ============================================================
void Ignition_ScheduleSpark(uint8_t cylinder_index, float advance_deg, float dwell_ms) {
    // Check if RPM is below minimum threshold, return if so
    if (g_engineState.rpm < 200) return;

    // Calculate time per degree in microseconds
    uint32_t time_per_degree_us = DegreesToTimeUs(1.0f); // µs per degree
    // Apply dwell compensation based on battery voltage and convert to microseconds
    uint32_t dwell_us = ApplyDwellCompensation(dwell_ms, g_engineData.battery_voltage) * 1000.0f;
    // Calculate advance time in microseconds
    uint32_t advance_us = (uint32_t)(advance_deg * time_per_degree_us);

    // Calculate dwell start delay
    uint32_t dwell_start_delay = advance_us + dwell_us;

    // Set pulse ticks to dwell time
    uint32_t pulse_ticks = dwell_us;
    // Set delay ticks to dwell start delay
    uint32_t delay_ticks = dwell_start_delay;

    // Configure timer based on cylinder index
    switch (cylinder_index) {
        case 0:
            // Set auto-reload register
            TIM4->ARR = delay_ticks + pulse_ticks;
            // Set compare register for channel 1
            TIM4->CCR1 = delay_ticks;
            break;
        case 1:
            // Set auto-reload register
            TIM4->ARR = delay_ticks + pulse_ticks;
            // Set compare register for channel 2
            TIM4->CCR2 = delay_ticks;
            break;
        case 2:
            // Set auto-reload register
            TIM4->ARR = delay_ticks + pulse_ticks;
            // Set compare register for channel 3
            TIM4->CCR3 = delay_ticks;
            break;
        case 3:
            // Set auto-reload register
            TIM4->ARR = delay_ticks + pulse_ticks;
            // Set compare register for channel 4
            TIM4->CCR4 = delay_ticks;
            break;
    }
}

/**
 * @brief It sends a single test pulse to a specific ignition coil.
 */
void Ignition_TestPulse(uint8_t cylinder_index, float dwell_ms) {
    // Basic safety validation for parameters
    if (cylinder_index >= 4 || dwell_ms <= 0.0f || dwell_ms > 10.0f) {
        return; // Invalid parameters
    }

    // Converts the dwell time from milliseconds to ticks (microseconds)
    uint32_t pulse_ticks = (uint32_t)(dwell_ms * 1000.0f);

    // Select the correct timer channel and trigger a single pulse
    switch (cylinder_index) {
        case 0:
            // Set auto-reload register (slightly longer for full pulse)
            TIM4->ARR = pulse_ticks + 10;
            // Set compare register for channel 1
            TIM4->CCR1 = pulse_ticks;
            // Set one-pulse mode
            TIM4->CR1 |= TIM_CR1_OPM;
            // Enable timer to start the pulse
            TIM4->CR1 |= TIM_CR1_CEN;
            break;
        case 1:
            // Set auto-reload register
            TIM4->ARR = pulse_ticks + 10;
            // Set compare register for channel 2
            TIM4->CCR2 = pulse_ticks;
            // Set one-pulse mode
            TIM4->CR1 |= TIM_CR1_OPM;
            // Enable timer to start the pulse
            TIM4->CR1 |= TIM_CR1_CEN;
            break;
        case 2:
            // Set auto-reload register
            TIM4->ARR = pulse_ticks + 10;
            // Set compare register for channel 3
            TIM4->CCR3 = pulse_ticks;
            // Set one-pulse mode
            TIM4->CR1 |= TIM_CR1_OPM;
            // Enable timer to start the pulse
            TIM4->CR1 |= TIM_CR1_CEN;
            break;
        case 3:
            // Set auto-reload register
            TIM4->ARR = pulse_ticks + 10;
            // Set compare register for channel 4
            TIM4->CCR4 = pulse_ticks;
            // Set one-pulse mode
            TIM4->CR1 |= TIM_CR1_OPM;
            // Enable timer to start the pulse
            TIM4->CR1 |= TIM_CR1_CEN;
            break;
    }
}

// ============================================================
// Safety callbacks (turns off PWM after pulse)
// ============================================================
void Ignition_TimerCallback(TIM_HandleTypeDef *htim) {
    // Check if the timer instance is TIM4 (corrected from TIM1 in original)
    if (htim->Instance == TIM4) {
        // Set compare values to 0 for all channels to turn off PWM
        TIM4->CCR1 = 0;
        TIM4->CCR2 = 0;
        TIM4->CCR3 = 0;
        TIM4->CCR4 = 0;
    }
}

// ============================================================
// Auxiliary functions
// ============================================================

// Converts degrees of rotation into microseconds.
uint32_t DegreesToTimeUs(float degrees) {
    // Calculate revolutions per millisecond
    float rev_per_ms = (float)g_engineState.rpm / 60000.0f;
    // Calculate time per revolution in milliseconds
    float time_per_rev_ms = 1.0f / rev_per_ms;
    // Calculate time per degree in milliseconds
    float time_per_deg_ms = time_per_rev_ms / 360.0f;
    // Convert to microseconds and return
    return (uint32_t)(degrees * time_per_deg_ms * 1000.0f);
}

// Compensates for dwell time based on battery voltage (simplified table)
float ApplyDwellCompensation(float dwell_ms, float vbat) {
    // Apply compensation factor if voltage below 10V
    if (vbat < 10.0f) return dwell_ms * 1.3f;
    // Apply compensation factor if voltage below 12V
    if (vbat < 12.0f) return dwell_ms * 1.1f;
    // Apply compensation factor if voltage above 14.5V
    if (vbat > 14.5f) return dwell_ms * 0.9f;
    // No compensation otherwise
    return dwell_ms;
}