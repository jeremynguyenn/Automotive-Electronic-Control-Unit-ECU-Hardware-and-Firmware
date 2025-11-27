/*
 * injector_driver.c
 *
 * Author: Nguyen Nhan
 */

#include "injector_driver.h"
#include "main.h"

// ============================================================
// Initialization
// ============================================================
void Injector_Init(void) {
    // Start PWM on all channels (with duty = 0)
    HAL_TIM_PWM_Start(&htim1, INJECTOR_1_CHANNEL); // Start PWM for injector 1
    HAL_TIM_PWM_Start(&htim1, INJECTOR_2_CHANNEL); // Start PWM for injector 2
    HAL_TIM_PWM_Start(&htim1, INJECTOR_3_CHANNEL); // Start PWM for injector 3
    HAL_TIM_PWM_Start(&htim1, INJECTOR_4_CHANNEL); // Start PWM for injector 4

    __HAL_TIM_SET_COMPARE(&htim1, INJECTOR_1_CHANNEL, 0); // Set compare value to 0 for injector 1
    __HAL_TIM_SET_COMPARE(&htim1, INJECTOR_2_CHANNEL, 0); // Set compare value to 0 for injector 2
    __HAL_TIM_SET_COMPARE(&htim1, INJECTOR_3_CHANNEL, 0); // Set compare value to 0 for injector 3
    __HAL_TIM_SET_COMPARE(&htim1, INJECTOR_4_CHANNEL, 0); // Set compare value to 0 for injector 4
}

// ============================================================
// Schedule injection pulse (in ms)
// ============================================================
void Injector_SchedulePulse(uint8_t cylinder_index, float pulse_width_ms) {
    if (pulse_width_ms <= 0.05f) return; // Ignore very short pulses

    // 1. Convert pulse width to timer ticks
    uint32_t pulse_ticks = (uint32_t)(pulse_width_ms * 1000.0f *
                                      (TIMER_CLOCK_FREQ_HZ / 1000000UL)); // Calculate ticks based on frequency

    // 2. Configure timer/channel based on cylinder
    switch (cylinder_index) {
        case 0: // Injector 1
        	htim1.Instance->ARR = pulse_ticks + 20; // Set auto-reload register
        	htim1.Instance->CCR1 = pulse_ticks; // Set compare register for channel 1
            HAL_TIM_OnePulse_Start(&htim1, INJECTOR_1_CHANNEL); // Start one-pulse mode
            break;

        case 1: // Injector 2
        	htim1.Instance->ARR = pulse_ticks + 20; // Set auto-reload register
        	htim1.Instance->CCR2 = pulse_ticks; // Set compare register for channel 2
            HAL_TIM_OnePulse_Start(&htim1, INJECTOR_2_CHANNEL); // Start one-pulse mode
            break;

        case 2: // Injector 3
        	htim1.Instance->ARR = pulse_ticks + 20; // Set auto-reload register
        	htim1.Instance->CCR1 = pulse_ticks; // Set compare register for channel 3 (note: CCR1 reused, possible typo?)
            HAL_TIM_OnePulse_Start(&htim1, INJECTOR_3_CHANNEL); // Start one-pulse mode
            break;

        case 3: // Injector 4
        	htim1.Instance->ARR = pulse_ticks + 20; // Set auto-reload register
        	htim1.Instance->CCR2 = pulse_ticks; // Set compare register for channel 4 (note: CCR2 reused, possible typo?)
            HAL_TIM_OnePulse_Start(&htim1, INJECTOR_4_CHANNEL); // Start one-pulse mode
            break;
    }
}

// Function to test injection pulse on a specific cylinder
void Injector_TestPulse(uint8_t cylinder_index, float pulse_width_ms) {
    if (cylinder_index >= 4 || pulse_width_ms <= 0.0f || pulse_width_ms > 50.0f) {
        return; // Invalid parameters
    }

    // 1. Convert pulse width in milliseconds to timer ticks (microseconds)
    uint32_t pulse_ticks = (uint32_t)(pulse_width_ms * 1000.0f); // Calculate ticks

    // 2. Select the correct timer/channel and trigger the pulse
    switch (cylinder_index) {
        case 0:
        	htim1.Instance->ARR = pulse_ticks + 20; // Set auto-reload register
        	htim1.Instance->CCR1 = pulse_ticks; // Set compare register for channel 1
            HAL_TIM_OnePulse_Start(&htim1, INJECTOR_1_CHANNEL); // Start one-pulse mode
            break;

        case 1:
        	htim1.Instance->ARR = pulse_ticks + 20; // Set auto-reload register
            htim1.Instance->CCR2 = pulse_ticks; // Set compare register for channel 2
            HAL_TIM_OnePulse_Start(&htim1, INJECTOR_2_CHANNEL); // Start one-pulse mode
            break;

        case 2:
        	htim1.Instance->ARR = pulse_ticks + 20; // Set auto-reload register
        	htim1.Instance->CCR3 = pulse_ticks; // Set compare register for channel 3
            HAL_TIM_OnePulse_Start(&htim1, INJECTOR_3_CHANNEL); // Start one-pulse mode
            break;

        case 3:
        	htim1.Instance->ARR = pulse_ticks + 20; // Set auto-reload register
        	htim1.Instance->CCR4 = pulse_ticks; // Set compare register for channel 4
            HAL_TIM_OnePulse_Start(&htim1, INJECTOR_4_CHANNEL); // Start one-pulse mode
            break;
    }
}

// ============================================================
// Optional callback (safety) -> reset duty after the pulse
// ============================================================
void Injector_TimerCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM1) { // Check if it's the correct timer
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0); // Reset compare for channel 1
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0); // Reset compare for channel 2
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0); // Reset compare for channel 3
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0); // Reset compare for channel 4
    }
}