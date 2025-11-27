/*
 * injector_driver.c
 *
 * Created on: Aug 20, 2025
 */

#include "injector_driver.h"
#include "main.h"

// ============================================================
// Initialization
// ============================================================
void Injector_Init(void) {
    // Starts PWM on all channels (with duty = 0)
    HAL_TIM_PWM_Start(&htim1, INJECTOR_1_CHANNEL);
    HAL_TIM_PWM_Start(&htim1, INJECTOR_2_CHANNEL);
    HAL_TIM_PWM_Start(&htim1, INJECTOR_3_CHANNEL);
    HAL_TIM_PWM_Start(&htim1, INJECTOR_4_CHANNEL);

    __HAL_TIM_SET_COMPARE(&htim1, INJECTOR_1_CHANNEL, 0);
    __HAL_TIM_SET_COMPARE(&htim1, INJECTOR_2_CHANNEL, 0);
    __HAL_TIM_SET_COMPARE(&htim1, INJECTOR_3_CHANNEL, 0);
    __HAL_TIM_SET_COMPARE(&htim1, INJECTOR_4_CHANNEL, 0);
}

// ============================================================
// Schedule injection pulse (in ms)
// ============================================================
void Injector_SchedulePulse(uint8_t cylinder_index, float pulse_width_ms) {
    if (pulse_width_ms <= 0.05f) return; // ignore very short pulses

    // 1. Convert pulse width to timer ticks
    uint32_t pulse_ticks = (uint32_t)(pulse_width_ms * 1000.0f *
                                      (TIMER_CLOCK_FREQ_HZ / 1000000UL));

    // 2. Configure timer/channel according to cylinder
    switch (cylinder_index) {
        case 0: // Injector 1
        	htim1.Instance->ARR = pulse_ticks + 20;
        	htim1.Instance->CCR1 = pulse_ticks;
            HAL_TIM_OnePulse_Start(&htim1, INJECTOR_1_CHANNEL);
            break;

        case 1: // Injector 2
        	htim1.Instance->ARR = pulse_ticks + 20;
        	htim1.Instance->CCR2 = pulse_ticks;
            HAL_TIM_OnePulse_Start(&htim1, INJECTOR_2_CHANNEL);
            break;

        case 2: // Injector 3
        	htim1.Instance->ARR = pulse_ticks + 20;
        	htim1.Instance->CCR1 = pulse_ticks;
            HAL_TIM_OnePulse_Start(&htim1, INJECTOR_3_CHANNEL);
            break;

        case 3: // Injector 4
        	htim1.Instance->ARR = pulse_ticks + 20;
        	htim1.Instance->CCR2 = pulse_ticks;
            HAL_TIM_OnePulse_Start(&htim1, INJECTOR_4_CHANNEL);
            break;
    }
}

void Injector_TestPulse(uint8_t cylinder_index, float pulse_width_ms) {
    if (cylinder_index >= 4 || pulse_width_ms <= 0.0f || pulse_width_ms > 50.0f) {
        return; // Invalid parameters
    }

    // 1. Convert pulse width in milliseconds to timer ticks (microseconds)
    uint32_t pulse_ticks = (uint32_t)(pulse_width_ms * 1000.0f);

    // 2. Select the correct timer/channel and trigger the pulse
    switch (cylinder_index) {
        case 0:
        	htim1.Instance->ARR = pulse_ticks + 20;
        	htim1.Instance->CCR1 = pulse_ticks;
            HAL_TIM_OnePulse_Start(&htim1, INJECTOR_1_CHANNEL);
            break;

        case 1:
        	htim1.Instance->ARR = pulse_ticks + 20;
            htim1.Instance->CCR2 = pulse_ticks;
            HAL_TIM_OnePulse_Start(&htim1, INJECTOR_2_CHANNEL);
            break;

        case 2:
        	htim1.Instance->ARR = pulse_ticks + 20;
        	htim1.Instance->CCR3 = pulse_ticks;
            HAL_TIM_OnePulse_Start(&htim1, INJECTOR_3_CHANNEL);
            break;

        case 3:
        	htim1.Instance->ARR = pulse_ticks + 20;
        	htim1.Instance->CCR4 = pulse_ticks;
            HAL_TIM_OnePulse_Start(&htim1, INJECTOR_4_CHANNEL);
            break;
    }
}

// ============================================================
// Optional callback (safety) -> zeros duty after the pulse
// ============================================================
void Injector_TimerCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM1) {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);
    }
}