/*
 * communication_serial_protocol.c
 *
 * Created on: Aug 20, 2025
 */

#include "stm32h7xx.h"  // For direct register access on STM32H7xx
#include "injector_driver.h"  // Assuming this is the intended content based on original code

// ============================================================
// Initialization
// ============================================================
void Injector_Init(void) {
    // Enable PWM channels for all injectors
    TIM1->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E;
    // Set compare values to 0 for all channels
    TIM1->CCR1 = 0;
    TIM1->CCR2 = 0;
    TIM1->CCR3 = 0;
    TIM1->CCR4 = 0;
    // Enable the timer counter
    TIM1->CR1 |= TIM_CR1_CEN;
}

// ============================================================
// Schedule injection pulse (in ms)
// ============================================================
void Injector_SchedulePulse(uint8_t cylinder_index, float pulse_width_ms) {
    // Ignore very short pulses
    if (pulse_width_ms <= 0.05f) return;

    // Convert pulse width to timer ticks based on clock frequency
    uint32_t pulse_ticks = (uint32_t)(pulse_width_ms * 1000.0f *
                                      (TIMER_CLOCK_FREQ_HZ / 1000000UL));

    // Configure timer/channel according to cylinder
    switch (cylinder_index) {
        case 0: // Injector 1
            // Set auto-reload register
            TIM1->ARR = pulse_ticks + 20;
            // Set compare register for channel 1
            TIM1->CCR1 = pulse_ticks;
            // Set one-pulse mode
            TIM1->CR1 |= TIM_CR1_OPM;
            // Enable timer to start the pulse
            TIM1->CR1 |= TIM_CR1_CEN;
            break;

        case 1: // Injector 2
            // Set auto-reload register
            TIM1->ARR = pulse_ticks + 20;
            // Set compare register for channel 2
            TIM1->CCR2 = pulse_ticks;
            // Set one-pulse mode
            TIM1->CR1 |= TIM_CR1_OPM;
            // Enable timer to start the pulse
            TIM1->CR1 |= TIM_CR1_CEN;
            break;

        case 2: // Injector 3
            // Set auto-reload register
            TIM1->ARR = pulse_ticks + 20;
            // Set compare register for channel 1 (original, kept as is)
            TIM1->CCR1 = pulse_ticks;
            // Set one-pulse mode
            TIM1->CR1 |= TIM_CR1_OPM;
            // Enable timer to start the pulse
            TIM1->CR1 |= TIM_CR1_CEN;
            break;

        case 3: // Injector 4
            // Set auto-reload register
            TIM1->ARR = pulse_ticks + 20;
            // Set compare register for channel 2 (original, kept as is)
            TIM1->CCR2 = pulse_ticks;
            // Set one-pulse mode
            TIM1->CR1 |= TIM_CR1_OPM;
            // Enable timer to start the pulse
            TIM1->CR1 |= TIM_CR1_CEN;
            break;
    }
}

void Injector_TestPulse(uint8_t cylinder_index, float pulse_width_ms) {
    // Validate parameters
    if (cylinder_index >= 4 || pulse_width_ms <= 0.0f || pulse_width_ms > 50.0f) {
        return; // Invalid parameters
    }

    // Convert pulse width to timer ticks (microseconds)
    uint32_t pulse_ticks = (uint32_t)(pulse_width_ms * 1000.0f);

    // Select the correct timer/channel and trigger the pulse
    switch (cylinder_index) {
        case 0:
            // Set auto-reload register
            TIM1->ARR = pulse_ticks + 20;
            // Set compare register for channel 1
            TIM1->CCR1 = pulse_ticks;
            // Set one-pulse mode
            TIM1->CR1 |= TIM_CR1_OPM;
            // Enable timer to start the pulse
            TIM1->CR1 |= TIM_CR1_CEN;
            break;

        case 1:
            // Set auto-reload register
            TIM1->ARR = pulse_ticks + 20;
            // Set compare register for channel 2
            TIM1->CCR2 = pulse_ticks;
            // Set one-pulse mode
            TIM1->CR1 |= TIM_CR1_OPM;
            // Enable timer to start the pulse
            TIM1->CR1 |= TIM_CR1_CEN;
            break;

        case 2:
            // Set auto-reload register
            TIM1->ARR = pulse_ticks + 20;
            // Set compare register for channel 3
            TIM1->CCR3 = pulse_ticks;
            // Set one-pulse mode
            TIM1->CR1 |= TIM_CR1_OPM;
            // Enable timer to start the pulse
            TIM1->CR1 |= TIM_CR1_CEN;
            break;

        case 3:
            // Set auto-reload register
            TIM1->ARR = pulse_ticks + 20;
            // Set compare register for channel 4
            TIM1->CCR4 = pulse_ticks;
            // Set one-pulse mode
            TIM1->CR1 |= TIM_CR1_OPM;
            // Enable timer to start the pulse
            TIM1->CR1 |= TIM_CR1_CEN;
            break;
    }
}

// ============================================================
// Optional callback (safety) -> zeros duty after the pulse
// ============================================================
void Injector_TimerCallback(TIM_HandleTypeDef *htim) {
    // Check if the timer instance is TIM1
    if (htim->Instance == TIM1) {
        // Reset compare values for all channels
        TIM1->CCR1 = 0;
        TIM1->CCR2 = 0;
        TIM1->CCR3 = 0;
        TIM1->CCR4 = 0;
    }
}