/*
 * crank_decoder.c
 *
 * Created on: Aug 20, 2025
 */

/*
 * crank_decoder.c
 *
 * Module to decode the crankshaft signal (CKP) using a timer in Input Capture mode.
 */

#include "stm32h7xx.h"  // For direct register access on STM32H7xx
#include "crank_decoder.h"

// Function to initialize crank decoder
void Crank_Init(void) {
    // Enable input capture on channel 1
    TIM2->CCER |= TIM_CCER_CC1E;
    // Enable capture/compare interrupt for channel 1
    TIM2->DIER |= TIM_DIER_CC1IE;
    // Enable the timer counter
    TIM2->CR1 |= TIM_CR1_CEN;
}

// IRQ handler for TIM2 (replaces HAL callback for low-level access)
void TIM2_IRQHandler(void) {
    // Check if capture/compare flag for channel 1 is set
    if (TIM2->SR & TIM_SR_CC1IF) {
        // Clear the interrupt flag
        TIM2->SR &= ~TIM_SR_CC1IF;

        // Get current capture tick from CCR1
        uint32_t current_capture_tick = TIM2->CCR1;

        // If last capture is valid
        if (last_capture_tick != 0) {
            // Calculate delta ticks considering overflow
            uint32_t dt_ticks;
            if (current_capture_tick >= last_capture_tick) {
                dt_ticks = current_capture_tick - last_capture_tick;
            } else {
                // Handle timer overflow
                dt_ticks = (0xFFFF - last_capture_tick) + current_capture_tick + 1;
            }

            // Detect missing tooth for synchronization
            if (!engine_is_synced && last_dt_ticks > 0) {
                if ((float)dt_ticks > 1.5f * (float)last_dt_ticks) {
                    engine_is_synced = 1;
                    tooth_counter = 0;
                    // Set engine angle at synchronization point
                    g_engineState.engine_angle_deg = 360.0f - 90.0f;
                }
            }

            // Calculate RPM
            float time_per_tooth_s = (float)dt_ticks / (float)TIMER_CLOCK_FREQ_HZ;
            if (time_per_tooth_s > 0.0f) {
                g_engineState.rpm = (uint32_t)((60.0f / time_per_tooth_s) / g_ecuConfig.ckp.num_teeth_on_wheel);
            }

            // Update last delta ticks
            last_dt_ticks = dt_ticks;

            // Update engine angle if synced
            if (engine_is_synced) {
                float degrees_per_tooth = 360.0f / g_ecuConfig.ckp.num_teeth_on_wheel;
                g_engineState.engine_angle_deg += degrees_per_tooth;
                tooth_counter++;

                // Keep angle within 0-720 degrees
                if (g_engineState.engine_angle_deg >= 720.0f) {
                    g_engineState.engine_angle_deg -= 720.0f;
                }
            }

            // Call engine control callback
            EngineControl_CrankEventCallback();
        }

        // Update last capture tick
        last_capture_tick = current_capture_tick;
    }
}