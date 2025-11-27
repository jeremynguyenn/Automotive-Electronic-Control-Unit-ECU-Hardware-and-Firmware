/*
 * crank_decoder.c
 *
<<<<<<< HEAD
 *  Created on: Aug 20, 2025
=======
>>>>>>> 6da437a2b5ecaab585351f10c0837700ec4ca60e
 */


/*
 * crank_decoder.c
 *
 * Module to decode the crankshaft signal (CKP) using a timer in Input Capture mode.
 */

#include "crank_decoder.h"

void Crank_Init(void) {
    HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
}

// THIS FUNCTION IS CALLED BY THE TIMER ISR
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM2) { // Confirm it is the correct timer
        uint32_t current_capture_tick = HAL_TIM_GetCapturedValue(htim, TIM_CHANNEL_1);

        if (last_capture_tick != 0) {
            // --- 1. Calculate delta ticks considering overflow ---
            uint32_t dt_ticks;
            if (current_capture_tick >= last_capture_tick) {
                dt_ticks = current_capture_tick - last_capture_tick;
            } else {
                // Timer overflow
                dt_ticks = (0xFFFF - last_capture_tick) + current_capture_tick + 1;
            }

            // --- 2. Detect Missing Tooth for synchronization ---
            if (!engine_is_synced && last_dt_ticks > 0) {
                if ((float)dt_ticks > 1.5f * (float)last_dt_ticks) {
                    engine_is_synced = 1;
                    tooth_counter = 0;
                    // Example: synchronization at 90 degrees before TDC of cylinder 1
                    g_engineState.engine_angle_deg = 360.0f - 90.0f;
                }
            }

            // --- 3. Calculate RPM ---
            float time_per_tooth_s = (float)dt_ticks / (float)TIMER_CLOCK_FREQ_HZ;
            if (time_per_tooth_s > 0.0f) {
                g_engineState.rpm = (uint32_t)((60.0f / time_per_tooth_s) / g_ecuConfig.ckp.num_teeth_on_wheel);
            }

            last_dt_ticks = dt_ticks;

            // --- 4. Update engine angle ---
            if (engine_is_synced) {
                float degrees_per_tooth = 360.0f / g_ecuConfig.ckp.num_teeth_on_wheel;
                g_engineState.engine_angle_deg += degrees_per_tooth;
                tooth_counter++;

                // Keep angle within 0-720 degrees
                if (g_engineState.engine_angle_deg >= 720.0f) {
                    g_engineState.engine_angle_deg -= 720.0f;
                }
            }

            // --- 5. Call callback for control task ---
            EngineControl_CrankEventCallback();
        }

        last_capture_tick = current_capture_tick;
    }
}