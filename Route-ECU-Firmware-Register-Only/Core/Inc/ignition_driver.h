/*
 * ignition_driver.h
 *
 * Created on: Aug 20, 2025
 */

#ifndef INC_IGNITION_DRIVER_H_
#define INC_IGNITION_DRIVER_H_

#include "stm32h7xx.h"  // For direct register access on STM32H7xx
#include "main.h"
#include "ecu_config.h"

// External constant for timer clock frequency
extern const long TIMER_CLOCK_FREQ_HZ;

// Mapping of channels for ignition coils
#define IGNITION_COIL_1_CHANNEL TIM_CHANNEL_1
#define IGNITION_COIL_2_CHANNEL TIM_CHANNEL_2
#define IGNITION_COIL_3_CHANNEL TIM_CHANNEL_3
#define IGNITION_COIL_4_CHANNEL TIM_CHANNEL_4

/**
 * @brief Triggers a single test pulse on a specific ignition coil.
 * @note This function is for diagnostic purposes and ignores the main engine logic.
 * @param cylinder_index The index of the coil to test (0 to 3).
 * @param dwell_ms The desired dwell time for the test, in milliseconds.
 */
void Ignition_TestPulse(uint8_t cylinder_index, float dwell_ms);

// Function to initialize the ignition module
void Ignition_Init(void);
// Function to schedule a spark for a cylinder
void Ignition_ScheduleSpark(uint8_t, float, float);
// Function to convert degrees to time in microseconds
uint32_t DegreesToTimeUs(float);
// Function to apply dwell compensation based on voltage
uint32_t ApplyDwellCompensation(float, float);

#endif /* INC_IGNITION_DRIVER_H_ */