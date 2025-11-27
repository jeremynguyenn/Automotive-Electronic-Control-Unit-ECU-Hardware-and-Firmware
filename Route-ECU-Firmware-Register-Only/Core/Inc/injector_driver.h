/*
 * injector_driver.h
 *
 * Created on: 20 de ago de 2025
 */

#ifndef INC_INJECTOR_DRIVER_H_
#define INC_INJECTOR_DRIVER_H_

#include "stm32h7xx.h"  // For direct register access on STM32H7xx
#include "main.h"
#include <stdbool.h>

// External constant for timer clock frequency
extern const long TIMER_CLOCK_FREQ_HZ;

// Mapping of timers/channels for each injector
#define INJECTOR_1_CHANNEL    TIM_CHANNEL_1
#define INJECTOR_2_CHANNEL    TIM_CHANNEL_2
#define INJECTOR_3_CHANNEL    TIM_CHANNEL_3
#define INJECTOR_4_CHANNEL    TIM_CHANNEL_4

/**
 * @brief Triggers a single test pulse on a specific injector.
 * @note This function is for diagnostic purposes and ignores the main engine logic.
 * @param cylinder_index The index of the injector to test (0 to 3).
 * @param pulse_width_ms The desired pulse width for the test, in milliseconds.
 */
void Injector_TestPulse(uint8_t cylinder_index, float pulse_width_ms);

/**
 * @brief Initializes the injection module.
 * @note Configures and starts the hardware timers for all injector channels.
 * Should be called once in the setup function.
 */
void Injector_Init(void);

/**
 * @brief Schedules an injection pulse for a specific cylinder.
 * @param cylinder_index The index of the cylinder to inject (e.g., 0 for cylinder 1).
 * @param pulse_width_ms The total duration the injector should remain open, in milliseconds.
 */
void Injector_SchedulePulse(uint8_t cylinder_index, float pulse_width_ms);

#endif /* INC_INJECTOR_DRIVER_H_ */