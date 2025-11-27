/*
 * engine_control.h
 *
 * Created on: Aug 19, 2025
 */

#ifndef INC_ENGINE_CONTROL_H_
#define INC_ENGINE_CONTROL_H_

#include "stm32h7xx.h"  // For direct register access on STM32H7xx
#include "main.h"
#include "injector_driver.h"
#include "ignition_driver.h"

// External constant for timer clock frequency
extern const long TIMER_CLOCK_FREQ_HZ;

// Function to initialize engine control
void EngineControl_Init(void);
// RTOS task for engine control
void Task_EngineControl(void const * argument);
// Callback for crank event
void EngineControl_CrankEventCallback(void);

#endif /* INC_ENGINE_CONTROL_H_ */