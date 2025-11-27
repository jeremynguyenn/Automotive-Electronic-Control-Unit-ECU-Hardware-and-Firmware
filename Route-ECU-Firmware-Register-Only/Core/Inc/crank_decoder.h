/*
 * crank_decoder.h
 *
 * Created on: Aug 20, 2025
 */

#ifndef INC_CRANK_DECODER_H_
#define INC_CRANK_DECODER_H_

#include "stm32h7xx.h"  // For direct register access on STM32H7xx
#include "main.h"
#include "ecu_config.h"
#include "engine_control.h"
#include <math.h>

// External constant for timer clock frequency
extern const long TIMER_CLOCK_FREQ_HZ;

// External variables for ECU config, engine data, and state
extern EcuConfig_t  g_ecuConfig;
extern EngineData_t g_engineData;
extern EngineState_t g_engineState;

// Private variables for crank decoder
static volatile uint32_t last_capture_tick = 0;
static volatile uint32_t last_dt_ticks = 0;
static volatile int engine_is_synced = 0;
static volatile uint16_t tooth_counter = 0;

// Function to initialize crank decoder
void Crank_Init(void);

#endif /* INC_CRANK_DECODER_H_ */