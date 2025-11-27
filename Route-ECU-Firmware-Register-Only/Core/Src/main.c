/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx.h"  // For direct register access on STM32H7xx
#include "cmsis_os.h"
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ecu_config.h"
#include "crank_decoder.h"
#include "ignition_driver.h"
#include "injector_driver.h"
#include "engine_sensors.h"
#include "engine_control.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "ff.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
// Global FATFS structure for SD card
FATFS SDFatFS;
// Global file structure for SD card
FIL SDFile;
// Path for SD card
char SDPath[4];

// Global ECU configuration
EcuConfig_t  g_ecuConfig;
// Global engine data
EngineData_t g_engineData;
// Global engine state
EngineState_t g_engineState;

// Timer clock frequency constant
const long TIMER_CLOCK_FREQ_HZ = 1000000UL;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
// Function to configure system clock
void SystemClock_Config(void);
// Function to configure peripheral common clock
void PeriphCommonClock_Config(void);
// Function to configure MPU
static void MPU_Config(void);
// Function to initialize GPIO
static void MX_GPIO_Init(void);
// Function to initialize FDCAN1
static void MX_FDCAN1_Init(void);
// Function to initialize I2C1
static void MX_I2C1_Init(void);
// Function to initialize I2C2
static void MX_I2C2_Init(void);
// Function to initialize IWDG1
static void MX_IWDG1_Init(void);
// Function to initialize SDMMC1
static void MX_SDMMC1_MMC_Init(void);
// Function to initialize SPI1
static void MX_SPI1_Init(void);
// Function to initialize SPI2
static void MX_SPI2_Init(void);
// Function to initialize UART7
static void MX_UART7_Init(void);
// Function to initialize USART1
static void MX_USART1_UART_Init(void);
// Function to initialize USART3
static void MX_USART3_UART_Init(void);
// Function to initialize WWDG1
static void MX_WWDG1_Init(void);
// Function to initialize TIM1
static void MX_TIM1_Init(void);
// Function to initialize TIM3
static void MX_TIM3_Init(void);
// Function to initialize TIM4
static void MX_TIM4_Init(void);
// Function to initialize ADC1
static void MX_ADC1_Init(void);
// Function to initialize ADC2
static void MX_ADC2_Init(void);
// Function to initialize ADC3
static void MX_ADC3_Init(void);
// Function to initialize TIM2
static void MX_TIM2_Init(void);
// Function to initialize TIM8
static void MX_TIM8_Init(void);
// Function to initialize TIM5
static void MX_TIM5_Init(void);
// Default task function
void StartDefaultTask(void const * argument);

/* USER CODE BEGIN PFP */
// Function to setup application
void setup();
// Function for main loop
void loop();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* MCU Configuration--------------------------------------------------------*/

  // Reset all peripherals, initialize Flash and Systick
  // Note: HAL_Init() not used, manual init
  SysTick->LOAD = SystemCoreClock / 1000 - 1;  // 1ms tick
  SysTick->VAL = 0;
  SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk;

  // Configure system clock
  SystemClock_Config();

  // Configure peripheral common clock
  PeriphCommonClock_Config();

  // Initialize GPIO
  MX_GPIO_Init();
  // Initialize FDCAN1 (register level)
  MX_FDCAN1_Init();
  // Initialize I2C1
  MX_I2C1_Init();
  // Initialize I2C2
  MX_I2C2_Init();
  // Initialize IWDG1
  MX_IWDG1_Init();
  // Initialize SDMMC1
  MX_SDMMC1_MMC_Init();
  // Initialize SPI1
  MX_SPI1_Init();
  // Initialize SPI2
  MX_SPI2_Init();
  // Initialize UART7
  MX_UART7_Init();
  // Initialize USART1
  MX_USART1_UART_Init();
  // Initialize USART3
  MX_USART3_UART_Init();
  // Initialize WWDG1
  MX_WWDG1_Init();
  // Initialize TIM1 (register level)
  MX_TIM1_Init();
  // Initialize TIM3
  MX_TIM3_Init();
  // Initialize TIM4
  MX_TIM4_Init();
  // Initialize ADC1
  MX_ADC1_Init();
  // Initialize ADC2
  MX_ADC2_Init();
  // Initialize ADC3
  MX_ADC3_Init();
  // Initialize TIM2
  MX_TIM2_Init();
  // Initialize TIM8
  MX_TIM8_Init();
  // Initialize TIM5
  MX_TIM5_Init();

  // Call setup function
  setup();

  // Start RTOS scheduler
  osKernelStart();

  // Main loop (RTOS running, but kept for compatibility)
  for(;;) {
    loop();
  }
}

// Function to load default configuration
void LoadDefaultConfig(void) {
    // Set config version
    g_ecuConfig.config_version = 1;
    // Set number of cylinders
    g_ecuConfig.num_cylinders = 4;
    // Set displacement in cc
    g_ecuConfig.displacement_cc = 2000;
    // Set injector flow in g/s
    g_ecuConfig.injector_flow_gps = 0.05f;
    // Set stoichiometric AFR
    g_ecuConfig.afr_stoich = 14.7f;
    // Set CKP teeth per revolution
    g_ecuConfig.ckp.teeth_per_rev = 60;
    // Set TDC angle after gap
    g_ecuConfig.ckp.tdc_angle_after_gap_deg = 114.0f;

    // TODO: Fill the rest of tables and parameters with default values.
}

// Function to setup application
void setup() {
	// --- Load Configuration ---
	// Mount SD card
	if (f_mount(&SDFatFS, (TCHAR const*) SDPath, 1) == FR_OK) {
		// Try to open config file "config.bin"
		if (f_open(&SDFile, "config.bin", FA_READ) == FR_OK) {
			UINT bytesRead;
			// Read file into global config structure
			f_read(&SDFile, &g_ecuConfig, sizeof(EcuConfig_t), &bytesRead);
			f_close(&SDFile);

			// Simple config validation
			if (bytesRead != sizeof(EcuConfig_t)
					|| g_ecuConfig.config_version != 1) {
				LoadDefaultConfig(); // Load defaults if invalid
			}
		} else {
			LoadDefaultConfig(); // Load defaults if file not found
		}
	} else {
		LoadDefaultConfig(); // Load defaults if SD fails
	}

	// --- Initialize Firmware Modules ---
	EngineSensors_Init();      // Initialize ADC+DMA for sensors
	Crank_Init();           // Initialize capture timer for CKP (VR or Hall)
	Ignition_Init();           // Initialize PWM timers for ignition
	Injector_Init();           // Initialize PWM timers for injection
	//Comm_Init(&huart1);        // Initialize UART communication with PC (commented in original)
	EngineControl_Init();      // Initialize control module (create semaphore)
}

// Function for main loop
void loop() {
/*
 * // Command to test an injector...
uint8_t inj_index = 2; // Extracted from command
float pw_ms = 5.0;     // Extracted from command
Injector_TestPulse(inj_index, pw_ms);

// Command to test ignition...
uint8_t ign_index = 0; // Extracted from command
float dwell_ms = 3.2;  // Extracted from command
Ignition_TestPulse(ign_index, dwell_ms);
 */
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
	/* Infinite loop */
	for (;;) {
		osDelay(1);
	}
  /* USER CODE END 5 */
}

 /* MPU Configuration */

// Function to configure MPU
void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  // Disable MPU
  MPU->CTRL &= ~MPU_CTRL_ENABLE_Msk;

  // Initialize and configure MPU region
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  // Set MPU region (manual, since no HAL)
  MPU->RNR = MPU_InitStruct.Number;
  MPU->RBAR = MPU_InitStruct.BaseAddress;
  MPU->RASR = ((uint32_t)MPU_InitStruct.DisableExec << MPU_RASR_XN_Pos) |
              ((uint32_t)MPU_InitStruct.AccessPermission << MPU_RASR_AP_Pos) |
              ((uint32_t)MPU_InitStruct.TypeExtField << MPU_RASR_TEX_Pos) |
              ((uint32_t)MPU_InitStruct.IsShareable << MPU_RASR_S_Pos) |
              ((uint32_t)MPU_InitStruct.IsCacheable << MPU_RASR_C_Pos) |
              ((uint32_t)MPU_InitStruct.IsBufferable << MPU_RASR_B_Pos) |
              ((uint32_t)MPU_InitStruct.SubRegionDisable << MPU_RASR_SRD_Pos) |
              ((uint32_t)MPU_InitStruct.Size << MPU_RASR_SIZE_Pos) |
              MPU_RASR_ENABLE_Msk;

  // Enable MPU
  MPU->CTRL |= MPU_CTRL_PRIVDEFENA_Msk | MPU_CTRL_ENABLE_Msk;
}

// Function to configure system clock (manual register)
void SystemClock_Config(void) {
  // Enable HSE
  RCC->CR |= RCC_CR_HSEON;
  // Wait for HSE ready
  while (!(RCC->CR & RCC_CR_HSERDY));

  // Configure PLL (example for H7, adjust as needed)
  RCC->PLLCKSELR = (RCC_PLLCKSELR_DIVM1_Msk & (4 << RCC_PLLCKSELR_DIVM1_Pos)) | RCC_PLLSOURCE_HSE;
  RCC->PLLCFGR = RCC_PLLCFGR_PLL1RGE_2 | RCC_PLLCFGR_PLL1VCOSEL;
  RCC->PLL1DIVR = (RCC_PLL1DIVR_N1_Msk & (200 << RCC_PLL1DIVR_N1_Pos)) | (RCC_PLL1DIVR_P1_Msk & (2 << RCC_PLL1DIVR_P1_Pos));
  RCC->CR |= RCC_CR_PLL1ON;
  // Wait for PLL ready
  while (!(RCC->CR & RCC_CR_PLL1RDY));

  // Set system clock to PLL
  RCC->CFGR |= RCC_CFGR_SW_PLL1;
  // Wait for switch
  while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL1);
}

// Function to configure peripheral common clock (manual)
void PeriphCommonClock_Config(void) {
  // Example: Set periph clock to HSE (adjust as needed)
  RCC->D1CCIPR = RCC_D1CCIPR_CKPERSEL_HSE;
}

// Function to initialize GPIO (manual)
static void MX_GPIO_Init(void) {
  // Enable GPIO clocks
  RCC->AHB4ENR |= RCC_AHB4ENR_GPIOAEN | RCC_AHB4ENR_GPIOBEN | RCC_AHB4ENR_GPIOCEN; // Add as needed
  // Configure pins (example, add specific configs)
  GPIOA->MODER = 0; // Reset, then set as needed
}

// Similar manual init for other peripherals (abbreviated for brevity)
static void MX_FDCAN1_Init(void) {
  // Enable FDCAN clock
  RCC->APB1HENR |= RCC_APB1HENR_FDCANEN;
  // Configure FDCAN registers as needed
}

static void MX_I2C1_Init(void) {
  // Enable I2C1 clock
  RCC->APB1LENR |= RCC_APB1LENR_I2C1EN;
  // Configure I2C1 registers
}

static void MX_I2C2_Init(void) {
  // Enable I2C2 clock
  RCC->APB1LENR |= RCC_APB1LENR_I2C2EN;
  // Configure I2C2 registers
}

static void MX_IWDG1_Init(void) {
  // Configure IWDG registers
  IWDG1->KR = 0xCCCC; // Start IWDG
  IWDG1->PR = 3; // Prescaler
  IWDG1->RLR = 4095; // Reload
}

static void MX_SDMMC1_MMC_Init(void) {
  // Enable SDMMC clock
  RCC->AHB3ENR |= RCC_AHB3ENR_SDMMC1EN;
  // Configure SDMMC registers
}

static void MX_SPI1_Init(void) {
  // Enable SPI1 clock
  RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
  // Configure SPI1 registers
}

static void MX_SPI2_Init(void) {
  // Enable SPI2 clock
  RCC->APB1LENR |= RCC_APB1LENR_SPI2EN;
  // Configure SPI2 registers
}

static void MX_UART7_Init(void) {
  // Enable UART7 clock
  RCC->APB1LENR |= RCC_APB1LENR_UART7EN;
  // Configure UART7 registers
}

static void MX_USART1_UART_Init(void) {
  // Enable USART1 clock
  RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
  // Configure USART1 registers
}

static void MX_USART3_UART_Init(void) {
  // Enable USART3 clock
  RCC->APB1LENR |= RCC_APB1LENR_USART3EN;
  // Configure USART3 registers
}

static void MX_WWDG1_Init(void) {
  // Configure WWDG registers
  WWDG1->CFR = 0x7F; // Window
  WWDG1->CR = 0xFF; // Downcounter
  WWDG1->CR |= WWDG_CR_WDGA; // Activate
}

static void MX_TIM1_Init(void) {
  // Enable TIM1 clock
  RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
  // Configure TIM1 as PWM (example)
  TIM1->PSC = 0; // Prescaler
  TIM1->ARR = 0xFFFF; // Auto-reload
  TIM1->CCMR1 = TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1; // PWM mode for CH1, CH2
  TIM1->CCMR2 = TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4M_1; // PWM mode for CH3, CH4
  TIM1->BDTR |= TIM_BDTR_MOE; // Enable outputs
}

// Similar for other TIMx
static void MX_TIM3_Init(void) {
  RCC->APB1LENR |= RCC_APB1LENR_TIM3EN;
  // Configure as needed
}

static void MX_TIM4_Init(void) {
  RCC->APB1LENR |= RCC_APB1LENR_TIM4EN;
  // Configure as PWM for ignition
  TIM4->PSC = 0;
  TIM4->ARR = 0xFFFF;
  TIM4->CCMR1 = TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1;
  TIM4->CCMR2 = TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4M_1;
  TIM4->BDTR |= TIM_BDTR_MOE;
}

static void MX_ADC1_Init(void) {
  // Enable ADC1 clock
  RCC->AHB1ENR |= RCC_AHB1ENR_ADC12EN;
  // Configure ADC1 (basic, no DMA here, assume external config)
  ADC1->CFGR = ADC_CFGR_CONT | ADC_CFGR_RES_1; // Continuous, 14-bit res example
  ADC1->SQR1 = 0; // Sequence
  ADC1->CR |= ADC_CR_ADEN; // Enable
  while (!(ADC1->ISR & ADC_ISR_ADRDY)); // Wait ready
}

static void MX_ADC2_Init(void) {
  // Configure ADC2 similar to ADC1
  ADC2->CFGR = ADC_CFGR_CONT | ADC_CFGR_RES_1;
  ADC2->SQR1 = 0;
  ADC2->CR |= ADC_CR_ADEN;
  while (!(ADC2->ISR & ADC_ISR_ADRDY));
}

static void MX_ADC3_Init(void) {
  // Enable ADC3 clock
  RCC->AHB4ENR |= RCC_AHB4ENR_ADC3EN;
  // Configure ADC3
  ADC3->CFGR = ADC_CFGR_CONT | ADC_CFGR_RES_1;
  ADC3->SQR1 = 0;
  ADC3->CR |= ADC_CR_ADEN;
  while (!(ADC3->ISR & ADC_ISR_ADRDY));
}

static void MX_TIM2_Init(void) {
  RCC->APB1LENR |= RCC_APB1LENR_TIM2EN;
  // Configure as input capture for crank
  TIM2->PSC = 0;
  TIM2->ARR = 0xFFFFFFFF;
  TIM2->CCMR1 = TIM_CCMR1_CC1S_0; // TI1 input
  TIM2->SMCR = 0; // No slave
}

static void MX_TIM8_Init(void) {
  RCC->APB2ENR |= RCC_APB2ENR_TIM8EN;
  // Configure as needed
}

static void MX_TIM5_Init(void) {
  RCC->APB1LENR |= RCC_APB1LENR_TIM5EN;
  // Configure as needed
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
	}
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */