/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
extern "C" {
#include "main.h"
#include "fdcan.h"
#include "spi.h"
#include "tim.h"
#include "usb_device.h"
#include "gpio.h"
}

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bootloader_jump.h"
#include "RobotConfig.h"
#include "UsbCdc.h"
#include "CanBus.h"
#include "CanDebug.h"
#include "AS5047P.h"
#include "StepperDriver.h"
#include "TMC5160.h"
#include "TMC2209.h"
#include "Encoder.h"
#include "StepCounter.h"
#include "PostionController.h"
#include "CanHandler.h"
#include "Io.h"
#include "SmartInput.h"
#include "SmartOutput.h"
#include "HomingSensor.h"
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

/* USER CODE BEGIN PV */
uint8_t ownNodeId = OWN_NODE_ID;

UsbCdc usb;
CanBus can(hfdcan1);
AS5047P as5047p(&hspi2, GPIOB, GPIO_PIN_12);

SmartInput toolFun(GPIO_Input_PF0_GPIO_Port, GPIO_Input_PF0_Pin, can, ownNodeId, 1, false);
SmartInput toolCls(GPIO_Input_PF1_GPIO_Port, GPIO_Input_PF1_Pin, can, ownNodeId, 2, false);
SmartInput toolOpn(GPIO_Input_PA0_GPIO_Port, GPIO_Input_PA0_Pin, can, ownNodeId, 3, false);
SmartOutput toolBuz(GPIO_Output_PA1_GPIO_Port, GPIO_Output_PA1_Pin);

#ifdef DRIVER_USE_TMC2209
StepperDriver* driver = new TMC2209(
  GPIO_Output_EN_GPIO_Port, GPIO_Output_EN_Pin,
  GPIO_Output_DIR_GPIO_Port, GPIO_Output_DIR_Pin,
  GPIOA, GPIO_PIN_8,
  GPIOA, GPIO_PIN_7,
  GPIOA, GPIO_PIN_5,
  GPIOA, GPIO_PIN_4,
  GPIOA, GPIO_PIN_6
);
#else
StepperDriver* driver = new TMC5160(
  &hspi1,
  GPIO_Output_CS1_GPIO_Port, GPIO_Output_CS1_Pin,
  GPIO_Output_EN_GPIO_Port, GPIO_Output_EN_Pin,
  GPIO_Output_DIR_GPIO_Port, GPIO_Output_DIR_Pin,
  GPIOA, GPIO_PIN_8
);
#endif
Encoder encoder(as5047p, ACTIVE_CONFIG.enc_gr, ACTIVE_CONFIG.home_offset);
PositionController controller(
  ACTIVE_CONFIG.kp,
  ACTIVE_CONFIG.ki,
  ACTIVE_CONFIG.kd,
  ACTIVE_CONFIG.kv,
  ACTIVE_CONFIG.mot_gr,
  0,
  ACTIVE_CONFIG.guiding_gain,
  ACTIVE_CONFIG.guiding_deadzone,
  ACTIVE_CONFIG.error_limit
);
StepCounter counter(&htim2);
HomingSensor homingSensor(GPIO_Input_DI0_GPIO_Port, GPIO_Input_DI0_Pin, encoder, can, ownNodeId, 10, false);

uint32_t controller_time = 0;
uint32_t controller_duration = 0;
uint32_t can_duration = 0;
AS5047PStatus as5047p_status;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
#ifdef __cplusplus
extern "C" {
#endif

void SystemClock_Config(void);
void DWT_Init(void);
uint32_t micros(void);
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs);

#ifdef __cplusplus
}
#endif
/* USER CODE BEGIN PFP */

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
  Bootloader_CheckAndJump();
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
  DWT_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_FDCAN1_Init();
  MX_USB_Device_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_TIM1_Init(); // Step output
  MX_TIM2_Init(); // Step event
  MX_TIM3_Init(); // PositionController

  /* USER CODE BEGIN 2 */
  //Bootloader_InitOptionBytes();
  // Panic Bootloader, pull pin high while restart
  if (HAL_GPIO_ReadPin(GPIO_Input_CBOOT_GPIO_Port, GPIO_Input_CBOOT_Pin) == GPIO_PIN_SET) {
    Bootloader_TriggerDFU();
  }

  HAL_Delay(100);
  HAL_TIM_Base_Start(&htim2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_Base_Start_IT(&htim3);

  can.configureCanFilters(&hfdcan1, ownNodeId);
  HAL_FDCAN_Start(&hfdcan1);
  HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
  
  counter.begin();
  driver->begin();
  driver->enable(false);
  driver->setVelocity(0);
  
  red_blink_routine(1, 200);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {  
    can_duration = processIncomingCAN();

    static uint32_t last_usb_debug = 0;
    if (HAL_GetTick() - last_usb_debug > 1000) {
      last_usb_debug = HAL_GetTick();

      as5047p_status = as5047p.getStatus();
      if (as5047p_status.magnetTooStrong || as5047p_status.magnetTooWeak) {
        HAL_GPIO_WritePin(GPIO_Output_RED_GPIO_Port, GPIO_Output_RED_Pin, GPIO_PIN_SET);
      } else {
        HAL_GPIO_WritePin(GPIO_Output_RED_GPIO_Port, GPIO_Output_RED_Pin, GPIO_PIN_RESET);
      }

      uint32_t arr_32 = (uint32_t)(driver->get_arr() > 0xFFFFFFFF ? 0xFFFFFFFF : driver->get_arr());
      uint32_t tmp_32 = (uint32_t)(driver->get_tmp() > 0xFFFFFFFF ? 0xFFFFFFFF : driver->get_tmp());

      static char usb_debug_buf[256];
      snprintf(usb_debug_buf, sizeof(usb_debug_buf),
        "{"
        "\"time_us\":%lu,"
        "\"pos\":%ld,"
        "\"arr\":%lu,"
        "\"psc\":%lu,"
        "\"vel_usb\":%ld,"
        "\"pid_us\":%ld,"
        "\"can_us\":%lu,"
        "\"hState\":%d,"
        "\"NO\":%d"
        "}\n",
        controller_time,
        encoder.getPosition(),
        arr_32,
        tmp_32,
        usb.get_vel(),
        controller_duration,
        can_duration,
        (uint32_t)homingSensor.getState(),
        HAL_GPIO_ReadPin(GPIO_Input_DI0_GPIO_Port, GPIO_Input_DI0_Pin) == GPIO_PIN_SET
      );
      usb.send(usb_debug_buf);

      HAL_GPIO_TogglePin(GPIO_Output_BLU_GPIO_Port, GPIO_Output_BLU_Pin);
    }

    static uint32_t last_button = 0;
    if (HAL_GetTick() - last_button > 10) {
      last_button = HAL_GetTick();
      toolFun.update();
      toolCls.update();
      toolOpn.update();
      toolBuz.update();
    }

    if (HAL_GetTick() > 600000) { Bootloader_TriggerDFU(); }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 12;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV4;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void DWT_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; // Enable DWT
    DWT->CYCCNT = 0;                                // Reset Counter
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;            // Start Counter
}

uint32_t micros(void) {
  return DWT->CYCCNT / (SystemCoreClock / 1000000);
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if (RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE)
    {
        FDCAN_RxHeaderTypeDef rxHeader;
        uint8_t rxData[64];

        HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rxHeader, rxData);

        can.onRxInterrupt(rxHeader, rxData);
    }
}

extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    // Ensure that the interrupt comes from 1-millisecond controller timer (TIM3)
    if (htim->Instance == TIM3)
    {
      controller_time = micros();
      
      // 2. ENCODER UPDATE (Must run every 1 ms for accurate speed calculation)
      encoder.update(1); // dt = 1ms
      counter.update(driver->getDirectionPinState(), driver->getEnabled());
      int32_t encoder_position = encoder.getPosition();
      homingSensor.update();
      //int32_t counter_position = counter.getPosition();
      
      // 3. CALCULATE CONTROL ALGORITHM (Interpolator + PID + Feed-Forward)
      int64_t calc_speed = controller.update(encoder_position);

      // 4. SET THE DRIVER TO THE NEW SPEED (mdeg/s)
      //driver->enable(usb.get_enable());
      //driver->setVelocity(usb.get_vel());
      driver->setVelocity(calc_speed);

      controller_duration = (micros() - controller_time);
    }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
    usb.send("Error happened");
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
