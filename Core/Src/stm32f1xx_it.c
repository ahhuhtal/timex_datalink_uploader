/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f1xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "cmsis_os.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
extern osMessageQueueId_t dataQueueHandle;
extern osMessageQueueId_t uartQueueHandle;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void handle_bit_transitions(void) {
  // timings: up to 2 bytes per frame. 59.94 Hz refresh rate.

  // 1 start bit (led ON) + 8 data bits (0=led ON, 1=led OFF)
  // total bit period: 490 us
  // ON period: ~30us

  // 3 dummy bits between bytes (led OFF)

  // 1 start bit (led ON) + 8 data bits (0=led ON, 1=led OFF)
  // total bit period: 490 us
  // ON period: ~30us

  // 13 dummy bits between frames (led OFF)

  // bit period counter. total of 34 bit periods in each frame
  static uint32_t counter = 0;

  // LED control shift register
  // assert the LED when current_data & 0x01 == 1
  // this is built before the start of the frame, and shifted out during the frame
  static uint32_t current_data = 0;

  LL_TIM_ClearFlag_UPDATE(TIM4);
  NVIC_ClearPendingIRQ(TIM4_IRQn);

  if(counter == 34) {
    size_t count = osMessageQueueGetCount(dataQueueHandle);
    if(count>=2) {
      // we have at least two pieces of data in the queue
      // we'll build the next frame
      uint16_t data1;
      uint16_t data2;

      osMessageQueueGet(dataQueueHandle, &data1, NULL, 0);
      osMessageQueueGet(dataQueueHandle, &data2, NULL, 0);

      // start by not asserting the LED at all
      current_data = 0;

      // a value greater than 0xff indicates that nothing is sent in the slot
      if(data1<=0xff) {
        // set the start bit, invert the data bits and move them in position
        data1 = (~data1)&0xff;
        current_data |= (1<<0)|(data1<<1);
      }
      if(data2<=0xff) {
        // set the start bit, invert the data bits and move them in position
        data2 = (~data2)&0xff;
        current_data |= (1<<12)|(data2<<13);
      }
    } else {
      // generate sync frame
      // frame bit 0 == 1, LED on  (start bit == 0)
      // frame bit 1 == 0, LED off (data bit 0 == 1)
      // frame bit 2 == 1, LED on  (data bit 1 == 0)
      // frame bit 3 == 0, LED off (data bit 2 == 1)
      // frame bit 4 == 1, LED on  (data bit 3 == 0)
      // frame bit 5 == 0, LED off (data bit 4 == 1)
      // frame bit 6 == 1, LED on  (data bit 5 == 0)
      // frame bit 7 == 0, LED off (data bit 6 == 1)
      // frame bit 8 == 1, LED on  (data bit 7 == 0)
      // frame bit 9 == 0, LED off (dummy bit == 1)
      // frame bit 10== 0, LED off (dummy bit == 1)
      // frame bit 11== 0, LED off (dummy bit == 1)
      // frame bit 12== 1, LED on  (start bit == 0)
      // frame bit 13== 0, LED off (data bit 0 == 1)
      // frame bit 14== 1, LED on  (data bit 1 == 0)
      // frame bit 15== 0, LED off (data bit 2 == 1)
      // frame bit 16== 1, LED on  (data bit 3 == 0)
      // frame bit 17== 0, LED off (data bit 4 == 1)
      // frame bit 18== 1, LED on  (data bit 5 == 0)
      // frame bit 19== 0, LED off (data bit 6 == 1)
      // frame bit 20== 1, LED on  (data bit 7 == 0)
      // frame bits 21-33, LED off (dummy bits == 1)
      current_data = 0x155155;
    }

    counter=0;
  }

  // values set here for the PWM period are used only after the next update event
  // thus we have 490us time to perform all that is going on in this function

  if(current_data & 0x01) {
    // assert signal for 30 us
    // entire period is 490 us
    LL_TIM_OC_SetCompareCH4(TIM4, 1440);
  } else {
    // keep signal deasserted for entire 490us
    LL_TIM_OC_SetCompareCH4(TIM4, 0);
  }

  current_data=current_data>>1;
  counter++;
}

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern PCD_HandleTypeDef hpcd_USB_FS;
extern TIM_HandleTypeDef htim1;

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M3 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/******************************************************************************/
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f1xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles USB low priority or CAN RX0 interrupts.
  */
void USB_LP_CAN1_RX0_IRQHandler(void)
{
  /* USER CODE BEGIN USB_LP_CAN1_RX0_IRQn 0 */

  /* USER CODE END USB_LP_CAN1_RX0_IRQn 0 */
  HAL_PCD_IRQHandler(&hpcd_USB_FS);
  /* USER CODE BEGIN USB_LP_CAN1_RX0_IRQn 1 */

  /* USER CODE END USB_LP_CAN1_RX0_IRQn 1 */
}

/**
  * @brief This function handles TIM1 update interrupt.
  */
void TIM1_UP_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_UP_IRQn 0 */

  /* USER CODE END TIM1_UP_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_UP_IRQn 1 */

  /* USER CODE END TIM1_UP_IRQn 1 */
}

/**
  * @brief This function handles TIM4 global interrupt.
  */
void TIM4_IRQHandler(void)
{
  /* USER CODE BEGIN TIM4_IRQn 0 */

  handle_bit_transitions();

  /* USER CODE END TIM4_IRQn 0 */
  /* USER CODE BEGIN TIM4_IRQn 1 */

  /* USER CODE END TIM4_IRQn 1 */
}

/* USER CODE BEGIN 1 */


/* USER CODE END 1 */
