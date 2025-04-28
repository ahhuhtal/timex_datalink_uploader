/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "cmsis_os.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include "usbd_cdc_if.h"
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

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for dataQueue */
osMessageQueueId_t dataQueueHandle;
const osMessageQueueAttr_t dataQueue_attributes = {
  .name = "dataQueue"
};
/* Definitions for uartQueue */
osMessageQueueId_t uartQueueHandle;
const osMessageQueueAttr_t uartQueue_attributes = {
  .name = "uartQueue"
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM4_Init(void);
void StartDefaultTask(void *argument);

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

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of dataQueue */
  dataQueueHandle = osMessageQueueNew (64, sizeof(uint16_t), &dataQueue_attributes);

  /* creation of uartQueue */
  uartQueueHandle = osMessageQueueNew (8192, sizeof(uint8_t), &uartQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */

  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
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
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_1);
  while(LL_FLASH_GetLatency()!= LL_FLASH_LATENCY_1)
  {
  }
  LL_RCC_HSE_Enable();

   /* Wait till HSE is ready */
  while(LL_RCC_HSE_IsReady() != 1)
  {

  }
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE_DIV_1, LL_RCC_PLL_MUL_6);
  LL_RCC_PLL_Enable();

   /* Wait till PLL is ready */
  while(LL_RCC_PLL_IsReady() != 1)
  {

  }
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_2);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {

  }
  LL_SetSystemCoreClock(48000000);

   /* Update the time base */
  if (HAL_InitTick (TICK_INT_PRIORITY) != HAL_OK)
  {
    Error_Handler();
  }
  LL_RCC_SetUSBClockSource(LL_RCC_USB_CLKSOURCE_PLL);
}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  LL_TIM_InitTypeDef TIM_InitStruct = {0};
  LL_TIM_OC_InitTypeDef TIM_OC_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM4);

  /* TIM4 interrupt Init */
  NVIC_SetPriority(TIM4_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),5, 0));
  NVIC_EnableIRQ(TIM4_IRQn);

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  TIM_InitStruct.Prescaler = 0;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 23520;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  LL_TIM_Init(TIM4, &TIM_InitStruct);
  LL_TIM_DisableARRPreload(TIM4);
  LL_TIM_SetClockSource(TIM4, LL_TIM_CLOCKSOURCE_INTERNAL);
  LL_TIM_OC_EnablePreload(TIM4, LL_TIM_CHANNEL_CH4);
  TIM_OC_InitStruct.OCMode = LL_TIM_OCMODE_PWM1;
  TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.CompareValue = 0;
  TIM_OC_InitStruct.OCPolarity = LL_TIM_OCPOLARITY_HIGH;
  LL_TIM_OC_Init(TIM4, LL_TIM_CHANNEL_CH4, &TIM_OC_InitStruct);
  LL_TIM_OC_DisableFast(TIM4, LL_TIM_CHANNEL_CH4);
  LL_TIM_SetTriggerOutput(TIM4, LL_TIM_TRGO_RESET);
  LL_TIM_DisableMasterSlaveMode(TIM4);
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOB);
  /**TIM4 GPIO Configuration
  PB9   ------> TIM4_CH4
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_9;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOD);
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOB);

}

/* USER CODE BEGIN 4 */

// receive packet_length bytes from the UART queue and push them to the data output buffer
// adding in the delay frames required between packets.
// echo determines whether to echo received bytes back or not

bool handle_packet(uint8_t packet_length, bool echo) {
  // data output buffer uses uint16_t instead of uint8_t to encode the idle state (>0xff)

  // send the packet length byte first
  uint16_t uint16_data = packet_length;
  osMessageQueuePut(dataQueueHandle, &uint16_data, 0, portMAX_DELAY);

  if(echo) while(CDC_Transmit_FS(&packet_length, 1) == USBD_BUSY);

  // transmit all following bytes (total bytes = packet_length, which includes the length byte itself)
  for(size_t i=0;i<packet_length-1;i++) {
    uint8_t data;

    status = osMessageQueueGet(uartQueueHandle, &data, NULL, 500);

    if(status == osOK) {
      // we got a byte from the queue. push it to be transmitted.
      uint16_data = data;
      osMessageQueuePut(dataQueueHandle, &uint16_data, 0, portMAX_DELAY);

      if(echo) while(CDC_Transmit_FS(&data, 1) == USBD_BUSY);
    } else {
      // we didn't get a byte in time. this breaks the transmission.

      // return failure
      return false;
    }
  }

  // if the packet was of odd length, push an extra idle byte to make output bytes even
  if(packet_first_byte%2==1) {
    uint16_t dummy = 0xffff;
    osMessageQueuePut(dataQueueHandle, &dummy, 0, portMAX_DELAY);
  }

  // push the end of packet delay as idle bytes at end of packet
  for(size_t i=0;i<22;i++) {
    uint16_t delay = 0xffff;
    osMessageQueuePut(dataQueueHandle, &delay, 0, portMAX_DELAY);
  }

  return true;
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 5 */

  LL_TIM_CC_EnableChannel(TIM4, LL_TIM_CHANNEL_CH4);
  LL_TIM_EnableIT_UPDATE(TIM4);

  LL_TIM_EnableCounter(TIM4);

  // wait for a while for USB to get initialized etc.
  osDelay(1000);

  // are we in stream mode?
  bool stream_started = 0;

  // are we emulating a notebook adapter?
  bool notebook_adapter_mode = 0;

  /* Infinite loop */
  for(;;) {
    // we can operate in a few different modes:
    // either we are in the middle of a data stream, or we are not
    // a data stream is the part of a transmission where we are getting a continuous stream of packets

    // if we are in a data stream, then things are clear:
    // we receive each packet separately and transmit it to the watch.
    // a packet starts with a byte describing the length of the packet

    // if we are not in a data stream, then
    // 1. either we have not received anything, or
    // 2. we have received data related to the notebook adapter handshake, but no actual data yet

    // if we have received notebook adapter handshake, then we echo all data back as is expected
    // if we have not received the notebook adapter handshake when going to data stream, we will not echo

    // not echoing the data allows a simple "cat data > /dev/ttyACM0" without the tty blocking due to full receive buffers.

    // wait for data on uart
    uint8_t byte;
    osStatus_t status = osMessageQueueGet(uartQueueHandle, &byte, NULL, 500);

    // did we get a byte, or did we time out?
    if(status == osOK) {
      // we got something. are we handling a stream yet?
      if(!stream_started) {
        // we have not yet started a data stream, so either this was the first byte ever
        // or we are in notebook adapter handshake

        // are we in notebook adapter mode?
        if(!notebook_adapter_mode) {
          // no. did we just receive the notebook handshake byte?
          if(byte == 'x') {
            // we received the notebook adapter initial handshake.
            // we go to notebook adapter mode
            notebook_adapter_mode = true;

            // echo back the byte
            while(CDC_Transmit_FS(&byte, 1) == USBD_BUSY);
          } else {
            // we did not receive handshake.
            // this is the beginning of a raw data stream.
            stream_started = true;
          }
        } else {
          // we are in notebook adapter mode, but stream has not been started
          if(byte=='?') {
            // we received the identification request. respond to that with the ID string.
            char* ident_string = "126\r M764 rev 764002";
            while(CDC_Transmit_FS((uint8_t*)ident_string, strlen(ident_string)+1) == USBD_BUSY);
          } else if(byte=='x' || byte==0x55 || byte==0xaa) {
            // we received either the handshake byte, a sync byte of start of transmission byte
            // sync bytes and start of transmissions bytes are handled internally
            // we're yet to start the stream, but need to echo these bytes
            while(CDC_Transmit_FS(&byte, 1) == USBD_BUSY);
          } else {
            //
          }
        }

        // did we start a stream just now?
        if(stream_started) {
          // transmit the start of transmission bytes.
          for(size_t i=0;i<50;i++) {
            uint16_t start = 0xaa;
            osMessageQueuePut(dataQueueHandle, &start, 0, portMAX_DELAY);
          }

          // handle the first packet
          handle_packet(byte, notebook_adapter_mode);
        }
      } else {
        // stream is ongoing. handle next packet
        handle_packet(byte, notebook_adapter_mode);
      }

    } else {
      // we timed out receiving a byte. reset the stream state and notebook adapter mode
      stream_started = 0;
      notebook_adapter_mode = 0;

      // generate sync bytes to be transmitted
      for(size_t i=0;i<64;i++) {
        uint16_t delay = 0x55;
        osMessageQueuePut(dataQueueHandle, &delay, 0, portMAX_DELAY);
      }
    }
  }
  /* USER CODE END 5 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
