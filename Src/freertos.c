/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */     
#include "cmsis_os2.h"
#include "command.h"
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
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
typedef StaticQueue_t osStaticMessageQDef_t;
osThreadId_t commandTaskHandle;
osThreadId_t AlgsTaskHandle;
osThreadId_t RawSendTaskHandle;
osMessageQueueId_t portRawDataRxHandle;
uint8_t portRawDataRxBuffer[ 2048 * sizeof( uint8_t ) ];
osStaticMessageQDef_t portRawDataRxControlBlock;
osMessageQueueId_t cmdPacketRxHandle;
uint8_t cmdPacketRxBuffer[ 16 * 64 ];
osStaticMessageQDef_t cmdPacketRxControlBlock;
osMessageQueueId_t cmdPacketTxHandle;
uint8_t cmdPacketTxBuffer[ 16 * 64 ];
osStaticMessageQDef_t cmdPacketTxControlBlock;
osMessageQueueId_t portRawDataTxHandle;
uint8_t portRawDataTxBuffer[ 2048 * sizeof( uint8_t ) ];
osStaticMessageQDef_t portRawDataTxControlBlock;
osMessageQueueId_t cmdPacketRxInstantHandle;
uint8_t cmdPacketRxInstantBuffer[ 16 * 64 ];
osStaticMessageQDef_t cmdPacketRxInstantControlBlock;
osMutexId_t portRawDataMutexHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
   
/* USER CODE END FunctionPrototypes */

void StartCommandTask(void *argument);
void StartAlgsTask(void *argument);
void StartRawSendTask(void *argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void vApplicationIdleHook(void);

/* USER CODE BEGIN 2 */
void vApplicationIdleHook( void )
{
   /* vApplicationIdleHook() will only be called if configUSE_IDLE_HOOK is set
   to 1 in FreeRTOSConfig.h. It will be called on each iteration of the idle
   task. It is essential that code added to this hook function never attempts
   to block in any way (for example, call xQueueReceive() with a block time
   specified, or call vTaskDelay()). If the application makes use of the
   vTaskDelete() API function (as this demo application does) then it is also
   important that vApplicationIdleHook() is permitted to return to its calling
   function, because it is the responsibility of the idle task to clean up
   memory allocated by the kernel to any task that has since been deleted. */
}
/* USER CODE END 2 */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
       
  /* USER CODE END Init */
osKernelInitialize();

  /* Create the mutex(es) */
  /* definition and creation of portRawDataMutex */
  const osMutexAttr_t portRawDataMutex_attributes = {
    .name = "portRawDataMutex"
  };
  portRawDataMutexHandle = osMutexNew(&portRawDataMutex_attributes);

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
  /* definition and creation of portRawDataRx */
  const osMessageQueueAttr_t portRawDataRx_attributes = {
    .name = "portRawDataRx",
    .cb_mem = &portRawDataRxControlBlock,
    .cb_size = sizeof(portRawDataRxControlBlock),
    .mq_mem = &portRawDataRxBuffer,
    .mq_size = sizeof(portRawDataRxBuffer)
  };
  portRawDataRxHandle = osMessageQueueNew (2048, sizeof(uint8_t), &portRawDataRx_attributes);

  /* definition and creation of cmdPacketRx */
  const osMessageQueueAttr_t cmdPacketRx_attributes = {
    .name = "cmdPacketRx",
    .cb_mem = &cmdPacketRxControlBlock,
    .cb_size = sizeof(cmdPacketRxControlBlock),
    .mq_mem = &cmdPacketRxBuffer,
    .mq_size = sizeof(cmdPacketRxBuffer)
  };
  cmdPacketRxHandle = osMessageQueueNew (16, 64, &cmdPacketRx_attributes);

  /* definition and creation of cmdPacketTx */
  const osMessageQueueAttr_t cmdPacketTx_attributes = {
    .name = "cmdPacketTx",
    .cb_mem = &cmdPacketTxControlBlock,
    .cb_size = sizeof(cmdPacketTxControlBlock),
    .mq_mem = &cmdPacketTxBuffer,
    .mq_size = sizeof(cmdPacketTxBuffer)
  };
  cmdPacketTxHandle = osMessageQueueNew (16, 64, &cmdPacketTx_attributes);

  /* definition and creation of portRawDataTx */
  const osMessageQueueAttr_t portRawDataTx_attributes = {
    .name = "portRawDataTx",
    .cb_mem = &portRawDataTxControlBlock,
    .cb_size = sizeof(portRawDataTxControlBlock),
    .mq_mem = &portRawDataTxBuffer,
    .mq_size = sizeof(portRawDataTxBuffer)
  };
  portRawDataTxHandle = osMessageQueueNew (2048, sizeof(uint8_t), &portRawDataTx_attributes);

  /* definition and creation of cmdPacketRxInstant */
  const osMessageQueueAttr_t cmdPacketRxInstant_attributes = {
    .name = "cmdPacketRxInstant",
    .cb_mem = &cmdPacketRxInstantControlBlock,
    .cb_size = sizeof(cmdPacketRxInstantControlBlock),
    .mq_mem = &cmdPacketRxInstantBuffer,
    .mq_size = sizeof(cmdPacketRxInstantBuffer)
  };
  cmdPacketRxInstantHandle = osMessageQueueNew (16, 64, &cmdPacketRxInstant_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of commandTask */
  const osThreadAttr_t commandTask_attributes = {
    .name = "commandTask",
    .priority = (osPriority_t) osPriorityNormal,
    .stack_size = 256
  };
  commandTaskHandle = osThreadNew(StartCommandTask, NULL, &commandTask_attributes);

  /* definition and creation of AlgsTask */
  const osThreadAttr_t AlgsTask_attributes = {
    .name = "AlgsTask",
    .priority = (osPriority_t) osPriorityRealtime,
    .stack_size = 512
  };
  AlgsTaskHandle = osThreadNew(StartAlgsTask, NULL, &AlgsTask_attributes);

  /* definition and creation of RawSendTask */
  const osThreadAttr_t RawSendTask_attributes = {
    .name = "RawSendTask",
    .priority = (osPriority_t) osPriorityHigh,
    .stack_size = 512
  };
  RawSendTaskHandle = osThreadNew(StartRawSendTask, NULL, &RawSendTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartCommandTask */
/**
  * @brief  Function implementing the commandTask thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartCommandTask */
void StartCommandTask(void *argument)
{
  /* USER CODE BEGIN StartCommandTask */
  MX_USB_DEVICE_Init();
  osDelay(1000);


  /* Infinite loop */
  for(;;)
  {
    osDelay(20);
    ProtocolReadPacket();
    commandExc();
    ProtocolWritePacket();
  }
  /* USER CODE END StartCommandTask */
}

/* USER CODE BEGIN Header_StartAlgsTask */
/**
* @brief Function implementing the AlgsTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartAlgsTask */
void StartAlgsTask(void *argument)
{
  /* USER CODE BEGIN StartAlgsTask */
  uint8_t ledCount = 0;
  jointParamInit();
  /* Infinite loop */
  for(;;)
  {
    osDelay(10);
    algsProfile();

    hhtKeyCheck();

    ledCount++;
    if(ledCount >= 25)
    {
      ledStatusCheck();
      ledCount = 0;
    }
  }
  /* USER CODE END StartAlgsTask */
}

/* USER CODE BEGIN Header_StartRawSendTask */
/**
* @brief Function implementing the RawSendTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartRawSendTask */
void StartRawSendTask(void *argument)
{
  /* USER CODE BEGIN StartRawSendTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(10);
    protocolRawDataSend();
  }
  /* USER CODE END StartRawSendTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
     
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
