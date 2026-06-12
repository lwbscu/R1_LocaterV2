/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : app_freertos.c
  * Description        : FreeRTOS application tasks for R1_LocaterV2.
  ******************************************************************************
  */
/* USER CODE END Header */

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */
extern void StartLocaterTask(void *argument);
extern void StartTelemetryTask(void *argument);
/* USER CODE END Includes */

osThreadId_t TaskLocaterHandle;
const osThreadAttr_t TaskLocater_attributes = {
        .name = "TaskLocater",
        .stack_size = 768 * 4,
        .priority = (osPriority_t) osPriorityNormal,
};

osThreadId_t TaskTelemetryHandle;
const osThreadAttr_t TaskTelemetry_attributes = {
        .name = "TaskTelemetry",
        .stack_size = 768 * 4,
        .priority = (osPriority_t) osPriorityLow,
};

void MX_FREERTOS_Init(void)
{
    TaskLocaterHandle = osThreadNew(StartLocaterTask, NULL, &TaskLocater_attributes);
    TaskTelemetryHandle = osThreadNew(StartTelemetryTask, NULL, &TaskTelemetry_attributes);
}
