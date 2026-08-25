/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "usart.h"
#include "tim.h"
#include "pwm.h"
#include "move.h"
#include "ultrasonic.h"
#include "sensor_data.h"

#include <string.h>
#include <stdio.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef enum
{
  DRIVE_MODE_MANUAL = 0,
  DRIVE_MODE_AUTO
} DriveMode_t;

typedef enum
{
  AUTO_STATE_STOPPED = 0,
  AUTO_STATE_DRIVE,
  AUTO_STATE_STOP_BEFORE_TURN,
  AUTO_STATE_PIVOT_LEFT,
  AUTO_STATE_PIVOT_RIGHT,
  AUTO_STATE_STOP_AFTER_TURN,
  AUTO_STATE_SAFE_STOP
} AutoState_t;

typedef enum
{
  SIDE_AVOID_NONE = 0,
  SIDE_AVOID_LEFT_WALL,
  SIDE_AVOID_RIGHT_WALL
} SideAvoid_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// AUTO 설정값
#define AUTO_FRONT_STOP_CM             30U
//#define AUTO_FRONT_RESUME_CM           40U
#define AUTO_TURN_CLEAR_CM             45U

#define AUTO_CENTER_MAX_AGE_MS        300U
#define AUTO_SIDE_MAX_AGE_MS          500U

#define AUTO_SIDE_MIN_CLEAR_CM         20U
#define AUTO_SIDE_DIFF_CM               8U

#define AUTO_SIDE_AVOID_ENTER_CM       18U
#define AUTO_SIDE_AVOID_EXIT_CM        23U

#define AUTO_SIDE_AVOID_INNER_PWM      64U
#define AUTO_SIDE_AVOID_OUTER_PWM      74U

#define AUTO_STOP_BEFORE_TURN_MS      150U
#define AUTO_PIVOT_MIN_MS             250U
#define AUTO_PIVOT_MAX_MS            1500U
#define AUTO_STOP_AFTER_TURN_MS       120U

#define AUTO_PIVOT_SPEED_PERCENT       63U

#define AUTO_TURN_DECISION_TIMEOUT_MS 1200U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

osMessageQueueId_t sensorDataQueueHandle;

/* USER CODE END Variables */
/* Definitions for ControlTask */
osThreadId_t ControlTaskHandle;
const osThreadAttr_t ControlTask_attributes = {
  .name = "ControlTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for UltrasonicTask */
osThreadId_t UltrasonicTaskHandle;
const osThreadAttr_t UltrasonicTask_attributes = {
  .name = "UltrasonicTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for BluetoothTask */
osThreadId_t BluetoothTaskHandle;
const osThreadAttr_t BluetoothTask_attributes = {
  .name = "BluetoothTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for LoggerTask */
osThreadId_t LoggerTaskHandle;
const osThreadAttr_t LoggerTask_attributes = {
  .name = "LoggerTask",
  .stack_size = 384 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for manualCmdQueue */
osMessageQueueId_t manualCmdQueueHandle;
const osMessageQueueAttr_t manualCmdQueue_attributes = {
  .name = "manualCmdQueue"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartControlTask(void *argument);
void StartUltrasonicTask(void *argument);
void StartBluetoothTask(void *argument);
void StartLoggerTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

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
  /* creation of manualCmdQueue */
  manualCmdQueueHandle = osMessageQueueNew (8, sizeof(uint8_t), &manualCmdQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */

  sensorDataQueueHandle =
      osMessageQueueNew(
          4U,
          sizeof(SensorData_t),
          NULL);

  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of ControlTask */
  ControlTaskHandle = osThreadNew(StartControlTask, NULL, &ControlTask_attributes);

  /* creation of UltrasonicTask */
  UltrasonicTaskHandle = osThreadNew(StartUltrasonicTask, NULL, &UltrasonicTask_attributes);

  /* creation of BluetoothTask */
  BluetoothTaskHandle = osThreadNew(StartBluetoothTask, NULL, &BluetoothTask_attributes);

  /* creation of LoggerTask */
  LoggerTaskHandle = osThreadNew(StartLoggerTask, NULL, &LoggerTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartControlTask */
/**
  * @brief  Function implementing the ControlTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartControlTask */
void StartControlTask(void *argument)
{
  /* USER CODE BEGIN StartControlTask */
  /* Infinite loop */

	  uint8_t command;

	  SensorData_t latest_sensor;
	  bool has_sensor_data = false;

	  DriveMode_t drive_mode =
	      DRIVE_MODE_MANUAL;

	  AutoState_t auto_state =
	      AUTO_STATE_STOPPED;

	  bool auto_run_enabled = false;

	  bool left_ok;
	  bool right_ok;

	  bool left_fresh;
	  bool right_fresh;

	  SideAvoid_t side_avoid =
	      SIDE_AVOID_NONE;

	  uint32_t now;
	  uint32_t state_tick = 0U;
	  uint32_t elapsed;

	  uint32_t center_sample_tick;
	  uint32_t last_center_sample_tick =
	      UINT32_MAX;

	  uint8_t turn_clear_count = 0U;


	  if (PWM_Init(&htim2) != HAL_OK)
	  {
	    Move_Stop();

	    for (;;)
	    {
	      osDelay(1000);
	    }
	  }


	  Move_Init();

	  /* MANUAL 기본 속도 */
	  Move_SetSpeed(60U);

	  Move_Stop();


	  for (;;)
	  {
	    now = HAL_GetTick();
	    Move_Task();

	    while (osMessageQueueGet(
	               sensorDataQueueHandle,
	               &latest_sensor,
	               NULL,
	               0U) == osOK)
	    {
	      has_sensor_data = true;
	    }


	    while (osMessageQueueGet(
	               manualCmdQueueHandle,
	               &command,
	               NULL,
	               0U) == osOK)
	    {
	      switch (command)
	      {
	        case 'm':
	        case 'M':

	          Move_Stop();

	          drive_mode =
	              DRIVE_MODE_MANUAL;

	          auto_run_enabled = false;

	          auto_state =
	              AUTO_STATE_STOPPED;

	          side_avoid =
	              SIDE_AVOID_NONE;

	          turn_clear_count = 0U;

	          Move_SetSpeed(75U);

	          break;

	        case 'u':
	        case 'U':

	          Move_Stop();

	          drive_mode =
	              DRIVE_MODE_AUTO;

	          auto_run_enabled = true;

	          auto_state =
	              AUTO_STATE_STOPPED;

	          side_avoid =
	              SIDE_AVOID_NONE;

	          state_tick = now;

	          turn_clear_count = 0U;

	          Move_SetSpeed(74U);

	          break;

	        case 'x':
	        case 'X':

	          Move_Stop();

	          if (drive_mode ==
	              DRIVE_MODE_AUTO)
	          {
	            auto_run_enabled = false;

	            auto_state =
	                AUTO_STATE_STOPPED;
	          }

	          side_avoid =
	              SIDE_AVOID_NONE;

	          break;

	        case 'w':
	        case 'W':

	          if (drive_mode ==
	              DRIVE_MODE_MANUAL)
	          {
	            Move_Forward();
	          }

	          break;


	        case 's':
	        case 'S':

	          if (drive_mode ==
	              DRIVE_MODE_MANUAL)
	          {
	            Move_Backward();
	          }

	          break;


	        case 'a':
	        case 'A':

	          if (drive_mode ==
	              DRIVE_MODE_MANUAL)
	          {
	            Move_Left();
	          }

	          break;


	        case 'd':
	        case 'D':

	          if (drive_mode ==
	              DRIVE_MODE_MANUAL)
	          {
	            Move_Right();
	          }

	          break;


	        default:
	          break;
	      }
	    }

	    if (drive_mode == DRIVE_MODE_AUTO)
	    {
	      if (!auto_run_enabled)
	      {
	        Move_Stop();
	      }


	      else if (!has_sensor_data ||
	               !latest_sensor.center_valid ||
	               latest_sensor.center_timeout ||
	               (latest_sensor.center_age_ms >
	                AUTO_CENTER_MAX_AGE_MS))
	      {
	        Move_Stop();
	      }

	      else
	      {
	        switch (auto_state)
	        {
	          case AUTO_STATE_STOPPED:

	            Move_Stop();

	            if (latest_sensor.center_cm >
	                AUTO_FRONT_STOP_CM)
	            {
	              auto_state =
	                  AUTO_STATE_DRIVE;

	              Move_Forward();
	            }
	            else
	            {
	              auto_state =
	                  AUTO_STATE_STOP_BEFORE_TURN;

	              state_tick = now;
	            }

	            break;


	          case AUTO_STATE_DRIVE:
	            if (latest_sensor.center_cm <=
	                AUTO_FRONT_STOP_CM)
	            {
	              Move_Stop();

	              side_avoid =
	                  SIDE_AVOID_NONE;

	              auto_state =
	                  AUTO_STATE_STOP_BEFORE_TURN;

	              state_tick = now;

	              break;
	            }


	            left_fresh =
	                latest_sensor.left_valid &&
	                !latest_sensor.left_timeout &&
	                (latest_sensor.left_age_ms <=
	                 AUTO_SIDE_MAX_AGE_MS);

	            right_fresh =
	                latest_sensor.right_valid &&
	                !latest_sensor.right_timeout &&
	                (latest_sensor.right_age_ms <=
	                 AUTO_SIDE_MAX_AGE_MS);


	            if (side_avoid ==
	                SIDE_AVOID_LEFT_WALL)
	            {
	              if ((!left_fresh) ||
	                  (latest_sensor.left_cm >=
	                   AUTO_SIDE_AVOID_EXIT_CM))
	              {
	                side_avoid =
	                    SIDE_AVOID_NONE;
	              }
	            }
	            else if (side_avoid ==
	                     SIDE_AVOID_RIGHT_WALL)
	            {
	              if ((!right_fresh) ||
	                  (latest_sensor.right_cm >=
	                   AUTO_SIDE_AVOID_EXIT_CM))
	              {
	                side_avoid =
	                    SIDE_AVOID_NONE;
	              }
	            }

	            if (side_avoid ==
	                SIDE_AVOID_NONE)
	            {
	              if (left_fresh &&
	                  right_fresh &&
	                  (latest_sensor.left_cm <=
	                   AUTO_SIDE_AVOID_ENTER_CM) &&
	                  (latest_sensor.right_cm <=
	                   AUTO_SIDE_AVOID_ENTER_CM))
	              {
	                if (latest_sensor.left_cm <
	                    latest_sensor.right_cm)
	                {
	                  side_avoid =
	                      SIDE_AVOID_LEFT_WALL;
	                }
	                else if (latest_sensor.right_cm <
	                         latest_sensor.left_cm)
	                {
	                  side_avoid =
	                      SIDE_AVOID_RIGHT_WALL;
	                }
	              }
	              else if (left_fresh &&
	                       (latest_sensor.left_cm <=
	                        AUTO_SIDE_AVOID_ENTER_CM))
	              {
	                side_avoid =
	                    SIDE_AVOID_LEFT_WALL;
	              }
	              else if (right_fresh &&
	                       (latest_sensor.right_cm <=
	                        AUTO_SIDE_AVOID_ENTER_CM))
	              {
	                side_avoid =
	                    SIDE_AVOID_RIGHT_WALL;
	              }
	            }


	            if (side_avoid ==
	                SIDE_AVOID_LEFT_WALL)
	            {

	              Move_ForwardPWM(
	                  AUTO_SIDE_AVOID_OUTER_PWM,
	                  AUTO_SIDE_AVOID_INNER_PWM);
	            }
	            else if (side_avoid ==
	                     SIDE_AVOID_RIGHT_WALL)
	            {

	              Move_ForwardPWM(
	                  AUTO_SIDE_AVOID_INNER_PWM,
	                  AUTO_SIDE_AVOID_OUTER_PWM);
	            }
	            else
	            {
	              Move_Forward();
	            }

	            break;

	          case AUTO_STATE_STOP_BEFORE_TURN:

	            Move_Stop();

	            elapsed =
	                (uint32_t)(now - state_tick);

	            if (elapsed <
	                AUTO_STOP_BEFORE_TURN_MS)
	            {
	              break;
	            }


	            left_ok =
	                latest_sensor.left_valid &&
	                !latest_sensor.left_timeout &&
	                (latest_sensor.left_age_ms <=
	                 AUTO_SIDE_MAX_AGE_MS) &&
	                (latest_sensor.left_cm >=
	                 AUTO_SIDE_MIN_CLEAR_CM);


	            right_ok =
	                latest_sensor.right_valid &&
	                !latest_sensor.right_timeout &&
	                (latest_sensor.right_age_ms <=
	                 AUTO_SIDE_MAX_AGE_MS) &&
	                (latest_sensor.right_cm >=
	                 AUTO_SIDE_MIN_CLEAR_CM);

	            if (left_ok && right_ok)
	            {
	              if (latest_sensor.left_cm >=
	                  (uint16_t)(
	                      latest_sensor.right_cm +
	                      AUTO_SIDE_DIFF_CM))
	              {
	                auto_state =
	                    AUTO_STATE_PIVOT_LEFT;
	              }

	              else if (latest_sensor.right_cm >=
	                       (uint16_t)(
	                           latest_sensor.left_cm +
	                           AUTO_SIDE_DIFF_CM))
	              {
	                auto_state =
	                    AUTO_STATE_PIVOT_RIGHT;
	              }

	              else
	              {
	                if (elapsed <
	                    AUTO_TURN_DECISION_TIMEOUT_MS)
	                {
	                  break;
	                }

	                if (latest_sensor.left_cm >
	                    latest_sensor.right_cm)
	                {
	                  auto_state =
	                      AUTO_STATE_PIVOT_LEFT;
	                }
	                else
	                {
	                  auto_state =
	                      AUTO_STATE_PIVOT_RIGHT;
	                }
	              }
	            }

	            else if (left_ok)
	            {
	              auto_state =
	                  AUTO_STATE_PIVOT_LEFT;
	            }

	            else if (right_ok)
	            {
	              auto_state =
	                  AUTO_STATE_PIVOT_RIGHT;
	            }

	            else
	            {
	              Move_Stop();

	              if (elapsed <
	                  AUTO_TURN_DECISION_TIMEOUT_MS)
	              {
	                break;
	              }

	              if (latest_sensor.left_cm >
	                  latest_sensor.right_cm)
	              {
	                auto_state =
	                    AUTO_STATE_PIVOT_LEFT;
	              }
	              else
	              {
	                auto_state =
	                    AUTO_STATE_PIVOT_RIGHT;
	              }
	            }

	            state_tick = now;

	            turn_clear_count = 0U;

	            last_center_sample_tick =
	                now -
	                latest_sensor.center_age_ms;

	            if (auto_state ==
	                AUTO_STATE_PIVOT_LEFT)
	            {
	              Move_PivotLeft(
	                  AUTO_PIVOT_SPEED_PERCENT);
	            }
	            else
	            {
	              Move_PivotRight(
	                  AUTO_PIVOT_SPEED_PERCENT);
	            }

	            break;

	          case AUTO_STATE_PIVOT_LEFT:

	            elapsed =
	                (uint32_t)(now - state_tick);

	            if (elapsed >=
	                AUTO_PIVOT_MAX_MS)
	            {
	              Move_Stop();

	              auto_state =
	                  AUTO_STATE_SAFE_STOP;

	              auto_run_enabled = false;

	              break;
	            }

	            if (elapsed >=
	                AUTO_PIVOT_MIN_MS)
	            {
	              center_sample_tick =
	                  now -
	                  latest_sensor.center_age_ms;

	              if (center_sample_tick !=
	                  last_center_sample_tick)
	              {
	                last_center_sample_tick =
	                    center_sample_tick;


	                if (latest_sensor.center_cm >=
	                    AUTO_TURN_CLEAR_CM)
	                {
	                  if (turn_clear_count < 2U)
	                  {
	                    turn_clear_count++;
	                  }
	                }
	                else
	                {
	                  turn_clear_count = 0U;
	                }
	              }

	              if (turn_clear_count >= 2U)
	              {
	                Move_Stop();

	                auto_state =
	                    AUTO_STATE_STOP_AFTER_TURN;

	                state_tick = now;

	                turn_clear_count = 0U;
	              }
	            }

	            break;

	          case AUTO_STATE_PIVOT_RIGHT:

	            elapsed =
	                (uint32_t)(now - state_tick);


	            if (elapsed >=
	                AUTO_PIVOT_MAX_MS)
	            {
	              Move_Stop();

	              auto_state =
	                  AUTO_STATE_SAFE_STOP;

	              auto_run_enabled = false;

	              break;
	            }


	            if (elapsed >=
	                AUTO_PIVOT_MIN_MS)
	            {
	              center_sample_tick =
	                  now -
	                  latest_sensor.center_age_ms;


	              if (center_sample_tick !=
	                  last_center_sample_tick)
	              {
	                last_center_sample_tick =
	                    center_sample_tick;


	                if (latest_sensor.center_cm >=
	                    AUTO_TURN_CLEAR_CM)
	                {
	                  if (turn_clear_count < 2U)
	                  {
	                    turn_clear_count++;
	                  }
	                }
	                else
	                {
	                  turn_clear_count = 0U;
	                }
	              }


	              if (turn_clear_count >= 2U)
	              {
	                Move_Stop();

	                auto_state =
	                    AUTO_STATE_STOP_AFTER_TURN;

	                state_tick = now;

	                turn_clear_count = 0U;
	              }
	            }

	            break;

	          case AUTO_STATE_STOP_AFTER_TURN:

	        	  Move_Stop();

	        	  elapsed =
	        	      (uint32_t)(now - state_tick);

	        	  if (elapsed >=
	        	      AUTO_STOP_AFTER_TURN_MS)
	        	  {
	        	    if (latest_sensor.center_cm >
	        	        AUTO_FRONT_STOP_CM)
	        	    {
	        	      auto_state =
	        	          AUTO_STATE_DRIVE;

	        	      Move_Forward();
	        	    }
	        	    else
	        	    {

	        	      auto_state =
	        	          AUTO_STATE_STOP_BEFORE_TURN;

	        	      state_tick = now;

	        	      side_avoid =
	        	          SIDE_AVOID_NONE;

	        	      turn_clear_count = 0U;
	        	    }
	        	  }

	        	  break;

	          case AUTO_STATE_SAFE_STOP:
	          default:

	            Move_Stop();

	            auto_run_enabled = false;

	            break;
	        }
	      }
	    }


	    osDelay(2);
	  }

  /* USER CODE END StartControlTask */
}

/* USER CODE BEGIN Header_StartUltrasonicTask */
/**
* @brief Function implementing the UltrasonicTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartUltrasonicTask */
void StartUltrasonicTask(void *argument)
{
  /* USER CODE BEGIN StartUltrasonicTask */
  /* Infinite loop */

	  UltrasonicReading_t left;
	  UltrasonicReading_t center;
	  UltrasonicReading_t right;

	  SensorData_t sensor_data;
	  SensorData_t discarded_data;

	  if (Ultrasonic_Init(&htim4) != HAL_OK)
	  {
	    for (;;)
	    {
	      osDelay(1000);
	    }
	  }

	  for (;;)
	  {
	    Ultrasonic_Task();


	    Ultrasonic_GetReading(
	        ULTRASONIC_LEFT,
	        &left);

	    Ultrasonic_GetReading(
	        ULTRASONIC_CENTER,
	        &center);

	    Ultrasonic_GetReading(
	        ULTRASONIC_RIGHT,
	        &right);

	    if (left.has_measurement &&
	        center.has_measurement &&
	        right.has_measurement)
	    {
	      sensor_data.left_cm =
	          left.distance_cm;

	      sensor_data.center_cm =
	          center.distance_cm;

	      sensor_data.right_cm =
	          right.distance_cm;


	      sensor_data.left_valid =
	          !left.timeout;

	      sensor_data.center_valid =
	          !center.timeout;

	      sensor_data.right_valid =
	          !right.timeout;


	      sensor_data.left_timeout =
	          left.timeout;

	      sensor_data.center_timeout =
	          center.timeout;

	      sensor_data.right_timeout =
	          right.timeout;


	      sensor_data.left_age_ms =
	          left.age_ms;

	      sensor_data.center_age_ms =
	          center.age_ms;

	      sensor_data.right_age_ms =
	          right.age_ms;


	      if (osMessageQueuePut(
	              sensorDataQueueHandle,
	              &sensor_data,
	              0U,
	              0U) != osOK)
	      {
	        (void)osMessageQueueGet(
	            sensorDataQueueHandle,
	            &discarded_data,
	            NULL,
	            0U);

	        (void)osMessageQueuePut(
	            sensorDataQueueHandle,
	            &sensor_data,
	            0U,
	            0U);
	      }
	    }

	    osDelay(5);
	  }

  /* USER CODE END StartUltrasonicTask */
}

/* USER CODE BEGIN Header_StartBluetoothTask */
/**
* @brief Function implementing the BluetoothTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartBluetoothTask */
void StartBluetoothTask(void *argument)
{
  /* USER CODE BEGIN StartBluetoothTask */
  /* Infinite loop */

	  uint8_t rx_byte;
	  osStatus_t queue_status;

	  for (;;)
	  {
	    if (HAL_UART_Receive(
	            &huart1,
	            &rx_byte,
	            1U,
	            0U) == HAL_OK)
	    {
	      switch (rx_byte)
	      {
	        case 'w':
	        case 'W':

	        case 's':
	        case 'S':

	        case 'a':
	        case 'A':

	        case 'd':
	        case 'D':

	        case 'x':
	        case 'X':

	        case 'm':
	        case 'M':

	        case 'u':
	        case 'U':

	          queue_status =
	              osMessageQueuePut(
	                  manualCmdQueueHandle,
	                  &rx_byte,
	                  0U,
	                  0U);

	          if (queue_status != osOK)
	          {
	            (void)osMessageQueueReset(
	                manualCmdQueueHandle);

	            (void)osMessageQueuePut(
	                manualCmdQueueHandle,
	                &rx_byte,
	                0U,
	                0U);
	          }

	          break;

	        default:
	          break;
	      }
	    }

	    osDelay(2);
	  }

  /* USER CODE END StartBluetoothTask */
}

/* USER CODE BEGIN Header_StartLoggerTask */
/**
* @brief Function implementing the LoggerTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartLoggerTask */
void StartLoggerTask(void *argument)
{
  /* USER CODE BEGIN StartLoggerTask */
  /* Infinite loop */

	  UltrasonicReading_t left;
	  UltrasonicReading_t center;
	  UltrasonicReading_t right;

	  char msg[128];

	  for (;;)
	  {
	    Ultrasonic_GetReading(
	        ULTRASONIC_LEFT,
	        &left);

	    Ultrasonic_GetReading(
	        ULTRASONIC_CENTER,
	        &center);

	    Ultrasonic_GetReading(
	        ULTRASONIC_RIGHT,
	        &right);

	    snprintf(
	        msg,
	        sizeof(msg),
	        "[US] L=%u%s C=%u%s R=%u%s\r\n",
	        left.distance_cm,
	        left.timeout ? "(TO)" : "",
	        center.distance_cm,
	        center.timeout ? "(TO)" : "",
	        right.distance_cm,
	        right.timeout ? "(TO)" : "");

	    HAL_UART_Transmit(
	        &huart2,
	        (uint8_t *)msg,
	        (uint16_t)strlen(msg),
	        100U);

	    osDelay(300);
	  }


  /* USER CODE END StartLoggerTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
