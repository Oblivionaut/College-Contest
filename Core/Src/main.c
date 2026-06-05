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
#include "main.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "headfile.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* 调试开关：下面几个测试宏全为0时，运行正常比赛赛道模式。 */
#define TEST_TARGET_YAW 100.0f          /* ANGLE_HOLD_TEST使用的目标角度 */
#define GY87_DATA_DEBUG_TEST 0          /* 1=只显示GY87姿态/磁力计调试数据 */
#define ANGLE_HOLD_TEST 0               /* 1=角度环定点测试 */
#define MOTOR_PID_DEBUG_TEST 0          /* 1=电机PID阶跃测试 */
#define MOTOR_PID_DEBUG_STEP_TICKS 400U /* 电机PID测试每个目标保持周期 */
#define MOTOR_DIRECT_TEST 0             /* 1=绕过赛道状态机，直接给左右轮速度 */
#define MOTOR_DIRECT_SPEED_A 8.0f       /* MOTOR_DIRECT_TEST左轮速度 */
#define MOTOR_DIRECT_SPEED_B 8.0f       /* MOTOR_DIRECT_TEST右轮速度 */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile int16_t Speed = 20;	//编码器测速测试变量
	  static uint8_t test_cnt = 0;
volatile uint8_t Debug_Print_Flag = 0;
volatile int16_t Location = 0;
uint16_t Temp = 0;
uint8_t Key_Status = 0;
#if MOTOR_PID_DEBUG_TEST
static uint16_t MotorPidDebugCnt = 0;
static uint8_t MotorPidDebugIndex = 0;
static const float MotorPidDebugTargets[4][2] =
{
	{8.0f, 0.0f},
	{0.0f, 8.0f},
	{14.0f, 14.0f},
	{0.0f, 0.0f}
};
#endif
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
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
  MX_TIM3_Init();
  MX_TIM1_Init();
  MX_TIM4_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_UART4_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_USART6_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
	OLED_Init();
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
//	MotorA.Kp = 3.0;
//	MotorA.Ki = 0.1;
//	MotorA.Kd = 0;
	GY87_Init();
	PID_SET(&MotorA, 95.0, 1.5, 0.0);//95.0, 1.5, 0.0
	PID_SET(&MotorB, 95.0, 1.5, 0.0);
	Angle_ResetController();
	Motor_Stop();
	HAL_TIM_Base_Start_IT(&htim2);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  (void)GY87_Update();

	  if(Debug_Print_Flag)
	  {
		  Debug_Print_Flag = 0;
#if GY87_DATA_DEBUG_TEST
		  OLED_GY87_Data_Debug();
#elif ANGLE_HOLD_TEST
		  OLED_Angle_Debug(TEST_TARGET_YAW);
#elif MOTOR_PID_DEBUG_TEST
		  OLED_MotorPid_Debug();
#else
		  OLED_Tracing_Run_Display();
#endif
	  }

//	  OLED_Tracing_Debug();
//	  Normal_Tracing();
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM2)
  {
	  
#if ANGLE_HOLD_TEST
	Angle_Hold(TEST_TARGET_YAW);
#elif MOTOR_PID_DEBUG_TEST
	Motor_SetSpeed(MotorPidDebugTargets[MotorPidDebugIndex][0],
	               MotorPidDebugTargets[MotorPidDebugIndex][1]);

	MotorPidDebugCnt++;

	if(MotorPidDebugCnt >= MOTOR_PID_DEBUG_STEP_TICKS)
	{
		MotorPidDebugCnt = 0;
		MotorPidDebugIndex++;

		if(MotorPidDebugIndex >= 4U)
		{
			MotorPidDebugIndex = 0;
		}
	}
#else
#if MOTOR_DIRECT_TEST
	Motor_SetSpeed(MOTOR_DIRECT_SPEED_A, MOTOR_DIRECT_SPEED_B);
#else
	Tracing_Button_Update();
	Tracing_Mode_Select(Tracing_GetActiveMode());
#endif
#endif
	  Motor_Pid();
	test_cnt++;

	if(test_cnt == 100)
	{
		Debug_Print_Flag = 1;
		test_cnt = 0;
	} 
	  
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
