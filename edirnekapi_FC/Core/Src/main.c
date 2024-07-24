/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <math.h>
#include "bme280.h"
#include "bmi088.h"
#include "algorithms.h"
#include "queternion.h"

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
 I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

//static BME_280_t BME280_sensor;
static bmi088_struct_t BMI_sensor;


// Free parameters in the Mahony filter and fusion scheme,
// Kp for proportional feedback, Ki for integral


extern int errorLine;

float currentTime = 0;
float lastTime =0;

int counter = 0;
int counterAcc = 0;
int counterGy = 0;
uint8_t buf[250];


#if defined(ALGORITHM_1)
static algorithmStatus algorithm_1_stat[2];
#endif
#if defined(ALGORITHM_2)
static algorithmStatus algorithm_2_stat[2];
#endif


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
static void bmiBegin();

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
  MX_I2C1_Init();
  MX_I2C3_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  //bme280_init(&BME280_sensor, &hi2c1, BME280_MODE_NORMAL, BME280_OS_8, BME280_FILTER_4);
  HAL_NVIC_DisableIRQ(EXTI3_IRQn);
  HAL_NVIC_DisableIRQ(EXTI4_IRQn);
  bmiBegin();
  HAL_Delay(200);
  //HAL_UART_Transmit(&huart2, (uint8_t*)"$PMTK251,38400*27\r\n", strlen("$PMTK251,38400*27\r\n"), 100);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  //bme280_update();
	  bmi088_update();

#if defined(ALGORITHM_1)
	  algorithm_1_update(&BME280_sensor, algorithm_1_stat);
#endif
#if defined(ALGORITHM_2)
	  algorithm_2_update(&BME280_sensor, &BMI_sensor, algorithm_2_stat);
#endif


	  /*
	 if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_RXNE) == SET) {
		 HAL_UART_Receive(&huart2, (uint8_t *) dat, 1, 100);
		 HAL_UART_Transmit(&huart1, (uint8_t *) dat, 1, 100);
		 HAL_GPIO_TogglePin(Led_GPIO_Port, Led_Pin);
	 }
	 */

	  currentTime = ((float)HAL_GetTick()) / 1000.0;

		 if(currentTime - lastTime > 0.03)
		 {

			 //sprintf((char*)buf, "temp = %.2fC  time %.0fuS A_x: %.1fmG  A_y: %.1fm  A_z: %.1fmG   G_z: %fdps\r\n", BMI_sensor.temp, BMI_sensor.currentTime, BMI_sensor.acc_x, BMI_sensor.acc_y, BMI_sensor.acc_z, BMI_sensor.gyro_z);
			 //sprintf((char*)buf, "G_x: %f  G_y: %f  G_z: %f   \r\n", BMI_sensor.gyro_x_angle, BMI_sensor.gyro_y_angle, BMI_sensor.gyro_z_angle);
			 //sprintf((char*)buf, "counter = %d  A_x: %.0f   A_y: %.0f   A_z: %.0f\r\n", counter, BMI_sensor.acc_x, BMI_sensor.acc_y, BMI_sensor.acc_z);
			 //sprintf((char*)buf, "counter = %d  g_x: %f   g_y: %f   g_z: %f\r\n", counter, BMI_sensor.gyro_x, BMI_sensor.gyro_y, BMI_sensor.gyro_z);
			 //sprintf((char*)buf, "gyro chip id = %d\r\n", bmi088_getGyroChipId());
			 //sprintf((char*)buf, "g_x:%f \t g_y:%f \t g_z:%f \t a_x:%f \t a_y:%f \t a_z:%f\r\n", BMI_sensor.gyro_x, BMI_sensor.gyro_y, BMI_sensor.gyro_z, BMI_sensor.acc_x, BMI_sensor.acc_y, BMI_sensor.acc_z);			 //sprintf((char*)buf, "Olusan Yeni Vektör: (%f, %f, %f)\n\r", rotatedVector.x, rotatedVector.y, rotatedVector.z);
			 //sprintf((char*)buf, "roll:%f   pitch:%f   yaw:%f\n\r", roll, pitch, yaw);
			 //sprintf((char*)buf, "irtifa: %.1f \t aci: %.0f \r\n", BME280_sensor.altitude, teta);
			 //sprintf((char*)buf, "irtifa: %.1fm \t sicaklik %.0fC \t nem: %0.f%% \r\n", BME280_sensor.altitude, BME280_sensor.temperature, BME280_sensor.humidity);
			 //HAL_I2C_Mem_Read(&hi2c3, ACC_I2C_ADD, ACC_CHIP_ID, 1, dat, I2C_MEMADD_SIZE_8BIT, 100);
			 //sprintf((char*)buf, "chip id: %d\n\r", *dat);

			 //sprintf((char*)buf, "%c", *dat);
			 //sprintf((char*)buf, "sizeof float: %d   sizeof double: %d\r\n", sizeof(float), sizeof(double));

			 //sprintf((char*)buf, "q[0]: %f  q[1]: %f  q[2]: %f   q[3]: %f\r\n", q[0], q[1], q[2], q[3]);
			 //sprintf((char*)buf, "v[0]: %f  v[1]: %f  v[2]: %f   teta: %f\r\n", vector[0], vector[1], vector[2], (180.0 / M_PI) * atan2(sqrt(pow(vector[0],2.0) + pow(vector[1],2.0)), vector[2]));
			 //sprintf((char*)buf, "teta: %f\r\n", (180.0 / M_PI) * atan2(sqrt(pow(BMI_sensor.acc_x,2.0) + pow(BMI_sensor.acc_y,2.0)), BMI_sensor.acc_z));
			 sprintf((char*)buf, "A %f %f %f\r\n", yaw,  -roll, pitch);

			 HAL_UART_Transmit(&huart1, buf, strlen((char*) buf), 250);

			 lastTime = currentTime;

		 }

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 64;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 100000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 38400;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, BUZZER_Pin|Led_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : BUZZER_Pin Led_Pin */
  GPIO_InitStruct.Pin = BUZZER_Pin|Led_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : INT_ACC_Pin INT_GYRO_Pin */
  GPIO_InitStruct.Pin = INT_ACC_Pin|INT_GYRO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI3_IRQn, 1, 0);


  HAL_NVIC_SetPriority(EXTI4_IRQn, 1, 1);

}

/* USER CODE BEGIN 4 */
void bmiBegin()
{
	//Acccel config
	BMI_sensor.deviceConfig.acc_bandwith = ACC_BWP_OSR4;
	BMI_sensor.deviceConfig.acc_outputDateRate = ACC_ODR_400;
	BMI_sensor.deviceConfig.acc_powerMode = ACC_PWR_SAVE_ACTIVE;
	BMI_sensor.deviceConfig.acc_range = ACC_RANGE_12G;

	//Gyro config
	BMI_sensor.deviceConfig.gyro_bandWidth = GYRO_BW_64;
	BMI_sensor.deviceConfig.gyro_range = GYRO_RANGE_2000;
	BMI_sensor.deviceConfig.gyro_powerMode = GYRO_LPM_NORMAL;
	bmi088_init(&BMI_sensor, &hi2c3);
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == INT_GYRO_Pin)
    {
    	bmi088_getGyroDatas_INT();
    	//counterGy++;
    }
    if(GPIO_Pin == INT_ACC_Pin)
    {
    	bmi088_getAccelDatas_INT();
    	//counterAcc++;
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
	HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);
	HAL_Delay(100);
	HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);

	sprintf((char*)buf, "error line: %d\r\n", errorLine);
	HAL_UART_Transmit(&huart1, buf, strlen((char*) buf), 250);
 /*
	__disable_irq();
  while (1)
  {

  }
  */
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
