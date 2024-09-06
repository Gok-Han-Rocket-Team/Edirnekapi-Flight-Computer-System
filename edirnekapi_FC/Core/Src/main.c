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
#include "lora.h"
#include "usr_gnss_l86_parser.h"
#include "dataPacking.h"
#include "externalPins.h"
#include "configuration.h"
#include "reset_detect.h"
#include "strain_gauge.h"
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
 ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c3;

RTC_HandleTypeDef hrtc;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_uart4_tx;
DMA_HandleTypeDef hdma_uart4_rx;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart2_rx;

/* USER CODE BEGIN PV */

static BME_280_t BME280_sensor;
bmi088_struct_t BMI_sensor;
static lorastruct e22_lora;
static S_GPS_L86_DATA gnss_data;
static power guc;
uint8_t mosfet_buffer[3] = {0};
volatile int is_updated_uart4 = 0;
static ext_pin_s mos_1;		//parachute 1
static ext_pin_s mos_2;		//parachute 2
ext_pin_s led;
ext_pin_s buzzer;

#ifndef ROCKET_CARD
hx711_t loadcell;
#endif

RTC_TimeTypeDef sTime;
RTC_DateTypeDef sDate;

extern float euler[3];
extern uint8_t is_offset_taken;
extern _f g_GnssRx_Flag;
extern _f g_openFixedDataTransmition;
float currentTime = 0;
float lastTime =0;
float lastTime2 =0;
float powerLastTime = 0;
float loraLastTime = 0;
float lora_hz = 1.0;
float wattLastTime = 0;
volatile float teta = 0;

extern int errorLine;

int counter = 0;
int counterAcc = 0;
int counterGy = 0;
int configBmi = 0;
int is_BME_ok = 0;
int is_BMI_ok = 0;
int is_quaternion_zeroed = 0;

uint32_t buzzLastTime = 0;

uint8_t buf[250];

backup_sram_datas_s *saved_datas = (backup_sram_datas_s *)BKPSRAM_BASE;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_UART4_Init(void);
static void MX_RTC_Init(void);
/* USER CODE BEGIN PFP */
static void bme280_begin();
static void bmi088_begin();
static void loraBegin();
void measurePower(power *guc);
void getWatt(void);

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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_UART4_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */
  HAL_NVIC_SetPriority(EXTI3_IRQn, 2, 0);
  HAL_NVIC_SetPriority(EXTI4_IRQn, 2, 0);
  lora_deactivate();
  HAL_PWR_EnableBkUpAccess();
  RCC->AHB1ENR |= RCC_AHB1ENR_BKPSRAMEN;
  HAL_PWR_EnableBkUpReg();

  HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
  HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

  HAL_Delay(5);
  bme280_begin();
  bmi088_begin();



  if(measure_abs_time(sTime, sDate) > 1)
  {
	  if(is_BMI_ok)
		  bmi088_config();
	  if(is_BME_ok)
		  bme280_config();

	  saved_datas->r_status = STAT_ROCKET_READY;
	  saved_datas->max_altitude = 0.0;
	  saved_datas->offset_vals[0] = 0.0;
	  saved_datas->offset_vals[1] = 0.0;
	  saved_datas->offset_vals[2] = 0.0;
	  saved_datas->q[0] = 0.0;
	  saved_datas->q[1] = 0.0;
	  saved_datas->q[2] = 0.0;
	  saved_datas->q[3] = 0.0;

	  for(int i = 0; i < 20; i++)
	  {
		  HAL_GPIO_TogglePin(BUZZER_GPIO_Port, BUZZER_Pin);
		  HAL_Delay(50);
	  }

	  loraBegin();
	  lora_deactivate();
	  HAL_UART_Transmit(&huart2, (uint8_t*)"$PMTK251,57600*2C\r\n", 19, 100);
	  //HAL_UART_Transmit(&huart2, "$PMTK251,9600*17\r\n", 18, 100);				// 9600 bps
	  if(is_BMI_ok)
		  getOffset();

#ifndef	ROCKET_CARD
  straing_gage_gpio_init(&loadcell, GPIO_0_GPIO_Port, GPIO_0_Pin, GPIO_1_GPIO_Port, GPIO_1_Pin);
#endif
	  HAL_Delay(10);
  }



  HAL_UART_DeInit(&huart4);
  HAL_UART_DeInit(&huart2);
  huart4.Init.BaudRate = 115200;
  huart2.Init.BaudRate = 57600;
  HAL_UART_Init(&huart4);					//Telemetry
  HAL_UART_Init(&huart2);					//GNSS
  HAL_DMA_Init(&hdma_usart1_tx);
  HAL_DMA_Init(&hdma_usart2_rx);
  HAL_DMA_Init(&hdma_uart4_tx);
  HAL_DMA_Init(&hdma_uart4_rx);
  HAL_UART_Receive_DMA(&huart4, mosfet_buffer, 3);

  mos_1.gpio_port = P_1_MOS_GPIO_Port;
  mos_1.gpio_pin = P_1_MOS_Pin;
  mos_2.gpio_port = P_2_MOS_GPIO_Port;
  mos_2.gpio_pin = P_2_MOS_Pin;
  led.gpio_port = LED_GPIO_Port;
  led.gpio_pin = LED_Pin;
  buzzer.gpio_port = BUZZER_GPIO_Port;
  buzzer.gpio_pin = BUZZER_Pin;

  //Interrupt activation for IMU sensor.
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);
  loraLastTime = -1.0;

  //This macro for viewing the gps raw data.
  //VIEW_GPS()

  UsrGpsL86Init(&huart2);

  ext_pin_open_duration(&buzzer, 1000);
  BMI_sensor.rawDatas.isGyroUpdated = 0;
  BMI_sensor.rawDatas.isAccelUpdated = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if(is_BMI_ok)
		  bmi088_update();
	  if(is_BME_ok)
		  bme280_update();

	  measurePower(&guc);

	  ext_pin_update(&mos_1);
	  ext_pin_update(&mos_2);
	  ext_pin_update(&led);
	  ext_pin_update(&buzzer);

#if defined(ALGORITHM_1)
	  if(is_BME_ok)
		  algorithm_1_update(&BME280_sensor);
#endif
#if defined(ALGORITHM_2)
	  teta = quaternionToTheta();
	  if(is_BMI_ok)
		  algorithm_2_update(&BME280_sensor, &BMI_sensor, teta);
#endif

		  if(saved_datas->r_status == STAT_FLIGHT_STARTED){lora_hz = 5;}
		  else if(saved_datas->r_status > STAT_MOTOR_BURNOUT){lora_hz = 1;}

		  if(BME280_sensor.altitude > 4000 && is_quaternion_zeroed == 0)
		  {
			  quaternionSet_zero();
			  is_quaternion_zeroed = 1;
		  }
	  	  currentTime = ((float)HAL_GetTick()) / 1000.0;

	  	 //Set initial quaternion every minute.
		 if(fabs(currentTime - lastTime2) > 60)
		 {
			 if(saved_datas->r_status == STAT_ROCKET_READY && sqrt(pow(BMI_sensor.gyro_x, 2) + pow(BMI_sensor.gyro_y, 2) + pow(BMI_sensor.gyro_z, 2)) < 5.0 && is_BMI_ok == 1)
			 {
				 getInitialQuaternion();
				 ext_pin_open_duration(&buzzer, 500);
			 }
			 lastTime2 = currentTime;
		 }

		 //GNSS get location
		 if(g_GnssRx_Flag)
		 {
			 Usr_GpsL86GetValues(&gnss_data);
		 }

		 //Lora timer;
		 //loop_counter += 1;
		 currentTime = ((float)HAL_GetTick()) / 1000.0;
		 if(fabs(currentTime - loraLastTime) > (1.0 / lora_hz))
		 {
			 //BME280_sensor.velocity = (float)(loop_counter);
			 getWatt();
			 packDatas(&BMI_sensor, &BME280_sensor, &gnss_data, &guc, saved_datas->r_status);
			 loraLastTime = currentTime;
			 //loop_counter = 0;
		 }


		 //some infos
		 if(fabs(currentTime - lastTime) > 0.2)
		 {
			 //sprintf((char*)buf, "temp = %.2fC  time %.0fuS A_x: %.1fmG  A_y: %.1fm  A_z: %.1fmG   G_z: %fdps\r\n", BMI_sensor.temp, BMI_sensor.currentTime, BMI_sensor.acc_x, BMI_sensor.acc_y, BMI_sensor.acc_z, BMI_sensor.gyro_z);
			 //sprintf((char*)buf, "G_x: %f  G_y: %f  G_z: %f   \r\n", BMI_sensor.gyro_x_angle, BMI_sensor.gyro_y_angle, BMI_sensor.gyro_z_angle);
			 //sprintf((char*)buf, "counter = %d  A_x: %.0f   A_y: %.0f   A_z: %.0f\r\n", counter, BMI_sensor.acc_x, BMI_sensor.acc_y, BMI_sensor.acc_z);
			 //sprintf((char*)buf, "g_x: %f   g_y: %f   g_z: %f\r\n", BMI_sensor.gyro_x, BMI_sensor.gyro_y, BMI_sensor.gyro_z);
			 //sprintf((char*)buf, "gyro chip id = %d\r\n", bmi088_getGyroChipId());
			 //sprintf((char*)buf, "a_x:%f \t a_y:%f \t a_z:%f\r\n", BMI_sensor.acc_x, BMI_sensor.acc_y, BMI_sensor.acc_z);
			 //sprintf((char*)buf, "Olusan Yeni Vektör: (%f, %f, %f)\n\r", rotatedVector.x, rotatedVector.y, rotatedVector.z);
			 //sprintf((char*)buf, "roll:%f   pitch:%f   yaw:%f\n\r", roll, pitch, yaw);
			 //sprintf((char*)buf, "irtifa: %.1f \t aci: %.0f \r\n", BME280_sensor.altitude, teta);
			 //sprintf((char*)buf, "irtifa: %.1fm \t sicaklik %.0fC \t nem: %0.f%% \r\n", BME280_sensor.altitude, BME280_sensor.temperature, BME280_sensor.humidity);
			 //sprintf((char*)buf, "chip id: %d\n\r", *dat);
			 //sprintf((char*)buf, "%c", *dat);
			 //sprintf((char*)buf, "sizeof float: %d   sizeof double: %d\r\n", sizeof(float), sizeof(double));
			 //sprintf((char*)buf, "q[0]: %f  q[1]: %f  q[2]: %f   q[3]: %f\r\n", q[0], q[1], q[2], q[3]);
			 //sprintf((char*)buf, "v[0]: %f  v[1]: %f  v[2]: %f   teta: %f\r\n", vector[0], vector[1], vector[2], (180.0 / M_PI) * atan2(sqrt(pow(vector[0],2.0) + pow(vector[1],2.0)), vector[2]));
			 //sprintf((char*)buf, "teta: %f\r\n", (180.0 / M_PI) * atan2(sqrt(pow(BMI_sensor.acc_x,2.0) + pow(BMI_sensor.acc_y,2.0)), BMI_sensor.acc_z));
			 //sprintf((char*)buf, "teta = %f", teta);
			 //sprintf((char*)buf, "speed = %f\n\r", BME280_sensor.velocity);
			 //HAL_UART_Transmit(&huart1, buf, strlen((char*) buf), 250);
			 HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
			 HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
			 save_time(sTime, sDate);
			 lastTime = currentTime;
		 }

		 //This block is used for manual deploy via telemetry for testing.
		if(is_updated_uart4 == 1)
		{
			//ext_pin_open(&buzzer);
		  if(strcmp((char*)mosfet_buffer, "OK1") == 0)
		  {
			  deploy_p_1();
			  ext_pin_open(&led);
		  }
		  if(strcmp((char*)mosfet_buffer, "OK2") == 0)
		  {
			  deploy_p_2();
			  ext_pin_open(&led);
		  }
		  is_updated_uart4 = 0;
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
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

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 9600;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

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
  huart2.Init.BaudRate = 9600;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);
  /* DMA1_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA2_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, P_1_MOS_Pin|P_2_MOS_Pin|LORA_M0_Pin|LORA_M1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_0_Pin|GPIO_1_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, BUZZER_Pin|LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : P_1_MOS_Pin P_2_MOS_Pin LORA_M0_Pin LORA_M1_Pin */
  GPIO_InitStruct.Pin = P_1_MOS_Pin|P_2_MOS_Pin|LORA_M0_Pin|LORA_M1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : GPIO_0_Pin */
  GPIO_InitStruct.Pin = GPIO_0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIO_0_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : GPIO_1_Pin BUZZER_Pin LED_Pin */
  GPIO_InitStruct.Pin = GPIO_1_Pin|BUZZER_Pin|LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : INT_ACC_Pin INT_GYRO_Pin */
  GPIO_InitStruct.Pin = INT_ACC_Pin|INT_GYRO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : LORA_AUX_Pin */
  GPIO_InitStruct.Pin = LORA_AUX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(LORA_AUX_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI3_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

}

/* USER CODE BEGIN 4 */

void bme280_begin()
{
	BME280_sensor.device_config.bme280_filter = 			BME280_FILTER_8;
	BME280_sensor.device_config.bme280_mode =				BME280_MODE_NORMAL;
	BME280_sensor.device_config.bme280_output_speed =		BME280_OS_8;
	bme280_init(&BME280_sensor, &hi2c1);
}
void bmi088_begin()
{
	//Acccel config
	BMI_sensor.deviceConfig.acc_bandwith = ACC_BWP_OSR4;
	BMI_sensor.deviceConfig.acc_outputDateRate = ACC_ODR_200;
	BMI_sensor.deviceConfig.acc_powerMode = ACC_PWR_SAVE_ACTIVE;
	BMI_sensor.deviceConfig.acc_range = ACC_RANGE_12G;

	//Gyro config
	BMI_sensor.deviceConfig.gyro_bandWidth = GYRO_BW_230;
	BMI_sensor.deviceConfig.gyro_range = GYRO_RANGE_2000;
	BMI_sensor.deviceConfig.gyro_powerMode = GYRO_LPM_NORMAL;
	bmi088_init(&BMI_sensor, &hi2c3);
}


void loraBegin()
{
	HAL_GPIO_WritePin(LORA_M0_GPIO_Port, LORA_M0_Pin, RESET);
	HAL_GPIO_WritePin(LORA_M1_GPIO_Port, LORA_M1_Pin, SET);
	HAL_Delay(100);
/*
    while(1){
    	if(__HAL_UART_GET_FLAG(&huart4, UART_FLAG_RXNE) == SET) {
			 uint8_t dat[1];
			 HAL_UART_Receive(&huart4, (uint8_t *) dat, 1, 100);
			 HAL_UART_Transmit(&huart1, (uint8_t *) dat, 1, 100);
			 HAL_GPIO_TogglePin(Led_GPIO_Port, Led_Pin);
   	 	 }
   	 	 if(__HAL_UART_GET_FLAG(&huart1, UART_FLAG_RXNE) == SET) {
			 uint8_t dat[1];
			 HAL_UART_Receive(&huart1, (uint8_t *) dat, 1, 100);
			 HAL_UART_Transmit(&huart4, (uint8_t *) dat, 1, 100);
			 HAL_GPIO_TogglePin(Led_GPIO_Port, Led_Pin);
   	 	 }
    }
*/
	//while(!HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9));

    e22_lora.baudRate = LORA_BAUD_115200;
    e22_lora.airRate = LORA_AIR_RATE_38_4k;
    e22_lora.packetSize = LORA_SUB_PACKET_64_BYTES;
    e22_lora.power = LORA_POWER_37dbm;
    e22_lora.loraAddress.address16 = 0x0000;
    e22_lora.loraKey.key16 = 0x0000;

#ifdef ROCKET_CARD
    e22_lora.channel = ROCKET_TELEM_FREQ;
#else
    e22_lora.channel = PAYLOAD_TELEM_FREQ;
#endif

    lora_configure(&e22_lora);

    HAL_Delay(100);
}

void measurePower(power *guc_)
{
	  HAL_ADC_Start(&hadc1);
	  HAL_ADC_PollForConversion(&hadc1, 10);
	  int adc1 = HAL_ADC_GetValue(&hadc1);
	  HAL_ADC_PollForConversion(&hadc1, 10);
	  int adc2 = HAL_ADC_GetValue(&hadc1);
	  HAL_ADC_Stop (&hadc1);

	  guc_->akim =   (float)adc1 * 3300 / 4096;
	  guc_->voltaj = (float)adc2 * 13.2 / 4096;
	  guc_->mWatt += guc_->akim * guc_->voltaj * (((float)HAL_GetTick() / 1000) - powerLastTime);
	  powerLastTime = (float)HAL_GetTick() / 1000;
}

void getWatt()
{

	float currentTime = (float)HAL_GetTick() / 1000;
	float deltaTime = currentTime - wattLastTime;
	wattLastTime = currentTime;
	guc.mWatt_s = guc.mWatt / deltaTime;
	guc.mWatt = 0.0;
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
    	counterAcc++;
    }
}

void deploy_p_1()
{
	ext_pin_open_duration(&mos_1, 100);
	ext_pin_open(&buzzer);
}

void deploy_p_2()
{
	ext_pin_open_duration(&mos_2, 100);
	ext_pin_open(&buzzer);
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

	sprintf((char*)buf, "error line: %d\r\n", errorLine);
	HAL_UART_Transmit(&huart1, buf, strlen((char*) buf), 250);


	__disable_irq();
  while (1)
  {
		HAL_GPIO_TogglePin(BUZZER_GPIO_Port, BUZZER_Pin);
		HAL_Delay(100);
		HAL_GPIO_TogglePin(BUZZER_GPIO_Port, BUZZER_Pin);
		HAL_Delay(100);
  }

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
