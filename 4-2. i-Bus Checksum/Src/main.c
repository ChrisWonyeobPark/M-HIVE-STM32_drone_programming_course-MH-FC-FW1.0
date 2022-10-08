/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bno080.h"
#include "quaternion.h"
#include "icm20602.h"
#include "lps22hh.h"
#include "M8N.h"
#include "FS-iA6B.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
int _write(int file, char* p, int len)
{
	for(int i=0;i<len;i++)
	{
		while(!LL_USART_IsActiveFlag_TXE(USART6));
		LL_USART_TransmitData8(USART6, *(p+i));
	}
	return len;
}
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
extern uint8_t uart6_rx_flag;
extern uint8_t uart6_rx_data;

extern uint8_t m8n_rx_buf[36];
extern uint8_t m8n_rx_cplt_flag;

extern uint8_t ibus_rx_buf[32];
extern uint8_t ibus_rx_cplt_flag;
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
	int count = 0;
	float f = 1.234;
	float q[4];
	float quatRadianAccuracy;
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
  MX_USART6_UART_Init();
  MX_SPI2_Init();
  MX_SPI1_Init();
  MX_SPI3_Init();
  MX_UART4_Init();
  MX_UART5_Init();
  /* USER CODE BEGIN 2 */
  LL_TIM_EnableCounter(TIM3);
  LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH4);

  TIM3->PSC = 2000;
  HAL_Delay(100);
  TIM3->PSC = 1500;
  HAL_Delay(100);
  TIM3->PSC = 1000;
  HAL_Delay(100);

  LL_TIM_CC_DisableChannel(TIM3, LL_TIM_CHANNEL_CH4);

  LL_USART_EnableIT_RXNE(USART6);
  LL_USART_EnableIT_RXNE(UART4);
  LL_USART_EnableIT_RXNE(UART5);

  BNO080_Initialization();
  BNO080_enableRotationVector(2500);

  ICM20602_Initialization();
  LPS22HH_Initialization();
  M8N_Initialization();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//	  if(BNO080_dataAvailable() == 1)
//	  {
//		  LL_GPIO_TogglePin(GPIOC, LL_GPIO_PIN_0);
//
//		  q[0] = BNO080_getQuatI();
//		  q[1] = BNO080_getQuatJ();
//		  q[2] = BNO080_getQuatK();
//		  q[3] = BNO080_getQuatReal();
//		  quatRadianAccuracy = BNO080_getQuatRadianAccuracy();
//
//		  Quaternion_Update(&q[0]);
//
//		  //printf("%d,%d,%d\n", (int)(BNO080_Roll*100), (int)(BNO080_Pitch*100), (int)(BNO080_Yaw*100));
//	  }

//	  if(ICM20602_DataReady() == 1)
//	  {
//		  LL_GPIO_TogglePin(GPIOC, LL_GPIO_PIN_1);
//
//		  ICM20602_Get3AxisGyroRawData(&ICM20602.gyro_x_raw);
//
//		  ICM20602.gyro_x = ICM20602.gyro_x_raw * 2000.f / 32768.f;
//		  ICM20602.gyro_y = ICM20602.gyro_y_raw * 2000.f / 32768.f;
//		  ICM20602.gyro_z = ICM20602.gyro_z_raw * 2000.f / 32768.f;
//
//		  printf("%d,%d,%d\n", ICM20602.gyro_x_raw, ICM20602.gyro_y_raw, ICM20602.gyro_z_raw);
//		  //printf("%d,%d,%d\n", (int)(ICM20602.gyro_x*100), (int)(ICM20602.gyro_y*100), (int)(ICM20602.gyro_z*100));
//	  }

//	  if(LPS22HH_DataReady() == 1)
//	  {
//		  LPS22HH_GetPressure(&LPS22HH.pressure_raw);
//		  LPS22HH_GetTemperature(&LPS22HH.temperature_raw);
//
//		  LPS22HH.baroAlt = getAltitude2(LPS22HH.pressure_raw/4096.f, LPS22HH.temperature_raw/100.f);
//
//#define X 0.90f
//		  LPS22HH.baroAltFilt = LPS22HH.baroAltFilt * X + LPS22HH.baroAlt * (1.0f - X);
//
//		  printf("%d,%d\n", (int)(LPS22HH.baroAlt*100), (int)(LPS22HH.baroAltFilt*100));
//	  }

//	  if(m8n_rx_cplt_flag == 1)
//	  {
//		  m8n_rx_cplt_flag = 0;
//
//		  if(M8N_UBX_CHKSUM_Check(&m8n_rx_buf[0], 36) == 1)
//		  {
//			  LL_GPIO_TogglePin(GPIOC, LL_GPIO_PIN_2);
//			  M8N_UBX_NAV_POSLLH_Parsing(&m8n_rx_buf[0], &posllh);
//
//			  printf("LAT: %ld\tLON: %ld\tHeight: %ld\n", posllh.lat, posllh.lon, posllh.height);
//		  }
//	  }

	  if(ibus_rx_cplt_flag == 1)
	  {
		  ibus_rx_cplt_flag = 0;
		  if(iBus_Check_CHKSUM(&ibus_rx_buf[0], 32) == 1)
		  {
			  LL_GPIO_TogglePin(GPIOC, LL_GPIO_PIN_2);
			  HAL_Delay(100);
		  }
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
  /** Initializes the CPU, AHB and APB busses clocks 
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
  /** Initializes the CPU, AHB and APB busses clocks 
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

/* USER CODE END 4 */

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
