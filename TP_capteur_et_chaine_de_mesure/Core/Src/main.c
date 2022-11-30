/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "Const.h"
#include "stdio.h"
#include <math.h>
#include "function.h"
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

UART_HandleTypeDef hlpuart1;

/* USER CODE BEGIN PV */

char mess1[30];
double Temperature = 0;
double Acceleration[3] = {0};
double norme_vecteur_gravite = 0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int __io_putchar(int ch) {
HAL_UART_Transmit(&hlpuart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
return ch;
}

/**
 	 * @brief Init I2C1
 	 * @Note Cette fonction met à 1 le bit 7 de PWR_MGMT_1 pour faire un reset de l'ensemble des registres du capteur, puis attend
	 * 100ms et choisit une horlorge
	 * @param p_hi2c1 Pointeur vers une structure I2C qui contient l'information de configuration pour un i2c particulier
	 * @retval None
	 */
void Init(I2C_HandleTypeDef* p_hi2c1)
{

	uint8_t buff[6];
	buff[0] = 0x80;
	HAL_I2C_Mem_Write ( p_hi2c1, MPU_ADD,  PWR_MGMT_1,  1, &buff[0], 1, 10);
	HAL_Delay(100);
	buff[0] = 0x1;
	HAL_I2C_Mem_Write ( p_hi2c1, MPU_ADD, PWR_MGMT_1,  1, &buff[0], 1, 10);


	buff[0] =0x00;  // changement de la sensibilité de l'accélérometre  00=2g 10=4g 01=8g 11=16g
		HAL_I2C_Mem_Write ( p_hi2c1, MPU_ADD,  ACCEL_CONFIG,  1, &buff[0], 1, 10);


	buff[0]=0x2;
		  if(HAL_I2C_Mem_Write(p_hi2c1,MPU_ADD,INT_PIN_CFG,1,&buff[0],1,10)!=HAL_OK){
		  	  Error_Handler();
		  }
	buff[0]=0x16;
		 	  if(HAL_I2C_Mem_Write(p_hi2c1,MAGNETO_ADD,AK8963_CNTL,1,&buff[0],1,10)!=HAL_OK){
		 	  	  Error_Handler();

		 	  }
	buff[0]=0x3;
	if(HAL_I2C_Mem_Write(p_hi2c1,MPU_ADD,CONFIG,1,buff,1,10)!=HAL_OK){
		Error_Handler();

	}
	buff[0]=0xFF;
	if(HAL_I2C_Mem_Write(p_hi2c1,MPU_ADD,SMPLRT_DIV,1,buff,1,10)!=HAL_OK){
		Error_Handler();

	}
}


/**
 	 * @brief Mesure temperature
 	 * @Note Cette fonction permet de lire la temperature du capteur puis de l'enregistrer dans une variable globale de type double
	 * @param p_hi2c1 Pointeur vers une structure I2C qui contient l'information de configuration pour un i2c particulier.
	 * @param Temperature Pointeur vers une zone mémoire de type double contenant l’information de température
	 * @retval None
	 */

void Measure_T(I2C_HandleTypeDef* p_hi2c1, double* Temperature)
{
	uint8_t buff[6];
	HAL_I2C_Mem_Read ( p_hi2c1, MPU_ADD, TEMP_OUT_H, 1, &buff[0], 1, 10);
	HAL_I2C_Mem_Read ( p_hi2c1, MPU_ADD, TEMP_OUT_L, 1, &buff[1], 1, 10);

	uint16_t Temp = 0;
	Temp = ((buff[0]<< 8) + buff[1]);

	*Temperature = ((Temp - ROOM_TEMP_OFFSET)/TEMP_SENS) + 21;

}


/**
 	 * @brief Mesure acceleration
 	 * @Note Cette fonction permet de lire l'acceleration du capteur sur les axes x,y et z puis de l'enregistrer dans une variable globale de type double
	 * @param p_hi2c1 Pointeur vers une structure I2C qui contient l'information de configuration pour un i2c particulier
	 * @param Acceleration Pointeur vers une zone mémoire de type double contenant l’information de température
	 * @retval None
	 */

void Measure_A(I2C_HandleTypeDef* p_hi2c1,double* Acceleration){
	uint8_t buff[6];
	HAL_I2C_Mem_Read ( p_hi2c1, MPU_ADD, ACCEL_XOUT_H, 1, &buff[0], 6, 10);


	uint16_t ax = 0;
		ax = ((buff[0]<< 8) + buff[1]);

	uint16_t ay = 0;
		ay = ((buff[2]<< 8) + buff[3]);

	uint16_t az = 0;
		az = ((buff[4]<< 8) + buff[5]);

	Acceleration[0] = ax*9.81*2/32767.0;
	Acceleration[1] = ay*9.81*2/32767.0;
	Acceleration[2] = az*9.81*2/32767.0;
}


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
  MX_LPUART1_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  int x=0;
  int i=0;
  int Devices[100]={0};

 uint8_t pData[100] = {0};

 pData[0]=0x75;
  if(HAL_I2C_Master_Transmit(&hi2c1, MPU_ADD, pData, 1, HAL_MAX_DELAY) != HAL_OK )
  {
	  printf("il y a une erreur avec I2C Master Transmit \r\n");
  }
  if(HAL_I2C_Master_Receive(&hi2c1, MPU_ADD, pData, 1, HAL_MAX_DELAY) != HAL_OK )
  {
	  printf("il y a une erreur avec I2C Master Receive \r\n");
  }


  printf(" L'identifiant du capteur est : %x \r\n", pData[0]);
  if((pData[0] =! 0x71))
  {
	  printf("ce n'est pas le bon capteur \r\n");
  }
  else
	  printf("MPU-9250 identified \r\n");


  for (i=0;i<256;i++){
       	  if(HAL_I2C_IsDeviceReady(&hi2c1, i, 4, 20)==HAL_OK){
       		  Devices[x]=i;
       		  printf("%d \r\n",Devices[x]);
       		  x=x+1;
       	  }

         }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

	  Measure_T(&hi2c1, &Temperature);
	  printf("Temperature = %.2f C \r\n", Temperature);

	  HAL_Delay(1000);

	  Measure_A(&hi2c1, Acceleration);
	  printf("Acceleration selon x = %.4f m/s2 \r\n", Acceleration[0]);
	  HAL_Delay(100);
	  printf("Acceleration selon y = %.4f m/s2 \r\n", Acceleration[1]);
	  HAL_Delay(100);
	  printf("Acceleration selon z = %.4f m/s2 \r\n", Acceleration[2]);
	  HAL_Delay(100);
	  norme_vecteur_gravite = sqrt(Acceleration[0]/9.81*Acceleration[0]/9.81+Acceleration[1]/9.81*Acceleration[1]/9.81+Acceleration[2]/9.81*Acceleration[2]/9.81);
	  printf("norme du vecteur gravite = %.4f \r\n", norme_vecteur_gravite);
	  HAL_Delay(1000);
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
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
  hi2c1.Init.Timing = 0x10802D9B;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief LPUART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 115200;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&hlpuart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&hlpuart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */

  /* USER CODE END LPUART1_Init 2 */

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

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
  __disable_irq();
  while (1)
  {
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
