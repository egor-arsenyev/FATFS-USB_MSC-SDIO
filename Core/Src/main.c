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
#include "adc.h"
#include "dma.h"
#include "fatfs.h"
#include "sdio.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_storage_if.h"
#include "usb_device.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ADC_TO_VOLTAGE 0.00080564f
#define START_ADC 1
#define STOP_ADC  0
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile uint8_t  flag_adc_cplt = 0;
volatile uint32_t adc_conv_num = 0;
volatile uint16_t adc_val = 0;
volatile uint8_t  adc_state = STOP_ADC;
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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_TIM8_Init();
  MX_TIM3_Init();
  MX_SDIO_SD_Init();
  /* USER CODE BEGIN 2 */
  MX_USB_DEVICE_Init();
  USBD_Stop(&hUsbDeviceFS);
  MX_FATFS_Init();

  HAL_Delay(1000);
  /* Register the file system object to the FatFs module */
  retSD = (f_mount(&SDFatFS, (TCHAR const*)SDPath, 0) != FR_OK);
  if(retSD != FR_OK){
  /* FatFs Initialization Error */
	  Error_Handler();
  }

  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
  HAL_TIM_Encoder_Start_IT(&htim2, TIM_CHANNEL_1);
//  HAL_ADC_Start_IT(&hadc1);
  HAL_TIM_Base_Start(&htim3);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if(adc_state == START_ADC){
		  /* Stop the USB Device Core */
		  USBD_Stop(&hUsbDeviceFS);
		  /* Create and Open a new text file object with write access */
		  retSD = f_open(&SDFile, "ADC_DATA.csv", FA_CREATE_ALWAYS | FA_WRITE);
		  if(retSD != FR_OK){
	      /* 'ADC_DATA.csv' file Open for write Error */
			  Error_Handler();
		  }
		  /* Writing ADC conversion results to file */
		  while ((adc_conv_num < 10000)&&(adc_state == START_ADC)){
			  if (flag_adc_cplt){
				  static char str_val[20];
				  sprintf (str_val, "%lu; %f", (adc_conv_num*100), (float)(adc_val*ADC_TO_VOLTAGE));
				  f_printf(&SDFile, "%s\n", str_val);
				  flag_adc_cplt = 0;
				  adc_conv_num++;
			  }
		  }
		  /* Close the open text file */
		  f_close(&SDFile);
		  adc_conv_num = 0;
		  adc_state = STOP_ADC;
		  /* Start the USB Device Core */
		  USBD_Start(&hUsbDeviceFS);
	  }

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
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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

/* USER CODE BEGIN 4 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim){
	static uint8_t cnt = 0;
	cnt = __HAL_TIM_GET_COUNTER(&htim2);
	TIM8->CCR1 = cnt>>1;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
	adc_val = HAL_ADC_GetValue(&hadc1);
	flag_adc_cplt = 1;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if (adc_state == STOP_ADC){
		HAL_ADC_Start_IT(&hadc1);
		adc_state = START_ADC;
	}
	else{
		HAL_ADC_Stop_IT(&hadc1);
		adc_state = STOP_ADC;
		}
}
/*
int _write(int file, char *ptr, int len){
	static uint8_t usb_flag = USBD_OK;

	do{
		CDC_Transmit_FS((uint8_t*)ptr, len);
	}while(USBD_BUSY == usb_flag);
	return len;
}
*/
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
