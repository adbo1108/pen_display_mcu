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
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_custom_hid_if.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define EMR_I2C_ADDR_7BIT 0x09
#define EMR_I2C_PACKET_SIZE 8
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t EMR_INT = 0 ;
extern I2C_HandleTypeDef hi2c3;
extern UART_HandleTypeDef huart6;
extern GPIO_InitTypeDef GPIO_InitStruct ;
uint8_t pData[EMR_I2C_PACKET_SIZE];
uint8_t aRxBuffer[5];
uint8_t send_data[9];
uint8_t i ;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void Handle_EMR_Data();

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int fputc(int32_t ch, FILE *f)
{
  //  while((USART6->SR&0X40)==0);
   // USART6->DR = (uint8_t) ch;  
	HAL_UART_Transmit(&huart6,(uint8_t *)&ch,1,1000);
        
    return ch;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    uint8_t data[5];
    if(huart->Instance==USART6)
    {
        data[0] = aRxBuffer[0];
		 memcpy(data,aRxBuffer,5);
		 memset(aRxBuffer,0,5);
        HAL_UART_Transmit(&huart6,(uint8_t*)data,5,1000);
        
    }
	 if(huart->Instance==USART2)
    {
        data[0] = aRxBuffer[0];
		 memcpy(data,aRxBuffer,5);
		 memset(aRxBuffer,0,5);
        HAL_UART_Transmit(&huart2,(uint8_t*)data,5,1000);
        
    }
}

void Handle_EMR_Data ()
{
	if(EMR_INT)
	{	
		HAL_StatusTypeDef status;
		uint8_t i ;
		status =  HAL_I2C_Master_Receive(&hi2c3, EMR_I2C_ADDR_7BIT, pData, EMR_I2C_PACKET_SIZE, 500);
		if(status == HAL_ERROR)
		{
			printf("i2c HAL_ERROR\n\r");
		}
		else if(status == HAL_BUSY)
		{
			printf("i2c HAL_BUSY\n\r");
		}
		else if(status == HAL_TIMEOUT)
		{
			printf("i2c HAL_TIMEOUT\n\r");
		}	
		else if (status == HAL_OK){
			printf("receive EMR Data \n\r");
			for(i=0;i<EMR_I2C_PACKET_SIZE ; i++ )
			printf("0x%x\n\r",pData[i]);
		}
		EMR_INT = 0 ;
	}	
	
}


/* EMR Interrupt */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == GPIO_PIN_4)
	{
		EMR_INT = 1;
	}		
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
  MX_I2C3_Init();
  MX_USART2_UART_Init();
  MX_USART6_UART_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
 HAL_UART_Receive_IT(&huart6,aRxBuffer, sizeof(aRxBuffer)); //�ϯ౵�����_
 HAL_UART_Receive_IT(&huart2,aRxBuffer, sizeof(aRxBuffer)); //�ϯ౵�����_
	printf("stm32 mcu init done V3\n\r");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  
  HAL_GPIO_WritePin(TP160_GPIO_Port, TP160_Pin, GPIO_PIN_RESET);
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  
#if 1	  
	   HAL_Delay(1);
	//  printf("stm32 mcu init done\n\r");
	 // printf("boot test \n\r");
		//HAL_UART_Transmit(&huart6,pData,1,1000);
	//  HAL_GPIO_WritePin(TP160_GPIO_Port, TP160_Pin, GPIO_PIN_SET);
	 // HAL_GPIO_TogglePin(TP160_GPIO_Port, TP160_Pin);
#endif
	  
#if 1
	  // assign Report ID
   send_data[0]=0x02;
   for(i=1;i<9;i++)
   {
      send_data[i]=0x01;
   }    
        
   USBD_CUSTOM_HID_SendReport_FS(send_data,9) ;
#endif	  
	//	Handle_EMR_Data () ;
	  
	  
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
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
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
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
	printf("e");
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
