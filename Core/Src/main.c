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
#include "fatfs.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "W25Qxx.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define FW_ADDR                                0x08020000
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */
static void CRIPTO(uint8_t* data, uint32_t size);
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
	FATFS fs;
	volatile FRESULT res, readres;
	FILINFO fileInfo;
	DIR dir;
	
	bool loadFile = 0;

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  //MX_USB_DEVICE_Init();
  MX_FATFS_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
	
	W25qxx_Init();
	
	HAL_Delay(1000);

	
	// РџСЂРѕРІРµСЂРєР° РЅР° СЂРµР¶РёРј Р±СѓС‚Р»РѕР°РґРµСЂР°
	
	if (HAL_GPIO_ReadPin(GPIOB, 0b111))
	{
		
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);
		res = f_mount(&fs, "", 0);	
		f_opendir(&dir, "");
		loadFile = 1;
	
	}
	else 
	{
		HAL_Delay(100);
		MX_USB_DEVICE_Init();
		HAL_Delay(100);
	}
	
	

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
		if (loadFile)
		{
			
		
			res = f_readdir(&dir, &fileInfo);

			if (res == FR_OK && fileInfo.fname[0])

			{
		  
				static const uint8_t constName[13] = { 'T', 'C', '.', 'F', 'i', 'r', 'm', '.', 'D', '.', 'b', 'i', 'n' };   
				static const uint8_t constNameHex[13] = { 'T', 'L', 'C', '.', 'h', 'e', 'x', };  
				uint8_t tmpFileInfo[23];																			  
			
				// РџСЂРёРІРµРґРµРЅРёРµ РїРѕР»СѓС‡РµРЅРЅРѕРіРѕ Р·Р°РіРѕР»РѕРІРєР° С„Р°Р№Р»Р° РґР»СЏ РїСЂРѕРІРµСЂРєРё РЅР° СЃРѕРѕС‚РІРµС‚СЃРІРёРµ
		  
				memcpy(tmpFileInfo, fileInfo.fname, 23);
		  
				tmpFileInfo[9] = tmpFileInfo[19];
				tmpFileInfo[10] = tmpFileInfo[20];
				tmpFileInfo[11] = tmpFileInfo[21];
				tmpFileInfo[12] = tmpFileInfo[22];
		  
				for (int i = 13; i <= 22; i++)
				{
					tmpFileInfo[i] = 0;
				}
		  
		  
				if (!strcmp(constName, tmpFileInfo))
				{
					// РќРµРѕР±С…РѕРґРёРјС‹Р№ С„Р°Р№Р» РЅР°Р№РґРµРЅ
			  
					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET); // Р?РЅРґРёРєР°С‚РѕСЂ РЅР°Р№РґРµРЅРЅРѕРіРѕ С„Р°Р№Р»Р°
					
					volatile HAL_StatusTypeDef res1 = HAL_FLASH_Unlock();
				
				
					FIL firmware;
					uint8_t firmBuffer[64];
					unsigned int firmBytes = 0;
					volatile unsigned int byteCounter = 0;
				
					FLASH_EraseInitTypeDef eraseInit;
					uint sectorsErorr = 0;
				
					eraseInit.Sector = 5; // РЎС‚РёСЂР°РµРј СЃ 5-РіРѕ СЃРµРєС‚РѕСЂР°, СЌС‚Рѕ 0x08020000
					eraseInit.TypeErase = FLASH_TYPEERASE_SECTORS; // РЎС‚РёСЂР°РµРј РїРѕ СЃРµРєС‚РѕСЂР°Рј
					eraseInit.NbSectors = 1 + (fileInfo.fsize / 131072); // РљРѕР»РёС‡РµСЃС‚РІРѕ СЃРµРєС‚РѕСЂРѕРІ   cc
				
				
					volatile HAL_StatusTypeDef res =  HAL_FLASHEx_Erase(&eraseInit, &sectorsErorr);
				
					
					
					f_open(&firmware, (char*)fileInfo.fname, FA_READ);
					
					// Р Р°СЃС€РёС„СЂРѕРІС‹РІР°С‚СЊ СЃС‚СЂРѕРєРё РїРѕ 64 Р±Р°Р№С‚Р° РїРѕРєР° СЂР°Р·РјРµСЂ С„Р°Р№Р»Р° РЅРµ СЃРѕРІРїР°РґРµС‚
					while (byteCounter < fileInfo.fsize)
					{
						f_read(&firmware, firmBuffer, 64, &firmBytes);							
						//CRIPTO(firmBuffer, 64);
					
					
						for (int byte = 0; byte < 64; byte++)
						{
							volatile HAL_StatusTypeDef vres = HAL_FLASH_Program(TYPEPROGRAM_BYTE, FW_ADDR + byteCounter + byte, firmBuffer[byte]);
							HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_2);
					
						}
					
				
					
					
						byteCounter += firmBytes;
					}
				
					HAL_FLASH_Lock();
					f_close(&firmware);
					//f_unlink((char*)fileInfo.fname);
					
					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET); // Р?РЅРґРёРєР°С‚РѕСЂ РЅР°Р№РґРµРЅРЅРѕРіРѕ С„Р°Р№Р»Р° dd
				
					typedef void(*pFunction)(void);
				
					HAL_Delay(10);
					__disable_irq();
				
					
				
					uint32_t jumpAddress = *(__IO uint32_t*)(FW_ADDR + 4);
					pFunction Jump_To_Application = (pFunction) jumpAddress;

					SCB->VTOR = *(__IO uint32_t*)FW_ADDR;
					
							
					
					__set_MSP(*(__IO uint32_t*) FW_ADDR);
					
					Jump_To_Application();
				
				
					while (1) {}
					;
				
				
				}
		 
		 
			
		
			
			
			}

			else break; 
	  
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
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 192;
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
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC0 PC1 PC2 PC3 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
static void CRIPTO(uint8_t* data, uint32_t size)
{
	for (uint32_t i = 0; i < size; i += 2)
	{
		data[i] = data[i] ^ 'T' ^ data[i + 1];
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
