#include "main.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_gpio.h"
#include "system_stm32f4xx.h"
#include<string.h>

CRC_HandleTypeDef hcrc;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

#define USER_APP_BASE_ADDR 0x08008004
#define RDP_BASE 0x1FFFC000;

#define GLED_PORT GPIOD
#define GLED_PIN	GPIO_PIN_12
#define BLED_PIN	GPIO_PIN_13
#define BUFFER_LEN 200

uint8_t g_rx_buffer[BUFFER_LEN];

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_CRC_Init(void);
static void MX_USART1_UART_Init(void);

unsigned char error_msg[] = "Invalid Message Received From Host\n";
void bootloader_handle_get_version(uint8_t* rx_buffer);
void bootloader_handle_get_help(uint8_t* rx_buffer);
void bootloader_handle_get_cid(uint8_t* rx_buffer);
void bootloader_handle_get_rdp_status(uint8_t* rx_buffer);
void bootloader_handle_go_to_addr(uint8_t* rx_buffer);
void bootloader_handle_flash_erase(uint8_t* rx_buffer);
void bootloader_handle_mem_write(uint8_t* rx_buffer);
void bootloader_handle_en_rw_protect(uint8_t* rx_buffer);
void bootloader_handle_mem_read(uint8_t* rx_buffer);
void bootloader_handle_read_sector_status(uint8_t* rx_buffer);
void bootloader_handle_otp_read(uint8_t* rx_buffer);
void bootloader_handle_dis_rw_protect(uint8_t* rx_buffer);

/********* COMMON FUNCTIONS *********/
void bootloader_send_ack(uint8_t command_code, uint8_t follow_len);
void bootloader_send_nack(void);
uint8_t bootloader_verify_crc(uint8_t* buffer, uint32_t buffer_len, uint32_t crc_rx_host);

/********* HELPER FUNCTIONS *********/
uint8_t get_bootloader_version(void);
uint8_t isAddressValid(uint32_t address);
uint8_t execute_mem_write(uint8_t* pBuffer, uint32_t mem_address, uint32_t len);

void bootloader_user_read_data(void)
{
	//Packet Contents: Length To Follow | Command Code | so on...
	
	uint8_t rx_len = 0;
	uint8_t command_name;
	
	while(1)
	{
		memset(g_rx_buffer, 0, 200);
		//Read the length of the packet to follow.
		//Here only one byte of data is read.
		HAL_UART_Receive(&huart2, g_rx_buffer , 1, HAL_MAX_DELAY);
		
		//Copy the length of the first byte or 'Length to follow'
		rx_len =  g_rx_buffer[0];
		
		//Now receive remaining bytes starting from the command code as len to follow is already received.
		//rx_len will be total_cmd_len - 1.
		HAL_UART_Receive(&huart2, &g_rx_buffer[1], rx_len, HAL_MAX_DELAY);
		
		//Command Code
		command_name = g_rx_buffer[1];
		
		//So g_rx_buffer has all the data which is passed to the functions.
		switch(command_name)
		{
			case BL_GET_VER:
				bootloader_handle_get_version(g_rx_buffer);
				HAL_GPIO_TogglePin(GLED_PORT, BLED_PIN);
				break;
			case BL_GET_HELP:
				bootloader_handle_get_help(g_rx_buffer);
				break;
			case BL_GET_CID:
				bootloader_handle_get_cid(g_rx_buffer);
				break;
			case BL_GET_RDP_STATUS:
				bootloader_handle_get_rdp_status(g_rx_buffer);
				break;
			case BL_GO_TO_ADDR:
				bootloader_handle_go_to_addr(g_rx_buffer);
				break;
			case BL_FLASH_ERASE:
				bootloader_handle_flash_erase(g_rx_buffer);
				break;
			case BL_MEM_WRITE:
				bootloader_handle_mem_write(g_rx_buffer);
				break;
			case BL_EN_R_W_PROTECT:
				bootloader_handle_en_rw_protect(g_rx_buffer);
				break;
			case BL_MEM_READ:
				bootloader_handle_mem_read(g_rx_buffer);
				break;
			case BL_READ_SECTOR_STATUS:
				bootloader_handle_read_sector_status(g_rx_buffer);
				break;
			case BL_OTP_READ:
				bootloader_handle_otp_read(g_rx_buffer);
				break;
			case BL_DIS_R_W_PROTECT:
				bootloader_handle_dis_rw_protect(g_rx_buffer);
				break;
			default:
					break;
		}
	}
}

unsigned char first_msg[] = "Start Of Bootloader Jump\n";
unsigned char stack_msg[] = "Copying RAM Address -> MSP\n";
unsigned char third_msg[] = "Copying the reset handlers address\n";
unsigned char fourth_msg[] = "Calling reset function of user application\n";

void bootloader_jump_to_user_app(void)
{
	HAL_UART_Transmit(&huart2, first_msg, sizeof(first_msg), 10);
	HAL_UART_Transmit(&huart2, stack_msg, sizeof(stack_msg), 10);
	
	uint32_t mspValue = *(volatile uint32_t*)USER_APP_BASE_ADDR;
	
	HAL_UART_Transmit(&huart2, third_msg, sizeof(third_msg), 10);
	
	uint32_t reset_handler_address = *(volatile uint32_t*)(USER_APP_BASE_ADDR);
	HAL_UART_Transmit(&huart2, (unsigned char*)&reset_handler_address, sizeof(reset_handler_address), HAL_MAX_DELAY);
	void (*reset_handler_funcPtr)(void);
	
	HAL_UART_Transmit(&huart2, fourth_msg, sizeof(fourth_msg), 10);
	reset_handler_funcPtr = (void*)reset_handler_address;
	
	//Jump to reset handler which will call the main function of user application.
	reset_handler_funcPtr();
}

int main(void)
{
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
  
	/* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_CRC_Init();
  MX_USART1_UART_Init();
  
	
  if(HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == GPIO_PIN_RESET) //If button not pressed
	{
		bootloader_user_read_data();
		
	}
	else //if button pressed
	{
		bootloader_jump_to_user_app();
	}
}

void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 8;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* CRC init function */
static void MX_CRC_Init(void)
{

  hcrc.Instance = CRC;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

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
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}


static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin 
                          |Audio_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PE2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : CS_I2C_SPI_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_I2C_SPI_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PE4 PE5 MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PDM_OUT_Pin */
  GPIO_InitStruct.Pin = PDM_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(PDM_OUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : I2S3_WS_Pin */
  GPIO_InitStruct.Pin = I2S3_WS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(I2S3_WS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI1_SCK_Pin SPI1_MISO_Pin SPI1_MOSI_Pin */
  GPIO_InitStruct.Pin = SPI1_SCK_Pin|SPI1_MISO_Pin|SPI1_MOSI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : CLK_IN_Pin PB12 */
  GPIO_InitStruct.Pin = CLK_IN_Pin|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin 
                           Audio_RST_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin 
                          |Audio_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : I2S3_MCK_Pin I2S3_SCK_Pin I2S3_SD_Pin */
  GPIO_InitStruct.Pin = I2S3_MCK_Pin|I2S3_SCK_Pin|I2S3_SD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Audio_SDA_Pin */
  GPIO_InitStruct.Pin = Audio_SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(Audio_SDA_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
uint8_t get_bootloader_version(void)
{
	return BL_VER;
}

void bootloader_handle_get_version(uint8_t *rx_buffer)
{
	unsigned char init_msg[] = "Inside get version function\n";
	unsigned char crc_pass[] = "CRC Is correct\n";
	unsigned char crc_fail[] = "CRC is wrong\n";
	
	uint8_t cmd_pkt_len = g_rx_buffer[0] + 1;
	
	uint32_t crc_rx_host = *((uint32_t*)(g_rx_buffer + cmd_pkt_len - 4));
	
	//IF CRC IS CORRECT
	if(!(bootloader_verify_crc(&g_rx_buffer[0], cmd_pkt_len - 4, crc_rx_host)))
	{
		//Send ack + follow length back: bootloader_send_ack(uint8_t command_code, uint8_t follow_len);
		uint8_t pkts_to_send_after_ack = 1;
		bootloader_send_ack(g_rx_buffer[0], pkts_to_send_after_ack);
		HAL_GPIO_TogglePin(GLED_PORT, GLED_PIN);
		uint8_t bl_ver = get_bootloader_version();
		
		//Once it sends ack, now send required data back to bootloader.
		HAL_UART_Transmit(&huart2, &bl_ver, sizeof(bl_ver), HAL_MAX_DELAY);
		
	}
	else //CRC IS WRONG
	{
		bootloader_send_nack();
		
	}
	
}

uint8_t supported_cmd[] = {0x51, 0x52, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59, 0x5A};

void bootloader_handle_get_help(uint8_t* rx_buffer)
{
	uint32_t rx_len = 0;
	uint32_t crc_rx_host = 0;
	
	rx_len = g_rx_buffer[0] + 1;
	
	crc_rx_host = *((uint32_t*)(g_rx_buffer + rx_len - 4)); //start address + (32*6 - 32*4) = start_address + (32*2)
	//HAL_GPIO_TogglePin(GLED_PORT, GLED_PIN);
	
	if(!(bootloader_verify_crc(&g_rx_buffer[0],(rx_len - 4), crc_rx_host)))
	{
		bootloader_send_ack(g_rx_buffer[0], sizeof(supported_cmd));
		HAL_UART_Transmit(&huart2, supported_cmd, sizeof(supported_cmd), HAL_MAX_DELAY);
	}
	else
	{
		bootloader_send_nack();
		HAL_GPIO_TogglePin(GLED_PORT, BLED_PIN);
	}
}

void bootloader_handle_get_cid(uint8_t* rx_buffer)
{

	uint32_t rx_len = g_rx_buffer[0] + 1;
	//Checksum
	uint32_t checksum_rx_host = *(uint32_t*)(g_rx_buffer + rx_len - 4);
	
	if(!(bootloader_verify_crc(&g_rx_buffer[0], rx_len - 4, checksum_rx_host)))
	{
		uint16_t dev_id = (uint16_t)DBGMCU->IDCODE & 0xFFF;
		bootloader_send_ack(g_rx_buffer[0], sizeof(dev_id));	
		HAL_UART_Transmit(&huart2, (uint8_t*)&dev_id, sizeof(dev_id), HAL_MAX_DELAY);
	}
	else
	{
		bootloader_send_nack();
	}
}

void bootloader_handle_get_rdp_status(uint8_t* rx_buffer)
{
	uint32_t rx_len = g_rx_buffer[0] + 1;
	
	uint32_t checksum_rx_host = *(uint32_t*)(g_rx_buffer + rx_len - 4);
	
	if(!(bootloader_verify_crc(&g_rx_buffer[0], rx_len - 4, checksum_rx_host)))
	{
		//send acknowledegment
		
		//Convert it to proper type.
		volatile uint32_t* rdp_32 = (uint32_t*)RDP_BASE;
		
		//Value is present at 8 - 15. So, shift the value at RDP_BASE by 8.
		uint8_t rdp_val = (uint8_t)(*rdp_32 >> 8);
		
		//Transmit Acknowledgement
		bootloader_send_ack(g_rx_buffer[0], 1);
		
		//Transmit Data.
		HAL_UART_Transmit(&huart2, &rdp_val, 1, HAL_MAX_DELAY);
		HAL_GPIO_TogglePin(GLED_PORT, BLED_PIN);
	}
	else
	{ 
		bootloader_send_nack();
	}
	
}

void bootloader_handle_go_to_addr(uint8_t* rx_buffer)
{
	uint32_t rx_len = g_rx_buffer[0] + 1;
	uint32_t checksum_rx_host = *(uint32_t*)(g_rx_buffer + rx_len - 4);
	
	if(!(bootloader_verify_crc(&g_rx_buffer[0], rx_len - 4, checksum_rx_host)))
	{
		//Send ack
		bootloader_send_ack(g_rx_buffer[0], 4);
		
		uint32_t jump_address = *(uint32_t*)(&g_rx_buffer[2]); 
		
		//Does it send back anything.??
		HAL_UART_Transmit(&huart2, (uint8_t*)(&jump_address), 4, HAL_MAX_DELAY);
		
		if(! (isAddressValid(jump_address)))
		{
			HAL_GPIO_TogglePin(GLED_PORT, GLED_PIN);
			
			uint32_t jump_addr = *(uint32_t*)0x080008004;
			void (*vectorTablePtr)(void) = (void*)jump_address;
			vectorTablePtr();
		}
		else
		{
			HAL_GPIO_TogglePin(GLED_PORT, BLED_PIN);
		}
	}
	else
	{
			bootloader_send_nack();
	}
}

void bootloader_handle_flash_erase(uint8_t* rx_buffer)
{
	//Refer to hal_flash.c
	//Basic Things To Be Checked
	//Type of erase: sector or mass
	//What are banks in flash memory??
	//Starting sector from which to erase
	//No of sectors to erase
	//Voltage Range: 
	
	uint32_t rx_len = g_rx_buffer[0] + 1;
	uint32_t checksum_rx_host = *(uint32_t*)(g_rx_buffer + rx_len - 4);
	
	if(!bootloader_verify_crc(&g_rx_buffer[0], rx_len - 4, checksum_rx_host))
	{
		bootloader_send_ack(g_rx_buffer[0], 1);
		
		//Format : LTO | Command Code | Sector Number | No Of Sectors
		uint8_t sector_number = g_rx_buffer[2];
		uint8_t no_of_sectors = g_rx_buffer[3];
		
		//Here flashing process starts.
		
		FLASH_EraseInitTypeDef flash_handle;
		if(sector_number == 0xff) //mass erase
		{
			HAL_GPIO_TogglePin(GLED_PORT, GLED_PIN);
			flash_handle.TypeErase = FLASH_TYPEERASE_MASSERASE;
		}
		else
		{
			flash_handle.TypeErase = FLASH_TYPEERASE_SECTORS;
			
			//Check remaining sectors
			uint8_t rem_sectors = 8 - sector_number;
			if(no_of_sectors > rem_sectors)
			{
					no_of_sectors = rem_sectors;
			}
			
			flash_handle.Sector = sector_number;
			flash_handle.NbSectors = no_of_sectors;
		}
		flash_handle.Banks = FLASH_BANK_1;
		
		//Unlocking the flash is enabling it for user access., set 2 keys.
		HAL_FLASH_Unlock();
		flash_handle.VoltageRange = FLASH_VOLTAGE_RANGE_3;
		uint32_t sec_err;
		
		//Erase Flash
		uint8_t flash_op_status = HAL_FLASHEx_Erase(&flash_handle, &sec_err);

		//Lock Flash Again
		HAL_FLASH_Lock();
		HAL_GPIO_TogglePin(GLED_PORT, GLED_PIN);
		
		HAL_UART_Transmit(&huart2, &flash_op_status, 1, HAL_MAX_DELAY);
	}
	else
	{
		bootloader_send_nack();
	}
}


 
void bootloader_handle_mem_write(uint8_t* rx_buffer)
{
	//LOW | Command Code | Base Memory Address | Payload Length | Payload | CRC
	uint32_t total_len = g_rx_buffer[0] + 1;
	
	//Check CRC
	uint32_t checksum = *(uint32_t*)(g_rx_buffer + total_len - 4);
	
	if(!bootloader_verify_crc(&g_rx_buffer[0], total_len - 4, checksum))
	{
		//Send ack
		bootloader_send_ack(g_rx_buffer[0], 1); 
		HAL_GPIO_TogglePin(GLED_PORT, GLED_PIN);
		
		//Starting Memory Address Of Flash Memory
		uint32_t mem_addr_start = *(uint32_t*)(g_rx_buffer + 2);
		uint8_t payload_len = g_rx_buffer[6];
		uint8_t status = HAL_OK;
		
		//Start Index Of Payload
		uint32_t start_address = 7; 
				
		//Assuming Address is validated.
		HAL_FLASH_Unlock();
	  for(uint32_t i = 0; i < payload_len; i++)
		{
				status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, mem_addr_start + i, g_rx_buffer[7 + i]);
		}
		HAL_FLASH_Lock();
		
	}
	else
	{
		bootloader_send_nack();
	}
}

uint8_t isAddressValid(uint32_t address)
{
	//0x2000 0000 - 0x2002 0000
  //0x0800 0000 - 0x0807 FFFF

	//Allowed memory addresses: SRAM, FLASH, BACKUP SRAM, PERIPHERAL(NO), EXTERNAL MEMORY(YES).
	//Not implementing as it is easy.
	//No worries.
	//Crux of this function is to write which memories are writable.
	return ADD_VALID;
}
void bootloader_send_ack(uint8_t follow_len_byte, uint8_t first_field_len)
{
	//CRC : verified
	//Send ack + length which we received from the host for verification.
	uint8_t ack_msg[2];
	
	ack_msg[0] = 0xA5;
	ack_msg[1] = first_field_len;
	HAL_UART_Transmit(&huart2, ack_msg, sizeof(ack_msg), HAL_MAX_DELAY);
}

void bootloader_send_nack(void)
{
	//As CRC Is not verified, NACK WIll just consist of one value assigned to it.
	uint8_t nack = (uint8_t)BL_NACK;
	HAL_UART_Transmit(&huart2, &nack, sizeof(uint8_t), HAL_MAX_DELAY);
}

uint8_t bootloader_verify_crc(uint8_t* buffer, uint32_t buffer_len, uint32_t crc_rx_host)
{
		uint32_t crc_accumulate = 0x00;
		
		for(uint32_t i = 0; i < buffer_len; i++)
		{
			crc_accumulate += buffer[i];
		}
		
		if(crc_accumulate == crc_rx_host)
		{
			return VERIFY_CRC_SUCCESS;
		}
		return VERIFY_CRC_FAILURE;
		
}
