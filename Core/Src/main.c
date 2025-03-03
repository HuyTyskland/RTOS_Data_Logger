/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include<stdio.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "string.h"
#include "bme680.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define hum_os BME68X_OS_16X
#define pres_os BME68X_OS_1X
#define temp_os BME68X_OS_2X
#define iir_filter BME68X_FILTER_OFF
#define odr_time BME68X_ODR_NONE

#define htr_enable BME68X_ENABLE
#define htr_dur BME68X_HEATR_DUR1
#define htr_temp BME68X_HIGH_TEMP

#define max_number_of_data_fail 10

#define DWT_CTRL (*(volatile uint32_t*)0xE0001000)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
const char *begin_msg = "Data Logger begins!\n";
const char *init_fail = "Init function fail\n";
const char *init_suc = "Init function success\n";
const char *conf_failt = "Configuration function fail\n";
const char *conf_suc = "Configuration function success\n";
const char *htr_conf_fail = "Heater configuration fail\n";
const char *htr_conf_suc = "Heater configuration success\n";
const char *op_mode_fail = "Setting Op Mode fail\n";
const char *op_mode_suc = "Setting Op Mode success\n";
const char *rd_dt_fail = "Data reading fail\n";
const char *rd_dt_suc = "Data reading success\n";

const char *process_fail = "Processed Data is corrupted!\n";

uint8_t n_fields;

struct bme68x_conf sensor_conf;
struct bme68x_heatr_conf htr_conf;

TaskHandle_t sensor_handle;
TaskHandle_t processing_handle;

struct bme68x_dev bme;
struct bme68x_data data;

QueueHandle_t measurement_data_queue;
QueueHandle_t display_information_queue;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
static void bme680_interface_init(struct bme68x_dev*);
static int8_t bme680_config(struct bme68x_dev*);
static int8_t bme680_htr_config(struct bme68x_dev*);

static void sensor_handler(void* parameters);
static void processing_handler(void* parameters);
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
	BaseType_t status;
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
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  int8_t result;

  //// Set up sensor
  // Initialize the interface
  bme680_interface_init(&bme);
  // initialize
  result = bme68x_init(&bme);
	print_error(result);
	if(result != BME68X_OK)
	{
		printmsg(init_fail);
	}
	else
	{
		printmsg(init_suc);
	}
  // configure oversampling
  result = bme680_config(&bme);
  if(result != BME68X_OK)
  {
  	printmsg(conf_failt);
  }
	else
	{
		printmsg(conf_suc);
	}
  // configure heater
  result = bme680_htr_config(&bme);
  if(result != BME68X_OK)
  {
  	printmsg(htr_conf_fail);
  }
	else
	{
		printmsg(htr_conf_suc);
	}

  //// Set up task
  DWT_CTRL |= ( 1 << 0 );

  status = xTaskCreate(sensor_handler, "Sensor_Task", 200, NULL, 2, &sensor_handle);
  configASSERT(status == pdPASS);

  status = xTaskCreate(processing_handler, "Processing_Task", 200, "PROCESSING", 2, &processing_handle);
  configASSERT(status == pdPASS);

  // set up queue
  measurement_data_queue = xQueueCreate(10, sizeof(size_t));
  configASSERT(measurement_data_queue != NULL);
  display_information_queue = xQueueCreate(10, sizeof(char));
  configASSERT(display_information_queue != NULL);
  // start scheduler
  vTaskStartScheduler();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 50;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
  huart2.Init.BaudRate = 115200;
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

  /*Configure GPIO pin : CS_I2C_SPI_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_I2C_SPI_GPIO_Port, &GPIO_InitStruct);

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

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

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
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT1_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CLK_IN_Pin */
  GPIO_InitStruct.Pin = CLK_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(CLK_IN_GPIO_Port, &GPIO_InitStruct);

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

  /*Configure GPIO pin : VBUS_FS_Pin */
  GPIO_InitStruct.Pin = VBUS_FS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(VBUS_FS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : OTG_FS_ID_Pin OTG_FS_DM_Pin OTG_FS_DP_Pin */
  GPIO_InitStruct.Pin = OTG_FS_ID_Pin|OTG_FS_DM_Pin|OTG_FS_DP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MEMS_INT2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void printmsg(const char* msg)
{
	HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
}

BME68X_INTF_RET_TYPE user_i2c_read(uint8_t reg_addr, uint8_t *data, uint32_t len, void *intf_ptr)
{
	uint8_t dev_addr = *(uint8_t*)intf_ptr;
	uint16_t DevAddress = dev_addr <<1;
//  HAL_I2C_Master_Transmit(&hi2c1, DevAddress, &reg_addr, 1, 1000);
//  HAL_I2C_Master_Receive(&hi2c1, DevAddress, data, len, 1000);
	HAL_I2C_Mem_Read(&hi2c1, DevAddress, reg_addr, 1, data, len, 15);
  return 0; // Success
}

BME68X_INTF_RET_TYPE user_i2c_write(uint8_t reg_addr, const uint8_t *data, uint32_t len, void *intf_ptr)
{
	uint8_t dev_addr = *(uint8_t*)intf_ptr;
	uint16_t DevAddress = dev_addr <<1;
//	HAL_I2C_Master_Transmit(&hi2c1, DevAddress, &reg_addr, len+1, 1000);
	HAL_I2C_Mem_Write(&hi2c1, DevAddress, reg_addr, 1, (uint8_t*)data, len, 15);
  return 0; // Success
}

void bme68x_delay_us(uint32_t period, void *ntf_ptr)
{
	HAL_Delay(period/1000);
}

static void bme680_interface_init(struct bme68x_dev* dev_ptr)
{
  uint8_t dev_addr;
	dev_addr = BME68X_I2C_ADDR_HIGH;
	dev_ptr -> intf = BME68X_I2C_INTF;
	dev_ptr -> read = user_i2c_read;
	dev_ptr -> write = user_i2c_write;
	dev_ptr -> delay_us = bme68x_delay_us;
	dev_ptr -> intf_ptr = &dev_addr;
	dev_ptr -> amb_temp = 25;
}

static int8_t bme680_config(struct bme68x_dev* bme680_ptr)
{
	sensor_conf.filter = iir_filter;
	sensor_conf.odr = odr_time;
	sensor_conf.os_hum = hum_os;
	sensor_conf.os_pres = pres_os;
	sensor_conf.os_temp = temp_os;

	int8_t rslt = bme68x_set_conf(&sensor_conf, bme680_ptr);
	return rslt;
}

static int8_t bme680_htr_config(struct bme68x_dev* bme680_ptr)
{
	htr_conf.enable = htr_enable;
	htr_conf.heatr_dur = htr_dur;
	htr_conf.heatr_temp = htr_temp;

	int8_t rslt = bme68x_set_heatr_conf(BME68X_FORCED_MODE, &htr_conf, bme680_ptr);
	return rslt;
}

void print_error(int8_t result)
{
	const char *return_msg;
	if(result == BME68X_OK)
	{
		return_msg = "Init Result: BME68X_OK\n";
		printmsg(return_msg);
	}
	else if(result == BME68X_E_NULL_PTR)
	{
		return_msg = "Init Result: BME68X_E_NULL_PTR\n";
		printmsg(return_msg);
	}
	else if(result == BME68X_E_COM_FAIL)
	{
		return_msg = "Init Result: BME68X_E_COM_FAIL\n";
		printmsg(return_msg);
	}
	else if(result == BME68X_E_DEV_NOT_FOUND)
	{
		return_msg = "Init Result: BME68X_E_DEV_NOT_FOUND\n";
		printmsg(return_msg);
	}
	else if(result == BME68X_E_INVALID_LENGTH)
	{
		return_msg = "Init Result: BME68X_E_INVALID_LENGTH\n";
		printmsg(return_msg);
	}
	else if(result == BME68X_E_SELF_TEST)
	{
		return_msg = "Init Result: BME68X_E_SELF_TEST\n";
		printmsg(return_msg);
	}
	else if(result == BME68X_W_DEFINE_OP_MODE)
	{
		return_msg = "Init Result: BME68X_W_DEFINE_OP_MODE\n";
		printmsg(return_msg);
	}
	else if(result == BME68X_W_NO_NEW_DATA)
	{
		return_msg = "Init Result: BME68X_W_NO_NEW_DATA\n";
		printmsg(return_msg);
	}
	else if(result == BME68X_W_DEFINE_SHD_HEATR_DUR)
	{
		return_msg = "Init Result: BME68X_W_DEFINE_SHD_HEATR_DUR\n";
		printmsg(return_msg);
	}
	else if(result == BME68X_I_PARAM_CORR)
	{
		return_msg = "Init Result: BME68X_I_PARAM_CORR\n";
		printmsg(return_msg);
	}
	else
	{
		return_msg = "Init Result: Not identified\n";
		printmsg(return_msg);
	}
}

static void sensor_handler(void* parameters)
{
  char msg[100];
  int len;
  int8_t result;
  uint32_t del_period;
  uint8_t num_fail_2_reset = 0;
	while(1)
	{
		result = bme68x_set_op_mode(BME68X_FORCED_MODE, &bme);
		del_period = bme68x_get_meas_dur(BME68X_FORCED_MODE, &sensor_conf, &bme) + (htr_conf.heatr_dur * 1000);
		bme.delay_us(del_period, bme.intf_ptr);
		// get data
		result = bme68x_get_data(BME68X_FORCED_MODE, &data, &n_fields, &bme);
		print_error(result);
		if(result == 0)
		{
			// print out data
			printmsg(rd_dt_suc);
			int len = snprintf(msg, 100, "Temperature: %.2f, Pressure: %.2f, Humidity: %.2f\n", data.temperature, data.pressure, data.humidity);
			//HAL_UART_Transmit(&huart2, (uint8_t *)msg, len, HAL_MAX_DELAY);
		}
		else
		{
			// print out error
			num_fail_2_reset++;
			printmsg(rd_dt_fail);
			len = snprintf(msg, 50, "result of data getting: %d\n", result);
			//HAL_UART_Transmit(&huart2, (uint8_t *)msg, len, HAL_MAX_DELAY);
			if(num_fail_2_reset >= max_number_of_data_fail)
			{
				num_fail_2_reset = 0;
				bme68x_soft_reset(&bme);
			}
			// put place holder into data
			data.temperature = 99.99;
			data.pressure = 99.99;
			data.humidity = 99.99;
		}
		xQueueSend(measurement_data_queue, &data, portMAX_DELAY);
		xTaskNotify(processing_handle, 0, eNoAction);
		HAL_Delay(500);
	}
}

static void processing_handler(void* parameters)
{
	char msg[100];
	int len;
	struct bme68x_data received_data;
	while(1)
	{
		// wait for data to be insert into the measurement queue
		xTaskNotifyWait(0, 0, NULL, portMAX_DELAY);
		xQueueReceive(measurement_data_queue, &received_data, 0);
		if((received_data.temperature == 99.99) && (received_data.pressure == 99.99) && (received_data.humidity == 99.99))
		{
			xQueueSend(display_information_queue, process_fail, portMAX_DELAY);
		}
		else
		{
			int len = snprintf(msg, 100, "Temperature: %.2f, Pressure: %.2f, Humidity: %.2f\n", data.temperature, data.pressure, data.humidity);
			xQueueSend(display_information_queue, &msg, portMAX_DELAY);
		}
	}
}
/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
