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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
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

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = { .name = "defaultTask",
		.stack_size = 128 * 4, .priority = (osPriority_t) osPriorityNormal, };
/* Definitions for gpsTask */
osThreadId_t gpsTaskHandle;
const osThreadAttr_t gpsTask_attributes = { .name = "gpsTask", .stack_size =
		1024 * 4, .priority = (osPriority_t) osPriorityNormal, };
/* Definitions for imuTask */
osThreadId_t imuTaskHandle;
const osThreadAttr_t imuTask_attributes = { .name = "imuTask", .stack_size = 256
		* 4, .priority = (osPriority_t) osPriorityNormal, };
/* USER CODE BEGIN PV */

// GPS
#define GPS_BUFFER_SIZE 512
#define GPS_LINE_SIZE 128

uint8_t gps_dma_buffer[GPS_BUFFER_SIZE];
uint16_t received_pack_size = 0;
uint8_t line[GPS_LINE_SIZE];
uint16_t old_pos = 0;

typedef struct {
	double latitude;
	double longitude;
	float speed_kmh;
	uint32_t timestamp;
	uint8_t satellite_count;
	uint8_t is_valid;
} GPS_Data_t;

GPS_Data_t gps_data;

// IMU
#define MPU6050_ADDR (0x68 << 1)

typedef struct {
	float acc_x, acc_y, acc_z;
	float gyro_x, gyro_y, gyro_z;
	float temperature;

	uint8_t is_connected;
} IMU_Data_t;

IMU_Data_t imu_data;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);
void StartDefaultTask(void *argument);
void StartGPSTask(void *argument);
void StartIMUTask(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

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
	MX_USART1_UART_Init();
	MX_I2C1_Init();
	/* USER CODE BEGIN 2 */

	/* USER CODE END 2 */

	/* Init scheduler */
	osKernelInitialize();

	/* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
	/* USER CODE END RTOS_MUTEX */

	/* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
	/* USER CODE END RTOS_SEMAPHORES */

	/* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
	/* USER CODE END RTOS_TIMERS */

	/* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
	/* USER CODE END RTOS_QUEUES */

	/* Create the thread(s) */
	/* creation of defaultTask */
	defaultTaskHandle = osThreadNew(StartDefaultTask, NULL,
			&defaultTask_attributes);

	/* creation of gpsTask */
	gpsTaskHandle = osThreadNew(StartGPSTask, NULL, &gpsTask_attributes);

	/* creation of imuTask */
	imuTaskHandle = osThreadNew(StartIMUTask, NULL, &imuTask_attributes);

	/* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
	/* USER CODE END RTOS_THREADS */

	/* USER CODE BEGIN RTOS_EVENTS */
	/* add events, ... */
	/* USER CODE END RTOS_EVENTS */

	/* Start scheduler */
	osKernelStart();

	/* We should never get here as control is now taken by the scheduler */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void) {

	/* USER CODE BEGIN I2C1_Init 0 */

	/* USER CODE END I2C1_Init 0 */

	/* USER CODE BEGIN I2C1_Init 1 */

	/* USER CODE END I2C1_Init 1 */
	hi2c1.Instance = I2C1;
	hi2c1.Init.ClockSpeed = 400000;
	hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN I2C1_Init 2 */

	/* USER CODE END I2C1_Init 2 */

}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void) {

	/* USER CODE BEGIN USART1_Init 0 */

	/* USER CODE END USART1_Init 0 */

	/* USER CODE BEGIN USART1_Init 1 */

	/* USER CODE END USART1_Init 1 */
	huart1.Instance = USART1;
	huart1.Init.BaudRate = 9600;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART1_Init 2 */

	/* USER CODE END USART1_Init 2 */

}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) {

	/* DMA controller clock enable */
	__HAL_RCC_DMA2_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA2_Stream2_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */

	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

	/*Configure GPIO pin : PA5 */
	GPIO_InitStruct.Pin = GPIO_PIN_5;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : PA6 */
	GPIO_InitStruct.Pin = GPIO_PIN_6;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI9_5_IRQn, 6, 0);
	HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

	/* USER CODE BEGIN MX_GPIO_Init_2 */

	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
	if (huart->Instance == USART1) {
		if (__HAL_UART_GET_FLAG(huart, UART_FLAG_ORE)) {
			__HAL_UART_CLEAR_OREFLAG(huart);
			HAL_UART_DMAStop(huart);
			old_pos = 0;
			HAL_UARTEx_ReceiveToIdle_DMA(huart, gps_dma_buffer,
			GPS_BUFFER_SIZE);
			return;
		}

		received_pack_size = Size;
		osThreadFlagsSet(gpsTaskHandle, 0x0001);
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == GPIO_PIN_6) {
		osThreadFlagsSet(imuTaskHandle, 0x0002);
	}
}

// GPS
void Format_NMEA_Sentence(char *dest, char *src) {
	int i = 0;
	int j = 0;

	while (src[i] != '\0') {
		dest[j++] = src[i];

		if (src[i] == ',' && src[i + 1] == ',') {
			dest[j++] = '0';
		}
		i++;
	}
	dest[j] = '\0';
}

void Parse_GPS_RMC(char *nmea_sentence) {
	char formatted[128];
	Format_NMEA_Sentence(formatted, nmea_sentence);

	char msg_id[10] = { 0 };
	char utc_time[20] = { 0 };
	char status = 'V';
	double raw_lat = 0.0, raw_lon = 0.0;
	char lat_dir = 'N', lon_dir = 'E';
	float speed_knots = 0.0;

	int parsed = sscanf(formatted, "$%[^,],%[^,],%c,%lf,%c,%lf,%c,%f", msg_id,
			utc_time, &status, &raw_lat, &lat_dir, &raw_lon, &lon_dir,
			&speed_knots);

	if (parsed >= 8 && status == 'A') {
		int lat_degrees = (int) (raw_lat / 100);
		double lat_minutes = raw_lat - (lat_degrees * 100);
		gps_data.latitude = lat_degrees + (lat_minutes / 60.0);
		if (lat_dir == 'S')
			gps_data.latitude = -gps_data.latitude;

		int lon_degrees = (int) (raw_lon / 100);
		double lon_minutes = raw_lon - (lon_degrees * 100);
		gps_data.longitude = lon_degrees + (lon_minutes / 60.0);
		if (lon_dir == 'W')
			gps_data.longitude = -gps_data.longitude;

		gps_data.speed_kmh = speed_knots * 1.852f;
		gps_data.timestamp = osKernelGetTickCount();
		gps_data.is_valid = 1;
	} else {
		gps_data.is_valid = 0;
	}
}

void Parse_GPS_GGA(char *nmea_sentence) {
	char formatted[128];
	Format_NMEA_Sentence(formatted, nmea_sentence);

	char msg_id[10] = { 0 };
	char utc_time[20] = { 0 };
	double dummy_lat = 0.0, dummy_lon = 0.0;
	char dummy_char1 = 'N', dummy_char2 = 'E';
	int fix_quality = 0;
	int satellites = 0;

	int parsed = sscanf(formatted, "$%[^,],%[^,],%lf,%c,%lf,%c,%d,%d", msg_id,
			utc_time, &dummy_lat, &dummy_char1, &dummy_lon, &dummy_char2,
			&fix_quality, &satellites);

	if (parsed >= 8) {
		gps_data.satellite_count = (uint8_t) satellites;
	}
}

// IMU
void MPU6050_Init(void) {
	uint8_t check_val = 0;
	uint8_t wake_data = 0x00;
	uint8_t int_en_data = 0x01;
	uint8_t rate_div = 0x09;

	HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, 0x75, 1, &check_val, 1, 100);

	if (check_val == 0x68) {
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, 0x6B, 1, &wake_data, 1, 100);

		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, 0x19, 1, &rate_div, 1, 100);

		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, 0x38, 1, &int_en_data, 1, 100);

		imu_data.is_connected = 1;
	} else {
		imu_data.is_connected = 0;
	}
}

void MPU6050_Read_All(void) {
	uint8_t raw_buffer[14];

	if (!imu_data.is_connected)
		return;

	HAL_StatusTypeDef mem_read_result = HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR,
			0x3B, 1, raw_buffer, 14, 100);

	if (mem_read_result == HAL_OK) {

		int16_t raw_acc_x = (int16_t) ((raw_buffer[0] << 8) | raw_buffer[1]);
		int16_t raw_acc_y = (int16_t) ((raw_buffer[2] << 8) | raw_buffer[3]);
		int16_t raw_acc_z = (int16_t) ((raw_buffer[4] << 8) | raw_buffer[5]);
		int16_t raw_temp = (int16_t) ((raw_buffer[6] << 8) | raw_buffer[7]);
		int16_t raw_gyro_x = (int16_t) ((raw_buffer[8] << 8) | raw_buffer[9]);
		int16_t raw_gyro_y = (int16_t) ((raw_buffer[10] << 8) | raw_buffer[11]);
		int16_t raw_gyro_z = (int16_t) ((raw_buffer[12] << 8) | raw_buffer[13]);

		imu_data.acc_x = (float) raw_acc_x / 16384.0f;
		imu_data.acc_y = (float) raw_acc_y / 16384.0f;
		imu_data.acc_z = (float) raw_acc_z / 16384.0f;

		imu_data.gyro_x = (float) raw_gyro_x / 131.0f;
		imu_data.gyro_y = (float) raw_gyro_y / 131.0f;
		imu_data.gyro_z = (float) raw_gyro_z / 131.0f;

		imu_data.temperature = ((float) raw_temp / 340.0f) + 36.53f;
	}
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument) {
	/* USER CODE BEGIN 5 */
	/* Infinite loop */
	for (;;) {
		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
		osDelay(500);
	}
	/* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartGPSTask */
/**
 * @brief Function implementing the gpsTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartGPSTask */
void StartGPSTask(void *argument) {
	/* USER CODE BEGIN StartGPSTask */
	/* Infinite loop */
	static uint8_t line_index = 0;
	uint16_t local_pack_size = 0;

	HAL_UARTEx_ReceiveToIdle_DMA(&huart1, gps_dma_buffer, GPS_BUFFER_SIZE);

	while (1) {
		osThreadFlagsWait(0x0001, osFlagsWaitAny, osWaitForever);

		local_pack_size = received_pack_size;

		while (old_pos != local_pack_size) {
			uint8_t data = gps_dma_buffer[old_pos];

			if (data == '\n') {
				line[line_index] = '\0';

				if (line[0] == '$' && line_index > 6) {

					if (line[3] == 'R' && line[4] == 'M' && line[5] == 'C') {
						Parse_GPS_RMC((char*) line);
					} else if (line[3] == 'G' && line[4] == 'G'
							&& line[5] == 'A') {
						Parse_GPS_GGA((char*) line);
					}

				}
				line_index = 0;
			} else if (data != '\r' && data >= 32 && data <= 126) {
				if (line_index < 127) {
					line[line_index++] = data;
				}
			}

			old_pos++;
			if (old_pos >= GPS_BUFFER_SIZE) {
				old_pos = 0;
			}
		}
	}

	/* USER CODE END StartGPSTask */
}

/* USER CODE BEGIN Header_StartIMUTask */
/**
 * @brief Function implementing the imuTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartIMUTask */
void StartIMUTask(void *argument) {
	/* USER CODE BEGIN StartIMUTask */
	HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);

	osDelay(100);

	MPU6050_Init();

	uint8_t dummy_status = 0;

	if (imu_data.is_connected) {

		HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, 0x3A, 1, &dummy_status, 1, 100);

		HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
	}

	/* Infinite loop */
	while (1) {
		osThreadFlagsWait(0x0002, osFlagsWaitAny, osWaitForever);

		MPU6050_Read_All();

		HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, 0x3A, 1, &dummy_status, 1, 5);
	}
	/* USER CODE END StartIMUTask */
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
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
