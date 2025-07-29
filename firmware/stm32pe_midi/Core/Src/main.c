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

//#include "usbd_def.h"
//#include "usbd_cdc_if.h"

#include <stdbool.h>
#include <string.h>

#include "tlc5940.h"
#include "i2c-mux.h"
#include "TMAG5273.h"

#include "tusb.h"

#define DEBUG4_LED_GPIO_Port GPIOA
#define DEBUG4_LED_Pin GPIO_PIN_8

#define DEBUG2_LED_GPIO_Port GPIOA
#define DEBUG2_LED_Pin GPIO_PIN_9

#define DEBUG3_LED_GPIO_Port GPIOA
#define DEBUG3_LED_Pin GPIO_PIN_10

uint16_t light_key_arr[25] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0, 0 };

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

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */

extern SPI_HandleTypeDef hspi1;
//extern USBD_HandleTypeDef hUsbDeviceFS;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_SPI1_Init(void);
static void MX_I2C1_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

i2c_mux_t tca_mux = { .hi2c = &hi2c1,     // Using I2C1 on PB6 (SCL) / PB7 (SDA)
		.rst_port = GPIOB,          // Reset pin is PB7
		.rst_pin = GPIO_PIN_7,      // Use correct GPIO pin number
		.addr_offset = 0            // 0 if all A0/A1/A2 pins are GND
		};

uint16_t map_float_to_uint16(float x, float in_min, float in_max,
		uint16_t out_min, uint16_t out_max) {
	if (x < in_min)
		x = in_min;
	if (x > in_max)
		x = in_max;
	return (uint16_t) (((x - in_min) * (out_max - out_min)) / (in_max - in_min)
			+ out_min);
}

int _write(int file, char *ptr, int len) {
	(void) file;

	if (!tud_cdc_connected()) {
		return 0;
	}

	int remaining = len;
	uint32_t start = HAL_GetTick();
	while (remaining > 0) {
		uint32_t n = tud_cdc_write(ptr, remaining);
		tud_cdc_write_flush(); // send data to host
		ptr += n;
		remaining -= n;

		// Allow TinyUSB to process USB events
		tud_task();

		// Timeout after 100ms to avoid deadlock
		if ((HAL_GetTick() - start) > 100) {
			break;
		}
	}

	return len;
}

// Variable that holds the current position in the sequence.
uint32_t note_pos = 0;

// Store example melody as an array of note values
const uint8_t note_sequence[] = { 74, 78, 81, 86, 90, 93, 98, 102, 57, 61, 66,
		69, 73, 78, 81, 85, 88, 92, 97, 100, 97, 92, 88, 85, 81, 78, 74, 69, 66,
		62, 57, 62, 66, 69, 74, 78, 81, 86, 90, 93, 97, 102, 97, 93, 90, 85, 81,
		78, 73, 68, 64, 61, 56, 61, 64, 68, 74, 78, 81, 86, 90, 93, 98, 102 };

void midi_task(void) {
	static int note_pos = 0;
	static uint32_t last_tick = 0;
	uint8_t const cable_num = 0;
	uint8_t const channel = 0;

	// Only send MIDI if enough time has passed
	uint32_t now = HAL_GetTick();
	if (now - last_tick < 286)
		return;
	last_tick = now;

	// Clear any received MIDI data
	while (tud_midi_available()) {
		uint8_t packet[4];
		tud_midi_packet_read(packet);
	}

	// Send Note On for current note
	uint8_t note_on[3] = { 0x90 | channel, note_sequence[note_pos], 127 };
	tud_midi_stream_write(cable_num, note_on, 3);

	// Send Note Off for previous note
	int previous =
			(note_pos == 0) ? (sizeof(note_sequence) - 1) : (note_pos - 1);
	uint8_t note_off[3] = { 0x80 | channel, note_sequence[previous], 0 };
	tud_midi_stream_write(cable_num, note_off, 3);

	// Advance note position, wrap around
	note_pos++;
	if (note_pos >= sizeof(note_sequence)) {
		note_pos = 0;
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

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

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
  MX_TIM2_Init();
  MX_SPI1_Init();
  MX_I2C1_Init();
  MX_USB_OTG_FS_PCD_Init();
  /* USER CODE BEGIN 2 */

	// start the timer for the multiplexer ic for the leds
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);

	//set mp-reset on the i2c multiplexer to be high as it is active-low reset input
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);

	//disable all i2c channels for a clean slate bruh
	if (i2c_mux_reset(&tca_mux) != 0) {
		// Handle error, e.g., blink LED or halt
		HAL_GPIO_WritePin(DEBUG2_LED_GPIO_Port, DEBUG2_LED_Pin, GPIO_PIN_SET);
	}

	// init device stack for tiny usb!!!
	tusb_init(BOARD_DEVICE_RHPORT_NUM, NULL);

	// Example: select channel 6 on the mux to talk to sensors on SC6
	if (i2c_mux_select(&tca_mux, 6) != 0) {
		HAL_GPIO_WritePin(DEBUG2_LED_GPIO_Port, DEBUG2_LED_Pin, GPIO_PIN_SET);
	}

	uint8_t tmag_addr = 0x35 << 1;
	if (HAL_I2C_IsDeviceReady(&hi2c1, tmag_addr, 3, HAL_MAX_DELAY) != HAL_OK) {
		HAL_GPIO_WritePin(DEBUG2_LED_GPIO_Port, DEBUG2_LED_Pin, GPIO_PIN_SET);
	}

	TMAG5273_Handle_t tmag = { .pI2c = &hi2c1,
			.address = 0x35, // 0x41
			.magTempcoMode = TMAG5273_NO_MAG_TEMPCO, .convAvgMode =
					TMAG5273_CONV_AVG_32X, .readMode =
					TMAG5273_READ_MODE_STANDARD, .lplnMode = TMAG5273_LOW_NOISE,
			.operatingMode = TMAG5273_OPERATING_MODE_STANDBY,
			.magXYRange =
					TMAG5273_MAG_RANGE_40MT_133MT,
			.magZRange = TMAG5273_MAG_RANGE_40MT_133MT, .magXYRange =
					TMAG5273_MAG_RANGE_40MT_133MT, .magZRange =
					TMAG5273_MAG_RANGE_80MT_266MT, .tempChEn =
					TMAG5273_TEMP_CH_DISABLED, .angEn = TMAG5273_ANG_X_Z,
			.magChEn = TMAG5276_MAG_Z_X, .crcEna = TMAG5273_CRC_DISABLE,
			.sensor_id =
					0, .sleep = TMAG5276_10MS };

	bool tmag_initialized = false;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

		tud_task();

		//TLC5940_BreatheAllTest();
		//continue;

		//printf("Test 1\r\n");

		if (!tmag_initialized) { // && hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED
			TMAG5273_Init(&tmag);
			tmag_initialized = true;
		}

		//printf("Test 2\r\n");
		//printf("Test 2.5 - Device ID set as: 0x%02X\n", tmag.deviceId);

		//for (uint8_t addr = 0x30; addr <= 0x3F; addr++) {
		//	if (HAL_I2C_IsDeviceReady(&hi2c1, addr << 1, 1, 10) == HAL_OK) {
		//		printf("Found device at 0x%02X\n", addr);
		//	}
		//}

		//printf("Test 3\r\n");

		//uint8_t id;
		//if (TMAG5273_ReadRegister(&tmag, DEVICE_ID, 1, &id) == 0) {
		//	printf("Device ID: 0x%02X\n", id);
		//} else {
		//	printf("Failed to read Device ID\n");
		//}

		// reading register data to test i2c device
		//uint8_t raw_data[6];
		//if (TMAG5273_ReadRegister(&tmag, X_MSB_RESULT, 6, raw_data) == 0) {
		//	printf("Raw register data: ");
		//	for (int i = 0; i < 6; i++) {
		//		printf("0x%02X ", raw_data[i]);
		//	}
		//	printf("\r\n");
		//} else {
		//	printf("Failed to read raw register data\r\n");
		//}

		// reading sensor to output led/etc.
		TMAG5273_Axis_t mag;
		uint8_t ret = TMAG5273_ReadMagneticField(&tmag, &mag);
		if (ret != 0) {
			printf("ReadMagneticField failed with code %d\r\n", ret);
		} else {
			//printf("Bx = %.3f mT, By = %.3f mT, Bz = %.3f mT\r\n", mag.Bx, mag.By, mag.Bz);
			//printf("Bz = %.3f mT\r\n", mag.Bz);

			//printf("Return value: %u\r\n", ret);

			uint16_t bz_u16 = map_float_to_uint16(mag.Bz, 10.0f, 80.0f, 0,
					3000);

			TLC5940_SetMappedLED(9, bz_u16);
			TLC5940_Update();
		}

		// reading rotation from sensor
		TMAG5273_Angle_t angle;
		ret = TMAG5273_ReadAngle(&tmag, &angle);
		if (ret == 0) {
			//printf("Angle: %.2f deg, Magnitude: %.2f\r\n", angle.angle, angle.magnitude);
			//printf("Angle: %.2f deg\r\n", angle.angle);
		} else {
			printf("Failed to read angle, code: %d\r\n", ret);
		}

		printf("Bz = %.3f mT, Angle: %.2f deg\r\n", mag.Bz, angle.angle);
		//min 24, max 80

		midi_task();

		//HAL_Delay(100);
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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOMEDIUM;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
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
  hi2c1.Init.Timing = 0x00707CBB;
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
  hspi1.Init.Direction = SPI_DIRECTION_2LINES_TXONLY;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 0x0;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi1.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi1.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi1.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi1.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi1.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi1.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi1.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 31;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 16;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 9;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.battery_charging_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9|GPIO_PIN_8|GPIO_PIN_10|GPIO_PIN_2
                          |GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin : PB8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PA9 PA8 PA10 PA2
                           PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_8|GPIO_PIN_10|GPIO_PIN_2
                          |GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

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
