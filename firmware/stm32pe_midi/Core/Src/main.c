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

#include <stdlib.h>
#include <stdbool.h>
#include <string.h>

#include "tlc5940.h"
#include "i2c-mux.h"
#include "TMAG5273.h"

#include "tusb.h"

#include "helper.h"
#include "mpe.h"

#define DEBUG4_LED_GPIO_Port GPIOA
#define DEBUG4_LED_Pin GPIO_PIN_8

#define DEBUG2_LED_GPIO_Port GPIOA
#define DEBUG2_LED_Pin GPIO_PIN_9

#define DEBUG3_LED_GPIO_Port GPIOA
#define DEBUG3_LED_Pin GPIO_PIN_10

// ==========================================================================================
// the config is below (not really recommended to edit these below unless if yk what you are doing)
// ==========================================================================================

//midi
int start_octave = 4; //midi code shift
int mt_send_black = 40; //midi on threshold
int mt_send_white = 30; //midi on threshold

//mpe
int16_t pitch_mpe_st = 682; //mpe semitone pitch range (682 = 4st, 341 = 2st)
float angle_mpe_padding = 15; //mpe pitch
float angle_mpe_realistic_max = 25; //mpe pitch

float aftertouch_mpe_white_padding = 9; //mpe aftertouch
float aftertouch_mpe_black_padding = 15; //mpe aftertouch

//octave
uint32_t octave_change_mode_time_threshold = 200; //diff change between mode/octive
uint32_t octave_led_flash = 500; //ms between led flash during mode

//led
int startup_cutoff_wait = 100; //led cutoff + cutoff for initial sensor readings saving

//mode
uint32_t octave_mode_hold_down = 1500; //octave mode keys milliseconds to hold down & change
uint32_t mpe_mode_hold_down = 3000; //~ same above but for mpe mode keys

//debug
bool disable_comport = false; //silences comport to possibly make processing faster?

// ==========================================================================================
// the config is above
// ==========================================================================================

//all data below is in order of b, c, c#, d, d#, e, etc..
//uint16_t light_key_arr[25];

bool key_activated[25]; //midi on/off
uint8_t key_note_active[25]; //midi on/off safety

float key_raw_initial[25]; //midi velocity
float key_raw_angle_initial[25]; //mpe pitch

float key_tick_start[25]; //midi velocity

float key_low_cutoff[25]; //leds

float key_off_cutoff[25]; //midi off

uint8_t midi_key_arr[25]; //midi codes

TMAG5273_Handle_t tmag_handles[25]; //tmag obj handles

uint8_t tmag_addr_arr[25] = { 0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16,
		0x17, 0x18, 0x19, 0x1A, 0x1B, 0x1C, 0x1D, 0x1E, 0x1F, 0x20, 0x21, 0x22,
		0x23,
		0x24, 0x25, 0x26, 0x27, 0x28 }; //tmag addresses

bool is_black_key[25] = { false, false, true, false, true, false, false, true,
		false, true, false, true, false, false, true, false, true, false, false,
		true, false, true, false, true, false };

uint32_t octave_held_time = -1; //octave mode change
uint32_t octave_mode_time_elapsed = 0; //octave led
uint32_t octave_mode_time2_elapsed = -1; //mode change safety

bool octave_change_mode = false; //mode on/off
bool prevent_rapid_fire = false; //can change octave

bool mpe_midi_mode = true; //mpe mode change
uint32_t mpe_held_time = -1; //time saving

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

//TODO remove?

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
		.rst_pin = GPIO_PIN_8,      // Use correct GPIO pin number
		.addr_offset = 0            // 0 if all A0/A1/A2 pins are GND
		};

int _write(int file, char *ptr, int len) {
	(void) file;

	if (disable_comport) {
		return 0;
	}

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

	// start the timer for the led mux
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);

	// set mp-reset on the i2c mux to be high as it is active-low reset input
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);

	// disable all i2c channels for a clean slate bruh
	if (i2c_mux_reset(&tca_mux) != 0) {
		// Handle error, e.g., blink LED or halt
		HAL_GPIO_WritePin(DEBUG2_LED_GPIO_Port, DEBUG2_LED_Pin, GPIO_PIN_SET);
	}

	//on each channel, there are four i2c sensors connected with unique addresses in that channel only
	//the address has to be rewritten for all the sensors to then combine all channels pull data easier

	//sc0
	i2c_mux_select(&tca_mux, 0);

	tmag_handles[0] = TMAG5273_CreateHandle(&hi2c1, 0x35);
	TMAG5273_Init(&tmag_handles[0]);
	TMAG5273_RewriteI2CAddress(&tmag_handles[0], tmag_addr_arr[0]);

	tmag_handles[1] = TMAG5273_CreateHandle(&hi2c1, 0x22);
	TMAG5273_Init(&tmag_handles[1]);
	TMAG5273_RewriteI2CAddress(&tmag_handles[1], tmag_addr_arr[1]);

	tmag_handles[3] = TMAG5273_CreateHandle(&hi2c1, 0x78);
	TMAG5273_Init(&tmag_handles[3]);
	TMAG5273_RewriteI2CAddress(&tmag_handles[3], tmag_addr_arr[3]);

	tmag_handles[5] = TMAG5273_CreateHandle(&hi2c1, 0x44);
	TMAG5273_Init(&tmag_handles[5]);
	TMAG5273_RewriteI2CAddress(&tmag_handles[5], tmag_addr_arr[5]);

	
	//sc1
	i2c_mux_select(&tca_mux, 1);

	tmag_handles[6] = TMAG5273_CreateHandle(&hi2c1, 0x35);
	TMAG5273_Init(&tmag_handles[6]);
	TMAG5273_RewriteI2CAddress(&tmag_handles[6], tmag_addr_arr[6]);

	tmag_handles[8] = TMAG5273_CreateHandle(&hi2c1, 0x22);
	TMAG5273_Init(&tmag_handles[8]);
	TMAG5273_RewriteI2CAddress(&tmag_handles[8], tmag_addr_arr[8]);

	tmag_handles[10] = TMAG5273_CreateHandle(&hi2c1, 0x78);
	TMAG5273_Init(&tmag_handles[10]);
	TMAG5273_RewriteI2CAddress(&tmag_handles[10], tmag_addr_arr[10]);

	tmag_handles[12] = TMAG5273_CreateHandle(&hi2c1, 0x44);
	TMAG5273_Init(&tmag_handles[12]);
	TMAG5273_RewriteI2CAddress(&tmag_handles[12], tmag_addr_arr[12]);


	//sc2
	i2c_mux_select(&tca_mux, 2);

	tmag_handles[13] = TMAG5273_CreateHandle(&hi2c1, 0x35);
	TMAG5273_Init(&tmag_handles[13]);
	TMAG5273_RewriteI2CAddress(&tmag_handles[13], tmag_addr_arr[13]);

	tmag_handles[15] = TMAG5273_CreateHandle(&hi2c1, 0x22);
	TMAG5273_Init(&tmag_handles[15]);
	TMAG5273_RewriteI2CAddress(&tmag_handles[15], tmag_addr_arr[15]);

	tmag_handles[17] = TMAG5273_CreateHandle(&hi2c1, 0x78);
	TMAG5273_Init(&tmag_handles[17]);
	TMAG5273_RewriteI2CAddress(&tmag_handles[17], tmag_addr_arr[17]);

	tmag_handles[18] = TMAG5273_CreateHandle(&hi2c1, 0x44);
	TMAG5273_Init(&tmag_handles[18]);
	TMAG5273_RewriteI2CAddress(&tmag_handles[18], tmag_addr_arr[18]);


	//sc3
	i2c_mux_select(&tca_mux, 3);

	tmag_handles[20] = TMAG5273_CreateHandle(&hi2c1, 0x35);
	TMAG5273_Init(&tmag_handles[20]);
	TMAG5273_RewriteI2CAddress(&tmag_handles[20], tmag_addr_arr[20]);

	tmag_handles[22] = TMAG5273_CreateHandle(&hi2c1, 0x22);
	TMAG5273_Init(&tmag_handles[22]);
	TMAG5273_RewriteI2CAddress(&tmag_handles[22], tmag_addr_arr[22]);

	tmag_handles[24] = TMAG5273_CreateHandle(&hi2c1, 0x78);
	TMAG5273_Init(&tmag_handles[24]);
	TMAG5273_RewriteI2CAddress(&tmag_handles[24], tmag_addr_arr[24]);

	tmag_handles[2] = TMAG5273_CreateHandle(&hi2c1, 0x44);
	TMAG5273_Init(&tmag_handles[2]);
	TMAG5273_RewriteI2CAddress(&tmag_handles[2], tmag_addr_arr[2]);


	//sc4
	i2c_mux_select(&tca_mux, 4);

	tmag_handles[4] = TMAG5273_CreateHandle(&hi2c1, 0x35);
	TMAG5273_Init(&tmag_handles[4]);
	TMAG5273_RewriteI2CAddress(&tmag_handles[4], tmag_addr_arr[4]);

	tmag_handles[7] = TMAG5273_CreateHandle(&hi2c1, 0x22);
	TMAG5273_Init(&tmag_handles[7]);
	TMAG5273_RewriteI2CAddress(&tmag_handles[7], tmag_addr_arr[7]);

	tmag_handles[9] = TMAG5273_CreateHandle(&hi2c1, 0x78);
	TMAG5273_Init(&tmag_handles[9]);
	TMAG5273_RewriteI2CAddress(&tmag_handles[9], tmag_addr_arr[9]);

	tmag_handles[11] = TMAG5273_CreateHandle(&hi2c1, 0x44);
	TMAG5273_Init(&tmag_handles[11]);
	TMAG5273_RewriteI2CAddress(&tmag_handles[11], tmag_addr_arr[11]);


	//sc5
	i2c_mux_select(&tca_mux, 5);

	tmag_handles[14] = TMAG5273_CreateHandle(&hi2c1, 0x35);
	TMAG5273_Init(&tmag_handles[14]);
	TMAG5273_RewriteI2CAddress(&tmag_handles[14], tmag_addr_arr[14]);

	tmag_handles[16] = TMAG5273_CreateHandle(&hi2c1, 0x22);
	TMAG5273_Init(&tmag_handles[16]);
	TMAG5273_RewriteI2CAddress(&tmag_handles[16], tmag_addr_arr[16]);

	tmag_handles[19] = TMAG5273_CreateHandle(&hi2c1, 0x78);
	TMAG5273_Init(&tmag_handles[19]);
	TMAG5273_RewriteI2CAddress(&tmag_handles[19], tmag_addr_arr[19]);

	tmag_handles[21] = TMAG5273_CreateHandle(&hi2c1, 0x44);
	TMAG5273_Init(&tmag_handles[21]);
	TMAG5273_RewriteI2CAddress(&tmag_handles[21], tmag_addr_arr[21]);


	//sc6
	i2c_mux_select(&tca_mux, 6);

	tmag_handles[23] = TMAG5273_CreateHandle(&hi2c1, 0x35);
	TMAG5273_Init(&tmag_handles[23]);
	TMAG5273_RewriteI2CAddress(&tmag_handles[23], tmag_addr_arr[23]);



	// enable sc0–sc6, disable sc7
	i2c_mux_select_multi(&tca_mux, 0x7F);




	// fill midi array
	fill_midi_key_arr(midi_key_arr, 25, start_octave);

	// init high/low cutoffs & other stuff
	for (int i = 0; i < 25; i++) {
		key_low_cutoff[i] = -1.0f;
		key_tick_start[i] = 0;
		key_raw_angle_initial[i] = -1;
	}

	// for if the user changed the default midi mpe mode
	if (mpe_midi_mode) {
		HAL_GPIO_WritePin(DEBUG3_LED_GPIO_Port, DEBUG3_LED_Pin, GPIO_PIN_RESET);
	} else {
		HAL_GPIO_WritePin(DEBUG3_LED_GPIO_Port, DEBUG3_LED_Pin, GPIO_PIN_SET);
	}

	// init device stack for tiny usb!!!
	tusb_init(BOARD_DEVICE_RHPORT_NUM, NULL);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

		tud_task();

		//dont allow cutoffs to be set until a solid line of sensor values are being read good
		static uint32_t startup_tick = 0;
		if (startup_tick == 0) {
			startup_tick = HAL_GetTick();
		}
		uint8_t allow_cutoff_set = (HAL_GetTick() - startup_tick
				> startup_cutoff_wait);
		// ^ this time is how long the user can not touch the keys after being plugged in

		//for each 25 keys
		for (int d = 0; d < 25; d++) {

			// read address register to see if address is actually rewritten
			//TODO remove in next code cleanup
			uint8_t addr_reg = 0;
			if (TMAG5273_ReadRegister(&tmag_handles[d], 0x0C, 1, &addr_reg)
					== 0) {
				//printf("I2C address register: 0x%02X\r\n", addr_reg);
			} else {
				printf(
						"Sensor Index = %d, Failed to read i2c register data\r\n",
						d);
			};

			// reading rotation from sensor
			TMAG5273_Axis_t mag;
			TMAG5273_Angle_t angle;
			uint8_t ret = -1;

			// reading sensor magnet pos. to output led/etc.
			ret = TMAG5273_ReadMagneticField(&tmag_handles[d], &mag);
			if (ret == 0) {
				//printf("Bx = %.3f mT, By = %.3f mT, Bz = %.3f mT\r\n", mag.Bx, mag.By, mag.Bz);
				//printf("Return value: %u\r\n", ret);

				uint16_t bz_u16 = 0;

				//use the key_low_cutoff nan to set an initial cutoff for later calculations
				if (key_low_cutoff[d] < 0 && allow_cutoff_set) {
					if (mag.Bz > 9) {
						key_low_cutoff[d] = mag.Bz - 9;
					} else {
						key_low_cutoff[d] = 0;
					}

					key_off_cutoff[d] = mag.Bz + 5; //add a padding of 5

					key_raw_initial[d] = mag.Bz;

					//key_raw_angle_initial[d] = angle.angle -- moved to key on for reasons explained
				} else {
					//normalize mag.Bz around ~9
					bz_u16 = map_float_to_int16(mag.Bz - key_low_cutoff[d],
							10.0f, 80.0f, 0, 3000);
				}

				TLC5940_SetMappedByKeyLED(d, bz_u16);
				TLC5940_Update();
			} else {
				printf(
						"I2C Address = 0x%02X, Sensor Index = %d, ReadMagneticField failed code = %d\r\n",
						addr_reg, d, ret);
			}

			ret = TMAG5273_ReadAngle(&tmag_handles[d], &angle);
			if (ret == 0) {
				//printf("Angle: %.2f deg, Magnitude: %.2f\r\n", angle.angle, angle.magnitude);
			} else {
				printf("Sensor Index = %d, Failed angle read, code: %d\r\n", d,
						ret);
			}

			//mpe midi mode change
			if (key_activated[1] && key_activated[22]) {
				if (mpe_held_time == -1) {
					//set the first time
					mpe_held_time = HAL_GetTick();
				} else if ((HAL_GetTick() - mpe_held_time) > mpe_mode_hold_down
						&& mpe_held_time != -1) {
					//count down the time until its more than 3 seconds held
					mpe_midi_mode = !mpe_midi_mode;

					if (mpe_midi_mode) {
						HAL_GPIO_WritePin(DEBUG3_LED_GPIO_Port, DEBUG3_LED_Pin,
								GPIO_PIN_RESET);
					} else {
						HAL_GPIO_WritePin(DEBUG3_LED_GPIO_Port, DEBUG3_LED_Pin,
								GPIO_PIN_SET);
					}

					//prevent rapid fire of changing modes
					mpe_held_time = -1;
				}
			} else if (!key_activated[1] && !key_activated[22]) {
				//reset time if both keys are unactivated
				mpe_held_time = -1;
			}

			//octave change
			if (key_activated[0] && key_activated[24]) {
				if (octave_held_time == -1) {
					//set the first time
					octave_held_time = HAL_GetTick();
				} else if ((HAL_GetTick() - octave_held_time)
						> octave_mode_hold_down
						&& octave_held_time != -1) {
					//count down the time until its more than 3 seconds held
					octave_change_mode = !octave_change_mode;

					if (octave_change_mode) {
						octave_mode_time_elapsed = HAL_GetTick();
					} else {
						HAL_GPIO_WritePin(DEBUG4_LED_GPIO_Port, DEBUG4_LED_Pin,
								GPIO_PIN_RESET);
					}

					//prevent rapid fire of changing modes
					octave_held_time = -1;
				}
			} else if (!key_activated[0] && !key_activated[24]) {
				//reset time if both keys are unactivated
				octave_held_time = -1;
			}

			//turn status led on to indicate octave change mode
			if (octave_change_mode
					&& (HAL_GetTick() - octave_mode_time_elapsed)
							> octave_led_flash) {
				HAL_GPIO_TogglePin(DEBUG4_LED_GPIO_Port, DEBUG4_LED_Pin);
				octave_mode_time_elapsed = HAL_GetTick();
			}

			//octave down
			if (d == 0 && mag.Bz > mt_send_white && start_octave >= 0
					&& !prevent_rapid_fire && octave_change_mode) {
				if (octave_mode_time2_elapsed == -1) {
					octave_mode_time2_elapsed = HAL_GetTick();
				} else if (key_activated[0] && key_activated[24]) {
					//catch if user is trying to exit octave mode instead of changing it
					octave_mode_time2_elapsed = -1;

				} else if ((HAL_GetTick() - octave_mode_time2_elapsed)
						> octave_change_mode_time_threshold) {
					start_octave--;
					fill_midi_key_arr(midi_key_arr, 25, start_octave);
					prevent_rapid_fire = true;
					octave_mode_time2_elapsed = -1;
				}
			}

			//octave up
			if (d == 24 && mag.Bz > mt_send_white && start_octave <= 9
					&& !prevent_rapid_fire && octave_change_mode) {
				if (octave_mode_time2_elapsed == -1) {
					octave_mode_time2_elapsed = HAL_GetTick();
				} else if (key_activated[0] && key_activated[24]) {
					//catch if user is trying to exit octave mode instead of changing it
					octave_mode_time2_elapsed = -1;

				} else if ((HAL_GetTick() - octave_mode_time2_elapsed)
						> octave_change_mode_time_threshold) {
					start_octave++;
					fill_midi_key_arr(midi_key_arr, 25, start_octave);
					prevent_rapid_fire = true;
					octave_mode_time2_elapsed = -1;
				}
			}

			//TODO remove next code cleanup
			//printf("Bz = %.3f mT, Angle: %.2f deg, Change: %.2f, Max: %.2f\r\n", mag.Bz, angle.angle, current_change, key_dist_max[d]);

			//start time for measuring velocity
			if (mag.Bz > key_raw_initial[d] + 1.5 && key_tick_start[d] == 0) {
				key_tick_start[d] = HAL_GetTick();
			} else if (mag.Bz < key_raw_initial[d] + 0.8
					&& key_tick_start[d] != 0) {
				//safety measure to stop measuring tick start as user has started measurement but did not trigger a key press
				key_tick_start[d] = 0;
			}

			//use a specific mt send as the black keys have a shorter distance compared to white keys
			int mt_send = is_black_key[d] ? mt_send_black : mt_send_white;

			//turn on key
			if (mag.Bz > mt_send && !key_activated[d]) {
				uint8_t measured_velocity = map_velocity_log(
						HAL_GetTick() - key_tick_start[d]);
				
				if (mpe_midi_mode) {
					MPE_Send_Note_On(midi_key_arr[d], measured_velocity);
				} else {
					uint8_t note_on[3] = { 0x90 | 0, midi_key_arr[d],
							measured_velocity };

					tud_midi_stream_write(0, note_on, 3);
				}

				printf(
						"%d, midi = on, code = %d, velocity = %d\r\n", d,
						midi_key_arr[d], measured_velocity);
				
				key_activated[d] = true;
				key_note_active[d] = midi_key_arr[d];

				//also set the raw initial angle here...
				//some downsides of this is that the user has to press the key perfectly/mostly flat at startup or else the pitch will be offset
				//though, this does solve the issue of the sensor changing the angle when the key is normally actuated for some reason
				if (key_raw_angle_initial[d] == -1) {
					key_raw_angle_initial[d] = angle.angle;
					printf("%d, raw angle init = %0.2f\r\n", d,
							key_raw_angle_initial[d]);
				}
			}

			//turn off key
			if (mag.Bz < key_off_cutoff[d] && key_activated[d]) {
				if (mpe_midi_mode) {
					MPE_Send_Note_Off(key_note_active[d]);
				} else {
					uint8_t note_off[3] = { 0x80 | 0, key_note_active[d], 0 };

					tud_midi_stream_write(0, note_off, 3);
				}

				printf("%d, midi = off, code = %d\r\n", d, key_note_active[d]);

				key_activated[d] = false;
				key_note_active[d] = 0;

				//reset velocity measurement
				key_tick_start[d] = 0;

				prevent_rapid_fire = false;
			}

			//pitch change per key
			if (key_activated[d] && mpe_midi_mode) {
				// black key bias - 40max, 44, 91-94-100mid, 134, 150max
				// white key bias - 20max, 30, 82-84-86mid, 130, 140max
				// +-20 from origin angle, +-5/3 as rest from origin angle
				int16_t shift = 0;

				if (angle.angle
						> (key_raw_angle_initial[d] + angle_mpe_padding)) { //up
					//min - the min includes the key's resting angle and the padding before pitch change gets triggered
					//max - includes the above + the realistic max angle/roll for each key
					shift = map_float_to_int16(angle.angle,
							(key_raw_angle_initial[d] + angle_mpe_padding),
							(key_raw_angle_initial[d] + angle_mpe_padding
									+ angle_mpe_realistic_max), 0,
							pitch_mpe_st);
				} else if (angle.angle
						< (key_raw_angle_initial[d] - angle_mpe_padding)) { //down
					//same min/max for pitching up, but negative
					shift = map_float_to_int16(angle.angle,
							(key_raw_angle_initial[d] - angle_mpe_padding
									- angle_mpe_realistic_max),
							(key_raw_angle_initial[d] - angle_mpe_padding),
							-pitch_mpe_st, 0);
				}

				MPE_Send_Pitch_Bend(midi_key_arr[d], shift);
			}

			int aftertouch_mpe_padding =
					is_black_key[d] ?
							aftertouch_mpe_black_padding :
							aftertouch_mpe_white_padding;
			
			//aftertouch per key
			if (mag.Bz > (mt_send + aftertouch_mpe_padding)
					&& key_activated[d]
					&& mpe_midi_mode) {
				uint8_t pressure = 0;

				pressure = map_float_to_uint8(mag.Bz,
						(mt_send + aftertouch_mpe_padding), 78, 0, 127);

				MPE_Send_Aftertouch(midi_key_arr[d], pressure);
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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

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
  RCC_OscInitStruct.PLL.PLLN = 50;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
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
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
  hi2c1.Init.Timing = 0x10C0ECFF;
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
