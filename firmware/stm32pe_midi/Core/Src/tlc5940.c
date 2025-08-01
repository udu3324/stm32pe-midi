#include "tlc5940.h"

#define GSCLK_GPIO_Port GPIOA
#define GSCLK_Pin       GPIO_PIN_1

#define BLANK_GPIO_Port GPIOA
#define BLANK_Pin       GPIO_PIN_3

#define XLAT_GPIO_Port  GPIOA
#define XLAT_Pin        GPIO_PIN_2

//#define DEBUG_LED_GPIO_Port GPIOA
//#define DEBUG_LED_Pin GPIO_PIN_8

extern SPI_HandleTypeDef hspi1;

static uint16_t led_pwm[TLC5940_CHANNELS] = { 0 };

static void TLC5940_SendPWMData(void);
static void TLC5940_PulsePin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);

void TLC5940_Init(void) {
	HAL_GPIO_WritePin(GSCLK_GPIO_Port, GSCLK_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(BLANK_GPIO_Port, BLANK_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(XLAT_GPIO_Port, XLAT_Pin, GPIO_PIN_RESET);

	for (int i = 0; i < TLC5940_CHANNELS; i++)
		led_pwm[i] = 0;

	TLC5940_Update();
}

void TLC5940_SetLED(uint8_t led_index, uint16_t brightness) {
	if (led_index >= TLC5940_USED_LEDS)
		return;

	if (brightness > TLC5940_MAX_PWM)
		brightness = TLC5940_MAX_PWM;

	led_pwm[led_index] = brightness;
}

void TLC5940_SetMappedPhysicallyLED(uint8_t logical_index, uint16_t brightness) {
	static const uint8_t mapping_table[10] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9 };

	uint8_t physical_index;

	if (logical_index < 10) {
		physical_index = mapping_table[logical_index];
	} else if (logical_index >= 10 && logical_index <= 25) { // 10–25 to physical 16–31
		physical_index = logical_index + 6;
	} else {
		return;
	}

	TLC5940_SetLED(physical_index, brightness);
}

//# index is the physical layout, with its index pointing to the actual key layout
static const uint8_t key_to_logical_map[25] = { 10, 11, 0, 12, 1, 13, 14, 2,
			15, 3, 16, 4, 17, 18, 5, 19, 6, 20, 21, 7, 22, 8, 23, 9, 24 };

void TLC5940_SetMappedByKeyLED(uint8_t key_index, uint16_t brightness) {
	if (key_index < 25) {
		TLC5940_SetMappedPhysicallyLED(key_to_logical_map[key_index],
				brightness);
	}
}

static void TLC5940_SendPWMData(void) {
	uint8_t data[48] = { 0 }; // 32 channels × 12 bits = 384 bits = 48 bytes
	uint32_t bit_cursor = 0;

	for (int ch = TLC5940_CHANNELS - 1; ch >= 0; ch--) // 👈 FIXED: use all 32 channels
			{
		uint16_t val = led_pwm[ch] & 0x0FFF;

		for (int b = 11; b >= 0; b--) {
			if ((val >> b) & 1)
				data[bit_cursor / 8] |= (1 << (7 - (bit_cursor % 8)));

			bit_cursor++;
		}
	}

	HAL_GPIO_WritePin(BLANK_GPIO_Port, BLANK_Pin, GPIO_PIN_SET);
	HAL_SPI_Transmit(&hspi1, data, sizeof(data), HAL_MAX_DELAY);
	TLC5940_PulsePin(XLAT_GPIO_Port, XLAT_Pin);
	HAL_GPIO_WritePin(BLANK_GPIO_Port, BLANK_Pin, GPIO_PIN_RESET);
}

static void TLC5940_PulsePin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin) {
	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_SET);
	for (volatile int i = 0; i < 20; i++)
		__NOP();
	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET);
}

void TLC5940_Update(void) {
	// DEBUG: Turn on LED before sending
	//HAL_GPIO_WritePin(DEBUG_LED_GPIO_Port, DEBUG_LED_Pin, GPIO_PIN_SET);

	// Send the full PWM data for all TLC5940 chips (should send 48 bytes for 2 chips)
	TLC5940_SendPWMData();

	// DEBUG: Turn off LED after sending
	//HAL_GPIO_WritePin(DEBUG_LED_GPIO_Port, DEBUG_LED_Pin, GPIO_PIN_RESET);

	// Pulse XLAT to latch the data
	HAL_GPIO_WritePin(XLAT_GPIO_Port, XLAT_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(XLAT_GPIO_Port, XLAT_Pin, GPIO_PIN_RESET);
}

void TLC5940_TestPattern(void) {
	for (uint8_t i = 0; i < TLC5940_USED_LEDS; i++) {
		for (uint8_t j = 0; j < TLC5940_CHANNELS; j++)
			TLC5940_SetLED(j, (j == i) ? 1000 : 0);

		TLC5940_Update();
		HAL_Delay(200);
	}
}

void TLC5940_AllOnTest(void) {
	for (int i = 0; i < TLC5940_CHANNELS; i++)
		TLC5940_SetLED(i, 1000);

	TLC5940_Update();
}

void TLC5940_BreatheTest(void) {
	for (uint8_t i = 0; i < TLC5940_NUM_LEDS; i++) {
		// Turn all LEDs off first
		for (uint8_t j = 0; j < TLC5940_NUM_LEDS; j++) {
			TLC5940_SetMappedLED(j, 0);
		}

		for (uint16_t b = 0; b < 1000; b++) {
			TLC5940_SetMappedLED(i, b);
			TLC5940_Update();
			HAL_Delay(0.4);
		}
		for (int16_t b = 999; b > 0; b--) {
			TLC5940_SetMappedLED(i, b);
			TLC5940_Update();
			HAL_Delay(0.4);
		}

		HAL_Delay(5);
	}

	TLC5940_Update();
}

void TLC5940_BreatheAllTest(void) {
	for (uint16_t b = 0; b < 1000; b++) {
		for (uint8_t i = 0; i < TLC5940_NUM_LEDS; i++) {
			TLC5940_SetMappedLED(i, b);
		}

		TLC5940_Update();
		HAL_Delay(1);
	}

	for (int16_t b = 999; b > 0; b--) {
		for (uint8_t i = 0; i < TLC5940_NUM_LEDS; i++) {
			TLC5940_SetMappedLED(i, b);
		}

		TLC5940_Update();
		HAL_Delay(1);
	}

	HAL_Delay(5);

	TLC5940_Update();
}
