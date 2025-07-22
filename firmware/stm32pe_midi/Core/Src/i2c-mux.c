//
// STM32 driver for I2C multiplexers TCA9548A and PCA9548A
//
// Copyright (c) 2023 Jonathan Tainer. Subject to the BSD 2-Clause License.
//

#include "i2c-mux.h"

#define I2C_MUX_BASE_ADDR 112

// All times are defined using milliseconds
#define I2C_MUX_TIMEOUT 1		// Depends on bit rate. At 400kHz, 1ms should be fine
#define I2C_MUX_RESET_TIME_LOW 1	// Minimum 6ns reset pulse according to datasheet
#define I2C_MUX_RESET_TIME_HIGH 0	// Start condition can begin immediately after reset

int i2c_mux_reset(i2c_mux_t* mux) {
	if (mux->rst_port == NULL || mux->rst_pin == 0) return 1;

	// Pull reset pin low
	HAL_GPIO_WritePin(mux->rst_port, mux->rst_pin, GPIO_PIN_RESET);
	HAL_Delay(I2C_MUX_RESET_TIME_LOW);

	// Bring multiplexer out of reset
	HAL_GPIO_WritePin(mux->rst_port, mux->rst_pin, GPIO_PIN_SET);
	HAL_Delay(I2C_MUX_RESET_TIME_HIGH);

	// Ensure all channels are disabled by default
	return i2c_mux_select_multi(mux, 0);
}

int i2c_mux_select(i2c_mux_t* mux, int ch) {

	// If ch is in range 0-7 then one channel is enabled, else all are disabled
	uint8_t mask = 1 << ch;
	return i2c_mux_select_multi(mux, mask);
}

int i2c_mux_select_multi(i2c_mux_t* mux, uint8_t mask) {
	if (mux->hi2c == NULL) return 1;

	// Transmit bitmask to multiplexer
	uint8_t addr = (I2C_MUX_BASE_ADDR + mux->addr_offset) << 1;
	HAL_StatusTypeDef res;
	res = HAL_I2C_Master_Transmit(mux->hi2c, addr, &mask, 1, I2C_MUX_TIMEOUT);
	if (res != HAL_OK) return 1;

	// Read back bitmask from multiplexer to verify
	uint8_t mask_check = 0;
	res = HAL_I2C_Master_Receive(mux->hi2c, addr, &mask_check, 1, I2C_MUX_TIMEOUT);
	if (res != HAL_OK || mask_check != mask) return 1;
	return 0;
}
