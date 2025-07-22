//
// STM32 driver for I2C multiplexers TCA9548A and PCA9548A
//
// Copyright (c) 2023 Jonathan Tainer. Subject to the BSD 2-Clause License.
//

#ifndef I2C_MUX_H
#define I2C_MUX_H

#include "main.h" // Can be replaced by stm32xxxx_hal.h

typedef struct i2c_mux {
	I2C_HandleTypeDef* hi2c;	// I2C bus controller
	GPIO_TypeDef* rst_port;		// Reset pin GPIO port (set NULL if unused)
	uint16_t rst_pin;		// Reset pin bitmask
	uint8_t addr_offset;		// Offset from base address (set using address pins)
} i2c_mux_t;

// Returns 0 on success, 1 on error.
int i2c_mux_reset(i2c_mux_t* mux);

// Assigning "ch" as 0-7 will enable the corresponding multiplexer channels.
// Any other value of "ch" will disable all channels.
// Returns 0 on success, 1 on error.
int i2c_mux_select(i2c_mux_t* mux, int ch);

// Multiple multiplexer channels can be enabled or disabled simultaneously.
// The 8 bits of "mask" correspond to the 8 multiplexer channels.
// If a bit is 1 then the corresponding channel is enabled, otherwise it is disabled.
// Returns 0 on success, 1 on error.
int i2c_mux_select_multi(i2c_mux_t* mux, uint8_t mask);

#endif
