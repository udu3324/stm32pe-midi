#ifndef HELPER_H
#define HELPER_H

int16_t map_float_to_int16(float x, float in_min, float in_max,
		int16_t out_min, int16_t out_max);

uint8_t map_float_to_uint8(float x, float in_min, float in_max, uint8_t out_min,
		uint8_t out_max);

int8_t map_velocity_log(uint32_t t);

void fill_midi_key_arr(uint8_t *arr, int size, int start_octave);

TMAG5273_Handle_t TMAG5273_CreateHandle(I2C_HandleTypeDef *hi2c,
		uint8_t address);

#endif
