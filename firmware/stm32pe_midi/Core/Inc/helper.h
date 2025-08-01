#ifndef HELPER_H
#define HELPER_H

uint16_t map_float_to_uint16(float x, float in_min, float in_max,
		uint16_t out_min, uint16_t out_max);

int8_t map_velocity_log(uint32_t t);

void fill_midi_key_arr(uint8_t *arr, int size, int start_octave);

TMAG5273_Handle_t TMAG5273_CreateHandle(I2C_HandleTypeDef *hi2c,
		uint8_t address);

#endif
