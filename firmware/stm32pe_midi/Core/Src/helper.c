#include <stdint.h>
#include "TMAG5273.h"
#include "helper.h"

uint16_t map_float_to_uint16(float x, float in_min, float in_max,
		uint16_t out_min, uint16_t out_max) {
	if (x < in_min)
		x = in_min;
	if (x > in_max)
		x = in_max;
	return (uint16_t) (((x - in_min) * (out_max - out_min)) / (in_max - in_min)
			+ out_min);
}

void fill_midi_key_arr(uint8_t *arr, int size, int start_octave) {
	int midi_note = 11 + (start_octave * 12);
	for (int i = 0; i < size; i++) {
		arr[i] = midi_note;
		// Step to next note in chromatic order
		// B, C, C#, D, D#, E, F, F#, G, G#, A, A#
		// Intervals: 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1
		midi_note++;
	}
}

TMAG5273_Handle_t TMAG5273_CreateHandle(I2C_HandleTypeDef *hi2c,
		uint8_t address) {
	TMAG5273_Handle_t tmag = { .pI2c = hi2c, .address = address,
			.magTempcoMode = TMAG5273_NO_MAG_TEMPCO, .convAvgMode =
					TMAG5273_CONV_AVG_32X, .readMode =
					TMAG5273_READ_MODE_STANDARD, .lplnMode = TMAG5273_LOW_NOISE,
			.operatingMode = TMAG5273_OPERATING_MODE_STANDBY, .magXYRange =
					TMAG5273_MAG_RANGE_40MT_133MT, .magZRange =
					TMAG5273_MAG_RANGE_40MT_133MT, .magXYRange =
					TMAG5273_MAG_RANGE_40MT_133MT, .magZRange =
					TMAG5273_MAG_RANGE_80MT_266MT, .tempChEn =
					TMAG5273_TEMP_CH_DISABLED, .angEn = TMAG5273_ANG_X_Z,
			.magChEn = TMAG5276_MAG_Z_X, .crcEna = TMAG5273_CRC_DISABLE,
			.sensor_id = 0, .sleep = TMAG5276_10MS };

	return tmag;
}
