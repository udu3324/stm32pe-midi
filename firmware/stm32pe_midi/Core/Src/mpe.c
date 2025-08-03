#include <stdint.h>
#include <stdbool.h>

#include "tusb.h"


#define MAX_MPE_CHANNELS 15
#define BASE_MPE_CHANNEL 1 // Channels 1 to 15 for notes
#define MPE_GLOBAL_CHANNEL 0 // Channel 0 is the global/master channel

typedef struct {
	uint8_t channel;
	uint8_t note;
	bool active;
} MPE_NoteSlot;

MPE_NoteSlot mpe_slots[MAX_MPE_CHANNELS] = { 0 };

static bool mpe_initialized = false;

//internal - Initialize MPE mode
void MPE_Init(void) {
	if (mpe_initialized) return;
	
	printf("Initializing MPE mode...\r\n");
	
	// Wait a bit to ensure USB connection is stable
	// Note: This should be called after USB is ready
	
	// Send MPE Configuration Message (MCM) to configure MPE Zone
	// RPN 6 (MPE Configuration Message) on global channel (channel 0)
	
	// RPN MSB (101): 0
	uint8_t rpn_msb[3] = { 0xB0 | MPE_GLOBAL_CHANNEL, 101, 0 };
	tud_midi_stream_write(0, rpn_msb, 3);
	
	// RPN LSB (100): 6 (MPE Configuration)
	uint8_t rpn_lsb[3] = { 0xB0 | MPE_GLOBAL_CHANNEL, 100, 6 };
	tud_midi_stream_write(0, rpn_lsb, 3);
	
	// Data Entry MSB (6): Number of member channels (15)
	uint8_t data_msb[3] = { 0xB0 | MPE_GLOBAL_CHANNEL, 6, MAX_MPE_CHANNELS };
	tud_midi_stream_write(0, data_msb, 3);
	
	// Data Entry LSB (38): 0 (not used for MPE)
	uint8_t data_lsb[3] = { 0xB0 | MPE_GLOBAL_CHANNEL, 38, 0 };
	tud_midi_stream_write(0, data_lsb, 3);
	
	// Reset RPN to null
	uint8_t rpn_null_msb[3] = { 0xB0 | MPE_GLOBAL_CHANNEL, 101, 127 };
	tud_midi_stream_write(0, rpn_null_msb, 3);
	
	uint8_t rpn_null_lsb[3] = { 0xB0 | MPE_GLOBAL_CHANNEL, 100, 127 };
	tud_midi_stream_write(0, rpn_null_lsb, 3);
	
	printf("MPE initialization complete\r\n");	
	mpe_initialized = true;
}

//internal
int allocate_mpe_channel(uint8_t note) {
	for (int i = 0; i < MAX_MPE_CHANNELS; i++) {
		if (!mpe_slots[i].active) {
			mpe_slots[i].channel = BASE_MPE_CHANNEL + i;
			mpe_slots[i].note = note;
			mpe_slots[i].active = true;
			printf("Allocated MPE channel %d for note %d\r\n", mpe_slots[i].channel, note);
			return mpe_slots[i].channel;
		}
	}

	// no free channels
	printf("Warning: No free MPE channels for note %d\r\n", note);
	return -1;
}

//internal
int release_mpe_channel(uint8_t note) {
	for (int i = 0; i < MAX_MPE_CHANNELS; i++) {
		if (mpe_slots[i].active && mpe_slots[i].note == note) {
			int channel = mpe_slots[i].channel;
			mpe_slots[i].active = false;
			printf("Released MPE channel %d for note %d\r\n", channel, note);
			return channel;
		}
	}

	printf("Warning: Could not find active MPE channel for note %d\r\n", note);
	return -1;
}

void MPE_Send_Note_On(uint8_t note, uint8_t velocity) {
	// Ensure MPE is initialized
	if (!mpe_initialized) {
		MPE_Init();
	}
	
	int channel = allocate_mpe_channel(note);
	if (channel < 0)
		return; // polyphony limit reached

	uint8_t msg[3] = { 0x90 | (channel & 0x0F), note, velocity };
	tud_midi_stream_write(0, msg, 3);
}

void MPE_Send_Note_Off(uint8_t note) {
	int channel = release_mpe_channel(note);
	if (channel < 0)
		return; // note not found

	uint8_t msg[3] = { 0x80 | (channel & 0x0F), note, 0 };
	tud_midi_stream_write(0, msg, 3);
}

// bend_val: -8192 to +8191, do not use the max as it would be about 90 semitones
void MPE_Send_Pitch_Bend(uint8_t note, int16_t bend_val) {
	uint16_t bend = bend_val + 8192; // convert to unsigned

	for (int i = 0; i < MAX_MPE_CHANNELS; i++) {
		if (mpe_slots[i].active && mpe_slots[i].note == note) {
			uint8_t channel = mpe_slots[i].channel;
			uint8_t msg[3] = { 0xE0 | (channel & 0x0F), bend & 0x7F, (bend >> 7)
					& 0x7F };
			tud_midi_stream_write(0, msg, 3);
			
			//printf("Pitch bend: note=%d, channel=%d, bend_val=%d, bend=%d\r\n", note, channel, bend_val, bend);
			return;
		}
	}
	
	printf("Warning: Pitch bend for note %d not found in active slots\r\n", note);
}

//0-127 pressure
void MPE_Send_Aftertouch(uint8_t note, uint8_t pressure) {
	for (int i = 0; i < MAX_MPE_CHANNELS; i++) {
		if (mpe_slots[i].active && mpe_slots[i].note == note) {
			uint8_t channel = mpe_slots[i].channel;
			uint8_t msg[2] = { 0xD0 | (channel & 0x0F), pressure };
			tud_midi_stream_write(0, msg, 2);

			//printf("Aftertouch: note=%d, channel=%d, pressure=%d\r\n", note, channel, pressure);
			return;
		}
	}

	printf("Warning: Aftertouch for note %d not found in active slots\r\n",
			note);
}

