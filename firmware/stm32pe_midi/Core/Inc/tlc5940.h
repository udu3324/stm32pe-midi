#ifndef TLC5940_H
#define TLC5940_H

#include "main.h"

#define TLC5940_NUM_CHIPS   2
#define TLC5940_CHANNELS    (TLC5940_NUM_CHIPS * 16)
#define TLC5940_NUM_LEDS    25
#define TLC5940_USED_LEDS   32
#define TLC5940_MAX_PWM  4095  // 12-bit PWM max value

// Initialize TLC5940 pins and SPI (called once)
void TLC5940_Init(void);

// Update all LEDs brightness buffer and push to TLC5940s
void TLC5940_Update(void);

// using setLED will have working indexes from 0-9 and 16-32 only
void TLC5940_SetLED(uint8_t led_index, uint16_t brightness);

// this uses setLED and maps them to combine working indexes for ease of use
void TLC5940_SetMappedPhysicallyLED(uint8_t logical_index, uint16_t brightness);

// uses the physical mapping to then map by b, c, c#, d, etc.
void TLC5940_SetMappedByKeyLED(uint8_t key_index, uint16_t brightness);

// tests below, including update
void TLC5940_TestPattern(void);
void TLC5940_AllOnTest(void);
void TLC5940_BreatheTest(void);
void TLC5940_BreatheAllTest(void);

#endif // TLC5940_H
