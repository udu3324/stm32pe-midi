---
title: "stm32pe midi controller"
author: "@Danny"
description: "An affordable music keyboard that enables more expression through the MIDI Polyphonic Expression (MPE) protocol."
created_at: "2025-06-06"
---

# June 6th: Sketching out the idea

Today, I started researching the components that I would depend on. (stm32 ecosystem, i2c multiplexers, sensors) I then created some technical sketches of the device and drew some core concepts. 

Feature Set
 - Recognizable by Ableton (or any general DAW software)
 - Sends mpe midi (duh)
 - USB-C connection for flashing & otg device
 - (optional) 3.5mm jack for int. synth engine
 - Cool rgb indicators

![image](https://github.com/user-attachments/assets/6459d8a0-7505-47d1-8316-bf31698aa046)

Components Needed
 - **STM32H7**43VIH (great mcu, bga style needing multilayer pcb fab)
 - TMAG5273 - I2C hall effect sensor
 - TCA9548A - For multiplexing i2c connections
 - Possibly SDRAM for an internal synth engine
 - IO Ports, Encoder
 - other smd components


**Total time spent: ~3 hours**

# June 8th: Getting the TMAG5273 to work

The core idea of this controller was to use a hall effect sensor (tmag5273) to measure the position of a magnet and output many things out of it. Before continuing any further, I need to make sure that it works.

## Wiring

Two resources were used in wiring the sensor. I looked at the [datasheet](https://www.ti.com/lit/ds/symlink/tmag5273.pdf) first to get an idea of the pinout and reference circuit, as well as how sparkfun implemented it through their board [here](https://docs.sparkfun.com/SparkFun_Qwiic_Hall_Effect_Sensor_TMAG5273/assets/board_files/schematic-mini.pdf). 

I decided for now to use my [esp32 devkit](https://docs.espressif.com/projects/esp-dev-kits/en/latest/esp32s3/esp32-s3-devkitc-1) to test out the sensor before switching to the stm32 ecosystem.

The INT pin for disabling the sensor was left unconnected, SDA and SCL to any GPIO with a 4.7k pullup resistor, and VCC to 3.3V. 

![image](https://github.com/user-attachments/assets/82824afd-b3b3-41d3-bf74-3a4f5ccb9313)

## Code

I am using PlatformIO's VSC plugin to flash and test everything. The first thing to flash was a simple [I2C device scanner](https://randomnerdtutorials.com/esp32-i2c-communication-arduino-ide) to see if I had everything wired correctly. This process took quite long as I did not know if the sensor was fried or if my wiring was horrible. (it was both 😭)

I can then move onto [sparkfun's library](https://github.com/sparkfun/SparkFun_TMAG5273_Arduino_Library) to communicate with the tmag sensor.

```cpp
#include <Arduino.h>
#include <Wire.h>
#include "SparkFun_TMAG5273_Arduino_Library.h" 

TMAG5273 sensor;

uint8_t i2cAddress = 0x35;

const int ledPin = 17;  // GPIO 17
const int I2C_SDA = 4;
const int I2C_SCL = 5;

void setup() {
  Serial.begin(115200);
  delay(1000);
  pinMode(ledPin, OUTPUT);  // Set GPIO 17 as an output
  Wire.begin(I2C_SDA, I2C_SCL);

  Serial.println("hello world");

  // If begin is successful (0), then start example
  if (sensor.begin(i2cAddress, Wire) == 1) {
    Serial.println("Begin");
  } else {
    Serial.println("Device failed to setup - Freezing code.");
  }
}

void loop() {
  // Checks if mag channels are on - turns on in setup
  if (sensor.getMagneticChannel() != 0) {
    sensor.setTemperatureEn(true);

    float magX = sensor.getXData();
    float magY = sensor.getYData();
    float magZ = sensor.getZData();
    float temp = sensor.getTemp();

    //Serial.printf("( %7.2f, %7.2f, %7.2f ) mT   %6.2f C\n", magX, magY, magZ, temp);
    
    Serial.printf("%f,%f,%f,%f\n", magX, magY, magZ, temp);
    //relative to dot on ic facing forward to viewer
    // left-right, forward-backward, up-down
  } else {
    Serial.println("Mag Channels disabled, stopping..");
  }

  delay(50);
}
```

![image](https://github.com/user-attachments/assets/bb2c4166-27de-40a5-b84f-6ef6e17467a9)

**Total time spent: 3.5 hours**
