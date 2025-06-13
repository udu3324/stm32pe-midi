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

# June 9th: Designing the flexures for the keys

I started off on a new page and drew two concepts of how I could implement [flexures](https://web.mit.edu/mact/www/Blog/Flexures/FlexureIndex.html) in a key. It would allow me to build the device with less parts with the same functionality, as well as saving on money. The print layers also had to be thought of to make sure flexing won't compromise on durability and break. 

Below is some rough sketches of how the keys would look like and how it would come together with the pcb. 

![image](https://github.com/user-attachments/assets/45038e86-c804-486a-84d6-df553b95d1cc)

**Total time spent: 2 hours**

# June 10th: Cadding and 3d printing key prototypes

I decided that I'll choose the more simpler key with only one model that i need to create. I am using Fusion 360 to accomplish this with their personal plan.

Key Requirements (ahaha get it)
 - Fits a 5mm cube neodymium magnet
 - Flexes easily, doesn't strain fingers
 - Can flex in all the neccesary axis
 - Use little to no extra plastic when printing

## Prototype V1

![v1](https://github.com/user-attachments/assets/73cd9562-37d2-4ac5-a7ac-75ec2b59992d)

This was the first prototype to kickstart my idea of a key. I started off with a side sketch and extruded it, then adding features in the other dimension. 

![image](https://github.com/user-attachments/assets/819f46e1-ac3f-4cde-a41d-1ba0ac762de0)

Well.. It did not turn out great the first try, and of course it would never be perfect the first try. I designed the flexure to be too thin and it wouldn't hold up the upper weight of the key. It also could not fit the magnet.

Instead of it flexing left or right, the key decided to rotate in place which i did not intend to. But wait... This gave me a new **game breaking** idea! Instead of+ designing it to move in two axises, I can just have it move in one axis and rotate in place which can also be measured by the sensor! It is something so unique that I could have never thought of without this prototype. 

## Protype V2

![v2](https://github.com/user-attachments/assets/404164a9-7ace-402b-8909-9496b2a2501b)

This second prototype solved many problems of the first one. It finally held the magnet, its own weight, and almost has the rotation action perfect.

![image](https://github.com/user-attachments/assets/697a05ec-768d-458f-8234-b37619ebfad9)

The only two problems now is that it is a bit stiff for a key on a midi keyboard as well as something I didn't think about for fabrication. Pressing it all the way down while trying to rotate it scrapes the layer lines making an irritating noise. 

## Prototype V3

![v3](https://github.com/user-attachments/assets/1b5caa83-9d4c-4ef3-8dd3-1ef0b61ddda2)

This prototype solved the issue of the key being too stiff. It requires less force to push down now, but still has the scraping problem which I'm not sure how to solve. I'll do that later lol.

![image](https://github.com/user-attachments/assets/a570f237-cc6d-4073-9218-a842fe3393ff)

The data isn't as smooth as i hoped, but I think thats an issue in the code. The white line looking like a sine wave is the rotation movement of the key.

**Total time spent: 5 hours** (2 hrs of printing)

# June 11-13th: Prototyping the smaller keys

I decided that there would be a smaller version of a key for the sharps and flats on the controller. It seemed easy at first until changing the length of a key would mean tweaking every other aspect of the flexure endlessly.

![image](https://github.com/user-attachments/assets/2eff6605-5d60-4495-8d86-32789546f280)

A total of 13 prototypes were printed. Many problems were faced, including but not limited to.... low durability, permanent deformation, too much pressure to push down, no springines, and my lack of knowledge in making flexures. Although I chose aesthetics over functionality, I think everything will turn out great. 

**Total time spent: 2.5 hours** (accumulated, not including hours of printing)
