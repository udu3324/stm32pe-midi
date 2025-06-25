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

# June 16th: Creating board schematics

I started working on the schematics for the board to bring all the components I need together.

![image](https://github.com/user-attachments/assets/9b095556-8960-4b0f-af6c-f5bf798f5661)

So far, I have been following the [power supply scheme](https://www.st.com/resource/en/datasheet/stm32h743vi.pdf) from the stm32 datasheet as well as researching some other sources. I chose KiCad to build my board in as it is free and an amazing open source tool. 

I still need to add some sort of fuse to the VBUS of the usb-c to protect against esd, as well as my sensors, leds, possibly a dac with audio out, and most definitely a button to allow flashing firmware to the mcu. 

**Total time spent: 3 hours**

# June 17th: Adding the rest

The boot and reset button needed to flash firmware was added as well as some useful indicator LEDs.

![image](https://github.com/user-attachments/assets/f0315bbd-7b02-414f-bda3-ee53db0c141b)

I decided to create a sub-sheet to fit all my sensors on another page. 

![image](https://github.com/user-attachments/assets/1c5f274c-731e-438f-b1e3-1929e54cbb0a)

**Total time spent: 3 hours**

# June 18th: Working on cad assembly

I decided that I would bring everything together to then model the size of the pcb to then take into KiCad. Currently, I am only familiar with OnShape's assembly feature and have no clue how it will go. 

![Untitled](https://github.com/user-attachments/assets/21fadf20-c203-47ec-8d2b-428e16d82cbf)

It went horrible of course. I tried editing my sub-assemblies to add a pcb groove but kept on running into so many issues. It took many sketches and saving/syncing issues until I finally was able to modify parts across f3d projects without f**king everything up. 

![image](https://github.com/user-attachments/assets/4159961d-e60d-4a9c-8dc8-1cdc8b276737)

I made sure to include a 0.15mm gap which would hopefully allow some tolerance for the pcb to fit. (±0.2mm for shape, ±10% for thickness) It would also create a friction fit for the board as well.

**Total time spent: 4 hours**

# June 20th: Adding leds + more

![image](https://github.com/user-attachments/assets/ba1aeb4b-c904-42d8-9c17-c02ffc613f8d)

I had to multiplex my leds to use less io on the stm32. The TLC5940 seemed like the best fit for me as it was pwm and would be nice if each key would have some sort of power indicator. 

I also used KiCad's feature of assigning every symbol to a footprint. I decided on every capacitor and resistor having the same package size (0805/2012 metric) and it being not microscopic in case of any repairs and to simplify the bom.

![image](https://github.com/user-attachments/assets/4de164a9-6b1b-45ac-ab16-cbfdafc9685d)

I decided to add a way to output audio. It uses a MCP4922 and two LM396s to allow simple, not audiophile quality, audio delivery to the user.

![image](https://github.com/user-attachments/assets/b55f5278-1351-45b6-abc9-be991cfdd276)

I was able to test a more simplified/mono design of the audio circuit some time ago with the parts I got from high seas.

**Total time spent: 2 hours**

# June 22th: Cadding plate + pcb layout

![image](https://github.com/user-attachments/assets/9c715d33-bfa5-4c9b-983a-5dedf1defa2b)

I finished up the plate/thing that would hold everything together. So far, most things are press fit into place while the main board uses m3 screws.

![image](https://github.com/user-attachments/assets/794ea05e-b816-4f78-b141-f46df08ebbc2)

I had no clue what i was getting myself into. Everything is laid out with the proper resistors or capacitors next to each other, but its a super dense board which creates a whole list of other challenges.

It was a pain trying to get everything to fit in the first place. I had to remove two extra leds, remove the entire audio output circuit (rip..), redo some schematics, and contemplate my life choices. 

![image](https://github.com/user-attachments/assets/fcb35233-848d-4590-a504-67ed48e1d579)

All that's needed left is to route all the traces and place the pullup resistors for the sda/scl lines, as well as calculating the amount of headers i need for the wires between the three boards.

**Total time spent: 7 hours**

# June 23th + 24th: Unseen Constraints - Routing All Traces

![image](https://github.com/user-attachments/assets/5a1312ef-efad-41c4-9509-1b6b9fc4b3fa)

I should have thought of the size of my board before doing anything. That was my hugest mistake. I was constrainted to the size/shape of my board which I never thought of while cadding only until now after some work was done.

I had to...

 - remove some features
 - resize the board to fit some components
 - constantly mess around with previous traces (because past me did not think about the future)
 - edit the heat spreader of the led multiplexer to be smaller which is kinda sus but who cares lol
 - cope

![image](https://github.com/user-attachments/assets/3e2c604a-bf54-4533-8c70-40bc96dfeb13)

In the end, I managed to get everything wired up. KiCad still yells about certain things being unconnected as well as a list of other stuff that isnt really important. I'm really proud of the work as this is a new high for me.

![image](https://github.com/user-attachments/assets/82984a8d-efe5-418b-a55d-c17eee4ce423)

While 80% done tracing everything, I decided to export the pcb to jlcpcb to see how much it would cost so far. It was priced around ~$140 USD for assembling two boards. I then clicked checkout and... I forgot there was tariffs going on right now.. (totaling around $230)

**Total time spent: ~14 hours**

# June 25th: Final touches + more

![kicad_t1TWV0PZLa](https://github.com/user-attachments/assets/4e8a43e9-ccc3-492c-bd33-839026a96d45)

After routing everything, I made sure i had no critical errors by using KiCad's design rule checker (DRC for short). It revealed a ton of problems like unconnected grounds, traces, and more. Although some components were marked as too close to others, it was only by a little and i could safely ignore it.

![image](https://github.com/user-attachments/assets/0bb56d30-c393-4b80-9bf8-0e8923adb3f5)

![image](https://github.com/user-attachments/assets/95bb8321-62ed-454d-83b1-672675b4ce6b)

I have no clue how i didn't notice this at all. I placed my sensor in the wrong place and only found out last second. It wasn't a pain to move though.

![image](https://github.com/user-attachments/assets/a87f269d-1335-4bd0-a22c-23cade6d2a16)

Here's the actual final final look of the pcb. It is amazing.

**Total time spent: 3 hours**
