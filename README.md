# stm32pe-midi controller

<img width="1351" height="559" alt="image" src="https://github.com/user-attachments/assets/e3a31bd5-8311-4625-8dfc-6f1083a7edc2" />

An expressive midi controller based on the stm32 ecosystem. It uses the MPE midi protocol to pass through signals that act as gestures/effects including velocity/note to any music DAW. (like ableton)

 - Uses the powerful STM32H7
 - Uses USB-C OTG to connect to a device
 - Has per key PWM leds for indication
 - Low profile/one sided pcb design
 - Small key footprint for compactness
 - Two octaves + extra key before an octave

See it in action [here](https://www.youtube.com/watch?v=022ZFJn4nkc)!

# Why I Made It

This controller will be able to make music more accessible by being more affordable to the typical person. Most MPE midi controllers on the market cost between [$500 USD to $2000](https://www.expressivee.com/2-osmose). I also made this project to touch the stm32 ecosystem and learn how to integrate and use it. This midi keyboard could also make my workflow nicer for making music in Ableton.

![image](https://github.com/user-attachments/assets/fa9f4120-156a-4272-a7e7-c6c8a3e5700d)

# Design

To reduce the cost of production, I decided to design a compliant mechanism (flexure) that would be only one part. It would reduce the labor and cost by a lot. This is because most keyboards/pianos have between 10 to 100 which I have reduced to a minimum of 2. A hall effect sensor is used to measure the position and rotation of a magnet embedded in each key.

![image](https://github.com/user-attachments/assets/c9061eba-b947-4af8-a9f3-8442207598c3)

# Problems

There were many problems while prototyping and laying out things. I had bare knowledge of how flexures worked and had to create as many prototypes as needed until the right movement was made. I also didn't know that the size of the board would be a big constraint that was completely ignored.

Problems during fabrication
 * TMAG5273 has a fixed i2c address which only differes by buying different models (B1, C1, D1) or running extra interrupt traces
 * M screw spacers broke off when screwing in
 * Too high of a tolerance between the board and 3d printed parts, had to cut off bottom two aligners on all keys

Here's a list of problems after fabrication + firmare
 * Flats/sharp/black keys flexure design suffers too much with plastic deformation permanently, as well as temporarily
 * ^ This issue also causes a permanent distance change as there is now less distance to measure than before
 * TMAG5273 library has no calibration feature and the measured angle is offset when brought closer
 * Keys are a bit too slippery and need a dimple to grip the finger
 * Flexures on both key types are a little on the firm side, takes a little time to get used to
 * Last white key (b) is impractical as header wires are blocking its roll to pitch change up
 * The power on led is a bit too bright, not changeable
 * The base/case of the stm32h7 is kinda slippery
 * PLA/flexures gets deformed under heat in a car which can leave a permanent loss of height on keys

These issues do not make the stm32pe midi keyboard worthless or horrible to use, but are just little tiny problems that could be polished in the future.

# Firmware

 * [tinyusb lib](https://github.com/hathach/tinyusb) on MIT license
 * [i2c-mux stm32 lib](https://github.com/jtainer/i2c-mux) on BSD-2-Clause license
 * [stm32 tmag5273 lib](https://github.com/devOramaMan/stm32_TMAG5273) on No license (added a i2c address rewrite based on the datasheet as well as a crucial fix)

Thank you to all the open source libraries above to make this project possible. I don't think I would have the expertise to implement all my ics this well.

The firmware for the stm32pe midi controller is written in c/cpp on stm32cubeide and flashed with stm32cubeprogrammer. Amazing platform. Outdated UI (lol). **To be as transparent as possible**, some of the firmware was written/assisted with AI. About 20% of it was written mostly with AI, while the rest + logic that ties everything together was me. `mpe.c`, 80% of `helper.c`, 60% of `tlc5940.c`, and 5% of `main.c` with some fixes to `TMAG5273.c` was assisted with AI. I do understand what the firmware does. Depending on your perspective on AI, it shouldn't really matter as I know what I'm doing. 

# Wiring

![image](https://github.com/user-attachments/assets/4aaa0f81-a04c-46bb-bf4a-90668fc7458c)
![image](https://github.com/user-attachments/assets/6c7d2a6b-d729-48a6-98f4-0517bcc13cc6)
![image](https://github.com/user-attachments/assets/2c60ebdc-f097-4882-bf1e-23de492c6ce5)

# BOM
This is an approximate bill of materials that is a little flexible. It would be around $333.27, ±5 dollars in case of shipping or extra taxes/fees. Thank you Hack Club - Highway for funding this.

| Part                       | Cost         | Count | Supplier | Note                                                           | Link                                                                                            |
|----------------------------|--------------|-------|----------|----------------------------------------------------------------|-------------------------------------------------------------------------------------------------|
| PLA Filament (white)       | $16.99       | 1     | Amazon   | I have a 3d printer and would like to print most of the parts. | [https://www.amazon.com/dp/B07PGZNM34](https://www.amazon.com/dp/B07PGZNM34)                    |
| PLA Filament (black)       | $16.99       | 1     | Amazon   | ~ (for the black keys)                                         | [https://www.amazon.com/dp/B07PGY2JP1](https://www.amazon.com/dp/B07PGY2JP1)                    |
| UV Plastic Glue            | $14.15       | 1     | Amazon   | This is for the underside of the keys to stop scraping.        | [https://www.amazon.com/dp/B00QU5M4VW](https://www.amazon.com/dp/B00QU5M4VW)                    |
| Header 10 Pins             | $0.78        | 2     | DigiKey  | (does not include shipping/taxes)                              | [68000-110HLF](https://www.digikey.com/en/products/detail/amphenol-cs-fci/68000-110HLF/1878503) |
| Header 5 Pins              | $0.63        | 1     | DigiKey  | ~                                                              | [68000-105HLF](https://www.digikey.com/en/products/detail/amphenol-cs-fci/68000-105HLF/2023304) |
| PCB Fabrication + Assembly | $151.09      | 2     | JLCPCB   | *no coupons applied yet                                        |                                                                                                 |
| PCB Shipping               | $33.52       | n/a   | ~        |                                                                |                                                                                                 |
| Customs duties & taxes     | $83.10       | n/a   | ~        |                                                                |                                                                                                 |
| Sales Tax                  | $15.24       | n/a   | ~        |                                                                |                                                                                                 |
| TOTAL                      | $333.27      |       |          |                                                                |                                                                                                 |

I forgot to add two more parts to the bom. They have been covered out of pocket which is fine

| Part                  | Cost    | Count | Supplier | Note                                                                                                                                                         | Link                                                                                                   |
|-----------------------|---------|-------|----------|--------------------------------------------------------------------------------------------------------------------------------------------------------------|--------------------------------------------------------------------------------------------------------|
| 5mm Cube Magnets      | $16.99  | 45    | Amazon   | I could not find any listings with lower quantities                                                                                                          | [https://www.amazon.com/dp/B0CW9GVTX2](https://www.amazon.com/dp/B0CW9GVTX2)                           |
| M3 5mm Screws         | ¥2.14   | 50    | Taobao   | Pretty cheap! (30 cents usd)                                                                                                                                 | [https://detail.tmall.com/item.htm?id=722759297726](https://detail.tmall.com/item.htm?id=722759297726) |
| JLCPCB Two Design Fee | $38.99  | n/a   | JLCPCB   | 😭 i did not think about this while designing, i tried negotiating and doing everything                                                                      |                                                                                                        |
| TMAG5273-B1/C1/D1     | $23.28  | 18    | Mouser   | i stg the datasheet said the sensor's i2c addresses were reprogrammable.. but they didn't say i needed to hookup interrupts to them too... lesson learned :p |                                                                                                        |
| Total out of pocket   | $79.56  |       |          |                                                                                                                                                              |                                                                                                        |
| REAL TOTAL            | $412.83 |       |          |                                                                                                                                                              |                                                                                                        |
