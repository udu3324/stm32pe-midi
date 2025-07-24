# stm32pe-midi controller

![stm32pe-midi case v41](https://github.com/user-attachments/assets/fadfc3aa-1e23-4ed8-a343-4e25dc6622fd)

An expressive midi controller based on the stm32 ecosystem. It uses the MPE midi protocol to pass through signals that act as gestures/effects including velocity/note to any music DAW. (like ableton)

 - Uses the powerful STM32H7
 - Uses USB-C OTG to connect to a device
 - Has per key PWM leds for indication
 - Low profile/one sided pcb design
 - Small key footprint for compactness
 - Two octaves + extra key before an octave

# Why I Made It

This controller will be able to make music more accessible by being more affordable to the typical person. Most MPE midi controllers on the market cost between [$500 USD to $2000](https://www.expressivee.com/2-osmose). I also made this project to touch the stm32 ecosystem and learn how to integrate and use it. This midi keyboard could also make my workflow nicer for making music in Ableton.

![image](https://github.com/user-attachments/assets/fa9f4120-156a-4272-a7e7-c6c8a3e5700d)

# Design

To reduce the cost of production, I decided to design a compliant mechanism (flexure) that would be only one part. It would reduce the labor and cost by a lot. This is because most keyboards/pianos have between 10 to 100 which I have reduced to a minimum of 2. A hall effect sensor is used to measure the position and rotation of a magnet embedded in each key.

![image](https://github.com/user-attachments/assets/c9061eba-b947-4af8-a9f3-8442207598c3)

# Problems

There were many problems while prototyping and laying out things. I had bare knowledge of how flexures worked and had to create as many prototypes as needed until the right movement was made. I also didn't know that the size of the board would be a big constraint that was completely ignored.

# Firmware

[i2c-mux stm32 lib](https://github.com/jtainer/i2c-mux) on BSD-2-Clause license
[stm32 tmag5273 lib](https://github.com/devOramaMan/stm32_TMAG5273) on No License

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

| Part                  | Cost    | Count | Supplier | Note                                                                                    | Link                                                                                                   |
|-----------------------|---------|-------|----------|-----------------------------------------------------------------------------------------|--------------------------------------------------------------------------------------------------------|
| 5mm Cube Magnets      | $16.99  | 45    | Amazon   | I could not find any listings with lower quantities                                     | [https://www.amazon.com/dp/B0CW9GVTX2](https://www.amazon.com/dp/B0CW9GVTX2)                           |
| M3 5mm Screws         | ¥2.14   | 50    | Taobao   | Pretty cheap! (30 cents usd)                                                            | [https://detail.tmall.com/item.htm?id=722759297726](https://detail.tmall.com/item.htm?id=722759297726) |
| JLCPCB Two Design Fee | $38.99  | n/a   | JLCPCB   | 😭 i did not think about this while designing, i tried negotiating and doing everything |                                                                                                        |
| Total out of pocket   | $56.28  |       |          |                                                                                         |                                                                                                        |
| REAL TOTAL            | $389.55 |       |          |                                                                                         |                                                                                                        |
