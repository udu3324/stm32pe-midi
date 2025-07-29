#ifndef TUSB_CONFIG_H
#define TUSB_CONFIG_H

#include "main.h"

// ty https://github.com/hathach/tinyusb/discussions/633#discussioncomment-342237

#define CFG_TUSB_MCU                 OPT_MCU_STM32H7
#define CFG_TUSB_OS                  OPT_OS_NONE

#define CFG_TUD_ENDPOINT0_SIZE       64
#define BOARD_DEVICE_RHPORT_SPEED    OPT_MODE_FULL_SPEED
#define BOARD_DEVICE_RHPORT_NUM      0
#define CFG_TUSB_RHPORT0_MODE        (OPT_MODE_DEVICE | OPT_MODE_FULL_SPEED)

#define CFG_TUD_CDC                  1
#define CFG_TUD_CDC_TX_BUFSIZE       512
#define CFG_TUD_CDC_RX_BUFSIZE       512

#define CFG_TUD_MIDI                 1
#define CFG_TUD_MIDI_TX_BUFSIZE      256
#define CFG_TUD_MIDI_RX_BUFSIZE      64

// In the case of the STM32H7 with an external HS 480 PHY, you must use root hub port 1 instead of 0
//    0 is for the internal FS 12mbit PHY so you'd use BOARD_DEVICE_RHPORT_NUM set to 0 and CFG_TUSB_RHPORT1_MODE set to (OPT_MODE_DEVICE | OPT_MODE_FULL_SPEED)

// vvv<...> (see tinyusb examples for remainder, configuring USB Class and buffer size basics)

#endif
