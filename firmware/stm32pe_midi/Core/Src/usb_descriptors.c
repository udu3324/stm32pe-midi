#include "tusb.h"
#include "tusb_config.h"

#include "bsp/board_api.h"

//------------- Product ID configuration -------------//
#define _PID_MAP(itf, n)  ( (CFG_TUD_##itf) << (n) )
#define USB_PID           (0x4000 | _PID_MAP(CDC, 0) | _PID_MAP(MIDI, 3))

//------------- Device Descriptor -------------//
tusb_desc_device_t const desc_device = { .bLength = sizeof(tusb_desc_device_t),
		.bDescriptorType = TUSB_DESC_DEVICE, .bcdUSB = 0x0200, .bDeviceClass =
				0x00, .bDeviceSubClass = 0x00, .bDeviceProtocol = 0x00,
		.bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE,

		.idVendor = 0xCafe, .idProduct = USB_PID, .bcdDevice = 0x0100,

		.iManufacturer = 0x01, .iProduct = 0x02, .iSerialNumber = 0x03,

		.bNumConfigurations = 0x01 };

uint8_t const* tud_descriptor_device_cb(void) {
	return (uint8_t const*) &desc_device;
}

//------------- Configuration Descriptor -------------//

#define CONFIG_TOTAL_LEN  (TUD_CONFIG_DESC_LEN + TUD_CDC_DESC_LEN + TUD_MIDI_DESC_LEN)
#define EPNUM_CDC_NOTIF   0x82
#define EPNUM_CDC_OUT     0x03
#define EPNUM_CDC_IN      0x84

#define EPNUM_MIDI_OUT    0x01
#define EPNUM_MIDI_IN     0x81

// for cdc
enum {	ITF_NUM_CDC = 0,
	ITF_NUM_CDC_DATA,
	ITF_NUM_MIDI,
	ITF_NUM_MIDI_STREAMING,
	ITF_NUM_TOTAL
};

uint8_t const desc_fs_configuration[] = {
  TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, CONFIG_TOTAL_LEN, 0x00, 100),

  // CDC Interface
  TUD_CDC_DESCRIPTOR(
      ITF_NUM_CDC, 0, EPNUM_CDC_NOTIF, 8,
						EPNUM_CDC_OUT, EPNUM_CDC_IN, 64),

  // MIDI Interface
  TUD_MIDI_DESCRIPTOR(
      ITF_NUM_MIDI, 0, EPNUM_MIDI_OUT,
						EPNUM_MIDI_IN, 64)
};



uint8_t const* tud_descriptor_configuration_cb(uint8_t index) {
	(void) index;
	return desc_fs_configuration;
}

//------------- String Descriptors -------------//

// for midi
enum {
STRID_LANGID = 0, STRID_MANUFACTURER, STRID_PRODUCT, STRID_SERIAL,
};



char const *string_desc_arr[] = { (const char[] ) { 0x09, 0x04 }, "stm32pe",
		"stm32pe midi device",
		NULL };

static uint16_t _desc_str[32 + 1];

uint16_t const* tud_descriptor_string_cb(uint8_t index, uint16_t langid) {
	(void) langid;
	size_t chr_count;

	if (index == STRID_LANGID) {
		memcpy(&_desc_str[1], string_desc_arr[0], 2);
		chr_count = 1;
	} else if (index == STRID_SERIAL) {
		chr_count = board_usb_get_serial(_desc_str + 1, 32);
	} else {
		const char *str = string_desc_arr[index];
		if (!str)
			return NULL;

		chr_count = strlen(str);
		if (chr_count > 32)
			chr_count = 32;

		for (size_t i = 0; i < chr_count; i++) {
			_desc_str[1 + i] = str[i];
		}
	}

	_desc_str[0] = (TUSB_DESC_STRING << 8) | (2 * chr_count + 2);
	return _desc_str;
}
