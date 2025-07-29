#ifndef USB_DESCRIPTORS_H
#define USB_DESCRIPTORS_H

#include "tusb.h"

// Device descriptor
uint8_t const* tud_descriptor_device_cb(void);

// Configuration descriptor
uint8_t const* tud_descriptor_configuration_cb(uint8_t index);

// String descriptor
uint16_t const* tud_descriptor_string_cb(uint8_t index, uint16_t langid);

#endif
