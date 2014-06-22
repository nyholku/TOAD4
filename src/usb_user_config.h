/*
 * usb_user_config.h
 *
 */

#ifndef USB_USER_CONFIG_H_
#define USB_USER_CONFIG_H_

#include "usb_defs.h"

// usb_user_config .c/.h contains the USB stack configuration and are the only files you as the stack user should be editing

#define USB_VID 0x1D50
#define USB_PID 0x6020

#define USB_MANUFACTURER_STRING  'S', 'p', 'a', 'r', 'e', 'T', 'i', 'm', 'e', 'L', 'a', 'b', 's'
#define USB_PRODUCT_STRING 'T', 'O', 'A', 'D', '4'

codePtr usb_user_get_device_descriptor();
codePtr usb_user_get_configuration_descriptor();
codePtr usb_user_get_hid_descriptor();
codePtr usb_user_get_hid_report_descriptor();
uint8_t usb_user_get_hid_report_descriptor_length();

#endif /* USB_USER_CONFIG_H_ */
