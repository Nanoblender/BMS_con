/*
Managing the HID interface.
For more info on descriptor https://eleccelerator.com/tutorial-about-usb-hid-report-descriptors/ 

*/

#include <libopencm3/usb/usbd.h>
#include <libopencm3/usb/hid.h>

#define GAMEPAD_REPORT_SIZE 3

extern struct usb_interface_descriptor hid_iface;
extern void hid_set_config(usbd_device *dev, uint16_t wValue);

struct gamepad_report_t
{
	uint16_t buttons;
	uint8_t x;
};

static const uint8_t hid_report_descriptor[] =
{
 0x05, 0x01, // USAGE_PAGE (Generic Desktop)
 0x09, 0x04, // USAGE (Joystick)
 0xa1, 0x01, // COLLECTION (Application)
 0xa1, 0x00,                    //   COLLECTION (Physical)
 0x05, 0x09, // USAGE_PAGE (Button)
 0x19, 0x01, // USAGE_MINIMUM (Button 1)
 0x29, 0x10, // USAGE_MAXIMUM (Button 16)
 0x15, 0x00, // LOGICAL_MINIMUM (0)
 0x25, 0x01, // LOGICAL_MAXIMUM (1)
 0x75, 0x01, // REPORT_SIZE (1)
 0x95, 0x10, // REPORT_COUNT (16)
 0x81, 0x02, // INPUT (Data,Var,Abs)
 			0x05, 0x01,                    //     USAGE_PAGE (Generic Desktop)
			0x09, 0x30,                    //     USAGE (X)
			0x15, 0x81,                    //     LOGICAL_MINIMUM (-127)
			0x25, 0x7f,                    //     LOGICAL_MAXIMUM (127)
			0x75, 0x08,                    //     REPORT_SIZE (8)
			0x95, 0x01,                    //     REPORT_COUNT (1)
			0x81, 0x02,                    //     INPUT (Data,Var,Abs)
 0xc0, // END_COLLECTION
 0xc0 // END_COLLECTION
};