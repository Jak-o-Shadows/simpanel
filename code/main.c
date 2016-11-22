/*
 * This file is part of the unicore-mx project.
 *
 * Copyright (C) 2010 Gareth McMullin <gareth@blacksphere.co.nz>
 * Copyright (C) 2011 Piotr Esden-Tempski <piotr@esden.net>
 * Copyright (C) 2015 Kuldeep Singh Dhaka <kuldeepdhaka9@gmail.com>
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdlib.h>
#include <unicore-mx/cm3/nvic.h>
#include <unicore-mx/cm3/systick.h>
#include <unicore-mx/usbd/usbd.h>
#include <unicore-mx/usb/class/hid.h>
#include <unicore-mx/usbd/misc/string.h>
#include "usbhid-target.h"

/* Define this (in the Makefile CFLAGS) to include the DFU APP interface. */

#define INCLUDE_DFU_INTERFACE

#ifdef INCLUDE_DFU_INTERFACE
#include <unicore-mx/cm3/scb.h>
#include <unicore-mx/usb/class/dfu.h>
#endif

static usbd_device *usbd_dev;

static const uint8_t hid_report_descriptor[] = {
    0x05, 0x01,                    // USAGE_PAGE (Generic Desktop)
    0x15, 0x00,                    // LOGICAL_MINIMUM (0)
    0x09, 0x04,                    // USAGE (Joystick)
    0xa1, 0x01,                    // COLLECTION (Application)
    0x05, 0x02,                    //   USAGE_PAGE (Simulation Controls)
    0x09, 0xbb,                    //   USAGE (Throttle)
    0x15, 0x81,                    //   LOGICAL_MINIMUM (-127)
    0x25, 0x7f,                    //   LOGICAL_MAXIMUM (127)
    0x75, 0x08,                    //   REPORT_SIZE (8)
    0x95, 0x01,                    //   REPORT_COUNT (1)
    0x81, 0x02,                    //   INPUT (Data,Var,Abs)
    0x05, 0x01,                    //   USAGE_PAGE (Generic Desktop)
    0x09, 0x01,                    //   USAGE (Pointer)
    0xa1, 0x00,                    //   COLLECTION (Physical)
    0x09, 0x30,                    //     USAGE (X)
    0x09, 0x31,                    //     USAGE (Y)
    0x95, 0x02,                    //     REPORT_COUNT (2)
    0x81, 0x02,                    //     INPUT (Data,Var,Abs)
    0xc0,                          //   END_COLLECTION
    0x09, 0x39,                    //   USAGE (Hat switch)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x25, 0x03,                    //   LOGICAL_MAXIMUM (3)
    0x35, 0x00,                    //   PHYSICAL_MINIMUM (0)
    0x46, 0x0e, 0x01,              //   PHYSICAL_MAXIMUM (270)
    0x65, 0x14,                    //   UNIT (Eng Rot:Angular Pos)
    0x75, 0x04,                    //   REPORT_SIZE (4)
    0x95, 0x01,                    //   REPORT_COUNT (1)
    0x81, 0x02,                    //   INPUT (Data,Var,Abs)
    0x05, 0x09,                    //   USAGE_PAGE (Button)
    0x19, 0x01,                    //   USAGE_MINIMUM (Button 1)
    0x29, 0x04,                    //   USAGE_MAXIMUM (Button 4)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x25, 0x01,                    //   LOGICAL_MAXIMUM (1)
    0x75, 0x01,                    //   REPORT_SIZE (1)
    0x95, 0x04,                    //   REPORT_COUNT (4)
    0x55, 0x00,                    //   UNIT_EXPONENT (0)
    0x65, 0x00,                    //   UNIT (None)
    0x81, 0x02,                    //   INPUT (Data,Var,Abs)
    0xc0                           // END_COLLECTION
};

//byte 1: throttle
//byte 2: X
//byte 3: Y
//byte 4: First 4 bits: POV hat. Last 4 bits: buttons

static const struct {
	struct usb_hid_descriptor hid_descriptor;
	struct {
		uint8_t bReportDescriptorType;
		uint16_t wDescriptorLength;
	} __attribute__((packed)) hid_report;
} __attribute__((packed)) hid_function = {
	.hid_descriptor = {
		.bLength = sizeof(hid_function),
		.bDescriptorType = USB_DT_HID,
		.bcdHID = 0x0100,
		.bCountryCode = 0,
		.bNumDescriptors = 1,
	},
	.hid_report = {
		.bReportDescriptorType = USB_DT_REPORT,
		.wDescriptorLength = sizeof(hid_report_descriptor),
	}
};

const struct usb_endpoint_descriptor hid_endpoint = {
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = 0x81,
	.bmAttributes = USB_ENDPOINT_ATTR_INTERRUPT,
	.wMaxPacketSize = 4,
	.bInterval = 0x02,
};

const struct usb_interface_descriptor hid_iface = {
	.bLength = USB_DT_INTERFACE_SIZE,
	.bDescriptorType = USB_DT_INTERFACE,
	.bInterfaceNumber = 0,
	.bAlternateSetting = 0,
	.bNumEndpoints = 1,
	.bInterfaceClass = USB_CLASS_HID,
	.bInterfaceSubClass = 1, /* boot */
	.bInterfaceProtocol = 2, /* mouse */
	.iInterface = 0,

	.endpoint = &hid_endpoint,

	.extra = &hid_function,
	.extra_len = sizeof(hid_function),
};

#ifdef INCLUDE_DFU_INTERFACE
const struct usb_dfu_descriptor dfu_function = {
	.bLength = sizeof(struct usb_dfu_descriptor),
	.bDescriptorType = DFU_FUNCTIONAL,
	.bmAttributes = USB_DFU_CAN_DOWNLOAD | USB_DFU_WILL_DETACH,
	.wDetachTimeout = 255,
	.wTransferSize = 1024,
	.bcdDFUVersion = 0x011A,
};

const struct usb_interface_descriptor dfu_iface = {
	.bLength = USB_DT_INTERFACE_SIZE,
	.bDescriptorType = USB_DT_INTERFACE,
	.bInterfaceNumber = 1,
	.bAlternateSetting = 0,
	.bNumEndpoints = 0,
	.bInterfaceClass = 0xFE,
	.bInterfaceSubClass = 1,
	.bInterfaceProtocol = 1,
	.iInterface = 0,

	.extra = &dfu_function,
	.extra_len = sizeof(dfu_function),
};
#endif

const struct usb_interface ifaces[] = {{
	.num_altsetting = 1,
	.altsetting = &hid_iface,
#ifdef INCLUDE_DFU_INTERFACE
}, {
	.num_altsetting = 1,
	.altsetting = &dfu_iface,
#endif
}};

const struct usb_config_descriptor config[] = {{
	.bLength = USB_DT_CONFIGURATION_SIZE,
	.bDescriptorType = USB_DT_CONFIGURATION,
	.wTotalLength = 0,
#ifdef INCLUDE_DFU_INTERFACE
	.bNumInterfaces = 2,
#else
	.bNumInterfaces = 1,
#endif
	.bConfigurationValue = 1,
	.iConfiguration = 0,
	.bmAttributes = 0xC0,
	.bMaxPower = 0x32,

	.interface = ifaces,
}};

const struct usb_device_descriptor dev_descr = {
	.bLength = USB_DT_DEVICE_SIZE,
	.bDescriptorType = USB_DT_DEVICE,
	.bcdUSB = 0x0200,
	.bDeviceClass = 0,
	.bDeviceSubClass = 0,
	.bDeviceProtocol = 0,
	.bMaxPacketSize0 = 64,
	.idVendor = 0x0483,
	.idProduct = 0x5710,
	.bcdDevice = 0x0200,
	.iManufacturer = 1,
	.iProduct = 2,
	.iSerialNumber = 3,
	.bNumConfigurations = 1,

	.config = config
};

static const char *usb_strings_ascii[] = {
	"Black Sphere Technologies",
	"JOYSTICK",
	"DEMO",
};

/* Buffer used for control requests. */
uint8_t usbd_control_buffer[128];

static int usb_strings(usbd_device *dev, struct usbd_get_string_arg *arg)
{
	(void)dev;
	return usbd_handle_string_ascii(arg, usb_strings_ascii, 3);
}

static int hid_control_request(usbd_device *dev, struct usbd_control_arg *arg)
{
	(void)dev;

	if(	(arg->setup.bmRequestType != 0x81) ||
		(arg->setup.bRequest != USB_REQ_GET_DESCRIPTOR) ||
		(arg->setup.wValue != 0x2200)) {
		return USBD_REQ_NEXT;
	}

	/* Handle the HID report descriptor. */
	arg->buf = (uint8_t *)hid_report_descriptor;
	if (arg->len > sizeof(hid_report_descriptor)) {
		arg->len = sizeof(hid_report_descriptor);
	}

	return USBD_REQ_HANDLED;
}

#ifdef INCLUDE_DFU_INTERFACE

void __attribute__((weak))
usbhid_detach_complete_before_scb_reset_core(void) { /* empty */ }

static void dfu_detach_complete(usbd_device *dev, struct usbd_control_arg *arg)
{
	(void)arg;
	(void)dev;

	usbhid_detach_complete_before_scb_reset_core();
	scb_reset_core();
}

static enum usbd_control_result
dfu_control_request(usbd_device *dev, struct usbd_control_arg *arg)
{
	(void)dev;

	if ((arg->setup.bmRequestType != 0x21) ||
		(arg->setup.bRequest != DFU_DETACH)) {
		return USBD_REQ_NEXT; /* Only accept class request. */
	}

	arg->complete = dfu_detach_complete;

	return USBD_REQ_HANDLED;
}
#endif

static enum usbd_control_result
control_callback(usbd_device *dev, struct usbd_control_arg *arg)
{
	enum usbd_control_result result;

	result = hid_control_request(dev, arg);

#ifdef INCLUDE_DFU_INTERFACE
	if (result == USBD_REQ_NEXT) {
		result = dfu_control_request(dev, arg);
	}
#endif

	return result;
}

static void hid_set_config(usbd_device *dev,
				const struct usb_config_descriptor *cfg)
{
	(void)cfg;

	usbd_ep_setup(dev, 0x81, USB_ENDPOINT_ATTR_INTERRUPT, 4, NULL);

#if defined(__ARM_ARCH_6M__)
	systick_set_clocksource(STK_CSR_CLKSOURCE_AHB);
	/* SysTick interrupt every N clock pulses: set reload to N-1 */
	systick_set_reload(99999*8);
#else
	systick_set_clocksource(STK_CSR_CLKSOURCE_AHB_DIV8);
	/* SysTick interrupt every N clock pulses: set reload to N-1 */
	systick_set_reload(99999);
#endif
	systick_interrupt_enable();
	systick_counter_enable();
}

void __attribute__((weak))
usbhid_target_usbd_after_init_and_before_first_poll(void) { /* empty */ }

int main(void)
{
	usbhid_target_init();

	usbd_dev = usbd_init(usbhid_target_usb_driver(), &dev_descr,
		usbd_control_buffer, sizeof(usbd_control_buffer));

	usbd_register_set_config_callback(usbd_dev, hid_set_config);
	usbd_register_control_callback(usbd_dev, control_callback);
	usbd_register_get_string_callback(usbd_dev, usb_strings);

	usbhid_target_usbd_after_init_and_before_first_poll();

	while (1) {
		usbd_poll(usbd_dev);
	}
}

/* Fake Version */
void __attribute__((weak))
usbhid_target_accel_get(int16_t *out_x, int16_t *out_y, int16_t *out_z)
{

	static int x = 0;
	static int dir = 1;

	if (out_x != NULL) {
		*out_x = dir;
		*out_z = dir;
	}

	x += dir;
	if (x > 60) {
		dir = -dir;
		*out_y = 0xAA;
	}
	if (x < -60) {
		dir = -dir;
		*out_y = 0x00;
	}
	
	
}

void sys_tick_handler(void)
{
	int16_t x = 0, y = 0, z = 0;
	uint8_t buf[4];

	usbhid_target_accel_get(&x, &y, &z);

	buf[0] = x;
	buf[1] = y;
	buf[2] = z;
	buf[3] = 0b01011011;

	usbd_ep_write_packet(usbd_dev, 0x81, buf, sizeof(buf));
}
