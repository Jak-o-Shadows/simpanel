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
#include <unicore-mx/stm32/rcc.h>
#include <unicore-mx/stm32/gpio.h>
#include <unicore-mx/stm32/adc.h>



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

uint8_t status = 0;
uint8_t joy = 0x00;
uint8_t joy2 = 0x00;

static const uint8_t hid_report_descriptor[] = {
/*     0x05, 0x01,                    // USAGE_PAGE (Generic Desktop)
    0x15, 0x00,                    // LOGICAL_MINIMUM (0)
    0x09, 0x04,                    // USAGE (Joystick)
    0xa1, 0x01,                    // COLLECTION (Application)
    0x05, 0x02,                    //   USAGE_PAGE (Simulation Controls)
    0x09, 0xbb,                    //   USAGE (Throttle)
    0x15, 0x81,                    //   LOGICAL_MINIMUM (-127)
    0x25, 0x7f,                    //   LOGICAL_MAXIMUM (127)
    0x75, 0x08,                    //   REPORT_SIZE (8)
    0x81, 0x02,                    //   INPUT (Data,Var,Abs)
    0x95, 0x01,                    //   REPORT_COUNT (1)
    0x05, 0x01,                    //   USAGE_PAGE (Generic Desktop)
    0x09, 0x01,                    //   USAGE (Pointer)
    0xa1, 0x00,                    //   COLLECTION (Physical)
    0x09, 0x30,                    //     USAGE (X)								
    0x09, 0x31,                    //     USAGE (Y)								
    //0x09, 0x32,                    //     USAGE (Z)								
    //0x09, 0x35,                    //     USAGE (Rz)							
    //0x09, 0x34,                    //     USAGE (Ry)							
    //0x09, 0x36,                    //     USAGE (Slider)						
    //0x09, 0x37,                    //     USAGE (Dial)							
    //0x09, 0x38,                    //     USAGE (Wheel)							
    //0x09, 0x40,                    //     USAGE (Vx)							
    //0x09, 0x41,                    //     USAGE (Vy)							
    //0x95, 0x0a,                    //     REPORT_COUNT (10)
	0x95, 0x02,						//		REPORT_COUNT (2)
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
    0x75, 0x04,                    //   REPORT_SIZE (4)
    0x95, 0x01,                    //   REPORT_COUNT (1)
    0x81, 0x03,                    //   INPUT (Cnst,Var,Abs)					
    0x05, 0x09,                    //   USAGE_PAGE (Button)
    0x19, 0x01,                    //   USAGE_MINIMUM (Button 1)
    0x29, 0x10,                    //   USAGE_MAXIMUM (Button 16)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x25, 0x01,                    //   LOGICAL_MAXIMUM (1)
    0x75, 0x01,                    //   REPORT_SIZE (1)
    0x95, 0x10,                    //   REPORT_COUNT (16)
    0x55, 0x00,                    //   UNIT_EXPONENT (0)
    0x65, 0x00,                    //   UNIT (None)
    0x81, 0x02,                    //   INPUT (Data,Var,Abs)					
    0xc0                           // END_COLLECTION
}; */
//byte 1: X
//byte 2: Y
//byte 3: Z
//byte 4: Rz
//byte 5: Ry
//byte 6: Slider
//byte 7: Dial
//byte 8: Wheel
//byte 9: Vx
//byte 10: Vy
//byte 11: First 4 bits: POV hat. Last 4 bits: nothing
//byte 11->(11+7)		11->18:	buttons. Buttons
    0x05, 0x01,                    // USAGE_PAGE (Generic Desktop)
    0x15, 0x00,                    // LOGICAL_MINIMUM (0)
    0x09, 0x04,                    // USAGE (Joystick)
    0xa1, 0x01,                    // COLLECTION (Application)
    0x05, 0x02,                    //   USAGE_PAGE (Simulation Controls)
    0x09, 0xbb,                    //   USAGE (Throttle)
    0x15, 0x81,                    //   LOGICAL_MINIMUM (-127)
    0x25, 0x7f,                    //   LOGICAL_MAXIMUM (127)
    0x75, 0x08,                    //   REPORT_SIZE (8)
    0x95, 0x01,                    //   REPORT_COUNT (1)					1 byte
    0x81, 0x02,                    //   INPUT (Data,Var,Abs)
    0x05, 0x01,                    //   USAGE_PAGE (Generic Desktop)
    0x09, 0x01,                    //   USAGE (Pointer)
    0xa1, 0x00,                    //   COLLECTION (Physical)
    0x09, 0x30,                    //     USAGE (X)
    0x09, 0x31,                    //     USAGE (Y)
    0x95, 0x02,                    //     REPORT_COUNT (2)					2 byte
    0x81, 0x02,                    //     INPUT (Data,Var,Abs)
    0xc0,                          //   END_COLLECTION
    0x09, 0x39,                    //   USAGE (Hat switch)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x25, 0x03,                    //   LOGICAL_MAXIMUM (3)
    0x35, 0x00,                    //   PHYSICAL_MINIMUM (0)
    0x46, 0x0e, 0x01,              //   PHYSICAL_MAXIMUM (270)
    0x65, 0x14,                    //   UNIT (Eng Rot:Angular Pos)
    0x75, 0x04,                    //   REPORT_SIZE (4)
    0x95, 0x01,                    //   REPORT_COUNT (1)					1/2 byte
    0x81, 0x02,                    //   INPUT (Data,Var,Abs)
    0x05, 0x09,                    //   USAGE_PAGE (Button)
    0x19, 0x01,                    //   USAGE_MINIMUM (Button 1)
    0x29, 0x0B,                    //   USAGE_MAXIMUM (Button 12)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x25, 0x01,                    //   LOGICAL_MAXIMUM (1)
    0x75, 0x01,                    //   REPORT_SIZE (1)
    0x95, 0x0B,                    //   REPORT_COUNT (12)					3/2 byte
    0x55, 0x00,                    //   UNIT_EXPONENT (0)
    0x65, 0x00,                    //   UNIT (None)
    0x81, 0x02,                    //   INPUT (Data,Var,Abs)
    0xc0                           // END_COLLECTION

/*     0x05, 0x01,                    // USAGE_PAGE (Generic Desktop)
    0x15, 0x00,                    // LOGICAL_MINIMUM (0)
    0x09, 0x04,                    // USAGE (Joystick)
    0xa1, 0x01,                    // COLLECTION (Application)
    0x05, 0x02,                    //   USAGE_PAGE (Simulation Controls)
    0x09, 0xbb,                    //   USAGE (Throttle)
    0x15, 0x81,                    //   LOGICAL_MINIMUM (-127)
    0x25, 0x7f,                    //   LOGICAL_MAXIMUM (127)
    0x75, 0x08,                    //   REPORT_SIZE (8)
    0x95, 0x01,                    //   REPORT_COUNT (1)					1 byte
    0x81, 0x02,                    //   INPUT (Data,Var,Abs)
    0x05, 0x01,                    //   USAGE_PAGE (Generic Desktop)
    0x09, 0x01,                    //   USAGE (Pointer)
    0xa1, 0x00,                    //   COLLECTION (Physical)
    0x09, 0x30,                    //     USAGE (X)
    0x09, 0x31,                    //     USAGE (Y)
    0x95, 0x02,                    //     REPORT_COUNT (2)					2 byte
    0x81, 0x02,                    //     INPUT (Data,Var,Abs)
    0xc0,                          //   END_COLLECTION
    0x09, 0x39,                    //   USAGE (Hat switch)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x25, 0x07,                    //   LOGICAL_MAXIMUM (7)
    0x35, 0x00,                    //   PHYSICAL_MINIMUM (0)
    0x46, 0x0e, 0x01,              //   PHYSICAL_MAXIMUM (270)
    0x65, 0x14,                    //   UNIT (Eng Rot:Angular Pos)
    0x75, 0x04,                    //   REPORT_SIZE (4)
    0x95, 0x01,                    //   REPORT_COUNT (1)					1/2 byte
    0x81, 0x02,                    //   INPUT (Data,Var,Abs)
    0x05, 0x09,                    //   USAGE_PAGE (Button)
    0x19, 0x01,                    //   USAGE_MINIMUM (Button 1)
    0x29, 0x04,                    //   USAGE_MAXIMUM (Button 4)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x25, 0x01,                    //   LOGICAL_MAXIMUM (1)
    0x75, 0x01,                    //   REPORT_SIZE (1)
    0x95, 0x04,                    //   REPORT_COUNT (4)					1/2 byte
    0x55, 0x00,                    //   UNIT_EXPONENT (0)
    0x65, 0x00,                    //   UNIT (None)
    0x81, 0x02,                    //   INPUT (Data,Var,Abs)
//add anothign lot of buttons
    0x05, 0x09,                    //   USAGE_PAGE (Button)
    0x19, 0x05,                    //   USAGE_MINIMUM (Button 5)
    0x29, 0x0C,                    //   USAGE_MAXIMUM (Button 12)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x25, 0x01,                    //   LOGICAL_MAXIMUM (1)
    0x75, 0x01,                    //   REPORT_SIZE (1)
    0x95, 0x08,                    //   REPORT_COUNT (8)					1 byte
    0x55, 0x00,                    //   UNIT_EXPONENT (0)
    0x65, 0x00,                    //   UNIT (None)
    0x81, 0x02,                    //   INPUT (Data,Var,Abs)
    0xc0                           // END_COLLECTION */
};



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
	.wMaxPacketSize = 64,   //WAS 4 - CHANGED TO TEST SOMETHING ON THE 2-12-2016 BY J
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


void nonUSBSetup(void){
	
	//Button Inputs:
	//Available button inputs for the STM32F103C8T6 Blue pill board are:
	//	PC13
	//	PB10
	//	PB11
	//	PB7
	//	PB6
	//	PB5
	//	PB4
	//	PB3
	//	PA15
	//	PA10
	//	PA9
	//	PA8
	//	PB13
	//	PB12
	//Must setup banks of pins at a time
	//		Bank A:
	rcc_periph_clock_enable(RCC_GPIOA);
	uint16_t bankPins = GPIO15 | GPIO10 | GPIO9 | GPIO8;
	gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, bankPins);
	gpio_clear(GPIOA, bankPins); // turn to pull-down resistors
	//		Bank B
	rcc_periph_clock_enable(RCC_GPIOB);
	bankPins = GPIO10 | GPIO11 | GPIO7 | GPIO6 | GPIO5 | GPIO4 | GPIO3 | GPIO13 | GPIO12;
	gpio_set_mode(GPIOB, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, bankPins);
	gpio_clear(GPIOB, bankPins); //turn to pull-down resistors
	//		Bank C
	rcc_periph_clock_enable(RCC_GPIOC);
	bankPins = GPIO13;
	gpio_set_mode(GPIOC, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, bankPins);
	gpio_clear(GPIOC, bankPins);// pull-down resistors
	//Now set pull-up (button active low) and pull-down (button active high) internal resistors
	//	Note: Defaults to pull-down resistors
	//gpio_set(GPIOA, ?????); //enable pull UP resistors for bank A
	gpio_set(GPIOB, GPIO10 | GPIO11); //for bank B - the thumbsticks are active LOW
	//gpio_set(GPIOC, GPIO13); //for bank C
	
	
	
	
	//An ADC Inputs
	//Available for the STM32F103C8T6 Blue Pill board:
	//	PA0:	ADC1, CH0
	//	PA1:	ADC1, CH1
	//	PA2:	ADC1, CH2
	//	PA3:	ADC1, CH3
	//	PA4:	ADC1, CH4
	//	PA5:	ADC1, CH5
	//	PA6:	ADC1, CH6
	//	PA7:	ADC1, CH7
	//	PB0:	ADC1, CH8
	//	PB1:	ADC1, CH9
	rcc_periph_clock_enable(RCC_ADC1);
	adc_power_off(ADC1); //turn ADC off during config
	//configure ADC1 for single conversion
	adc_disable_scan_mode(ADC1);
	adc_set_single_conversion_mode(ADC1);
	adc_disable_external_trigger_regular(ADC1);
	adc_set_right_aligned(ADC1);
	//Set sample time?
	adc_set_sample_time_on_all_channels(ADC1, ADC_SMPR_SMP_28DOT5CYC);
	//turn it on
	adc_power_on(ADC1);
	//wait for it to turn on
	for (int i=0;i<800000;i++){
		__asm__("nop");
	}
	adc_reset_calibration(ADC1);
	adc_calibration(ADC1);
	
	
	
	//I2C:
	//Two I2C interfaces available: Use I2C1
	//	PB8:	I2C1_SCL
	//	PB9:	I2C1_SDA
	
	
	
	//SPI:
	//Using only simple SPI (MOSI & MISO only)
	//	PB14:	SPI2_MISO
	//	PB15:	SPI2_MOSI
	
}



int main(void)
{
	
	nonUSBSetup();
	
	
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


void readAndPackButtons(uint8_t buttons[], uint8_t numButtons){
	//Contains the button mapping

	uint16_t GPIOInput;
	
	for (int i=0;i<numButtons/8; i++){
		buttons[i] = 0x00;
	}
	
	//Bank A:
	//PA15, PA10, PA9, PA8
	//Mapping:
	//	PA15	Toggle Switch	Active HIGH		Button 3
	//	PA10	NOT USED
	//	PA9		NOT USED
	//	PA8		NOT USED
	GPIOInput = gpio_port_read(GPIOA);
	if ((GPIOInput & GPIO15) == GPIO15) {buttons[3/8] |= 1 << 0x03;}
	
	//BANK B:
	//PB10, PB11, PB7, PB5, PB4, PB3, PB13, PB12
	//Mapping:
	//	PB10	Left Thumbstick Button		Active LOW		Button 1
	//	PB11	Right Thumbstick Button		Active LOW		Button 2
	//	PB7		Toggle Switch	Active HIGH		Button 4
	//	PB6		Toggle Switch	Active HIGH		Button 5
	//	PB5		Toggle Switch	Active HIGH		Button 6
	//	PB4		Toggle Switch	Active HIGH		Button 7
	//	PB3		NOT USED
	//	PB13	NOT USED
	//	PB12	NOT USED
	GPIOInput = gpio_port_read(GPIOB);
	if ((GPIOInput & GPIO10) == GPIO10) {buttons[1/8] |= 1 << 0x01;}
	if ((GPIOInput & GPIO11) == GPIO11) {buttons[2/8] |= 1 << 0x02;}
	if ((GPIOInput & GPIO7) == GPIO7) {buttons[4/8] |= 1 << 0x04;}
	if ((GPIOInput & GPIO6) == GPIO6) {buttons[5/8] |= 1 << 0x05;}
	if ((GPIOInput & GPIO5) == GPIO5) {buttons[6/8] |= 1 << 0x06;}
	if ((GPIOInput & GPIO4) == GPIO4) {buttons[7/8] |= 1 << 0x07;}////////////////////
	
	//Bank C:
	//PC13
	//Mapping:
	//	PC13	NOT USED
	GPIOInput = gpio_port_read(GPIOC);
	
}


void pollSensors(uint8_t inputs[], uint8_t numInputs){
	uint16_t joyRaw;
	//read buttons
	for (int i=0;i<11;i++){
		inputs[i] = 0x00;
	}
	readAndPackButtons(&inputs[11], 16);
	
	//read ADC
	uint8_t channel_array[16]; //array of channels to read this time
	channel_array[0] = 7; //want to read channel 7 -(as it is PA7)
	adc_set_regular_sequence(ADC1, 1, channel_array); //tell ADC1 to read the channels in channel_array. Also tell it is there is one of those
	//start the ADC conversion directly (not trigger mode)
	adc_start_conversion_direct(ADC1);
	//Wait until it's finished
	while (!(ADC_SR(ADC1) & ADC_SR_EOC));
	//Hence read it
	joyRaw = ADC_DR(ADC1);
	joy = (uint8_t) ((joyRaw >> 4 ) & 0xFF); //truncate the 12 bit ADC to fit in a uint8
	//Now, the reading goes from 0->255. With a joystick value of -127 -> 127.
	//Windows interprets:
	//		0->127 as 0->127
	//		128->255 as -127 -> -1 (or 0)
	//There is obviously a discontinuity there -> hence remap and remove it
	if (joy <= 127){
		joy = joy + 127;
	} else{
		joy = joy - 127;
	}
	inputs[0] = joy;
	
	//read channel 6 - PA6
	channel_array[0] = 6; //want to read channel 6 -(as it is PA6)
	adc_set_regular_sequence(ADC1, 1, channel_array); //tell ADC1 to read the channels in channel_array. Also tell it is there is one of those
	//start the ADC conversion directly (not trigger mode)
	adc_start_conversion_direct(ADC1);
	//Wait until it's finished
	while (!(ADC_SR(ADC1) & ADC_SR_EOC));
	//Hence read it
	joyRaw = ADC_DR(ADC1);
	joy2 = (uint8_t) ((joyRaw >> 4 ) & 0xFF); //truncate the 12 bit ADC to fit in a uint8
	//Now, the reading goes from 0->255. With a joystick value of -127 -> 127.
	//Windows interprets:
	//		0->127 as 0->127
	//		128->255 as -127 -> -1 (or 0)
	//There is obviously a discontinuity there -> hence remap and remove it
	if (joy2 <= 127){
		joy2 = joy2 + 127;
	} else{
		joy2 = joy2 - 127;
	}
	inputs[1] = joy2;
	
	
}


void sys_tick_handler(void)
{
	int16_t x = 0, y = 0, z = 0;
	uint8_t buf[5];
	
	//pollSensors(buf, 13);

	usbhid_target_accel_get(&x, &y, &z);
	joy++;
	if (joy >= 255){
		joy = 0;
	}

	buf[0] = joy;
	buf[1] = joy;
	buf[2] = joy;
	buf[3] = joy;
	buf[4] = joy;

	usbd_ep_write_packet(usbd_dev, 0x81, buf, sizeof(buf));
}
