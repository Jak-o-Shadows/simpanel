/*
 * usb.c
 * 
 * Copyright 2017 jak <jak@linux-quu2>
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA 02110-1301, USA.
 * 
 * 
 */


#include "usb.h"


uint8_t usbd_control_buffer[128];

void (*fun_ptr)(void) = &usbCallback;



static const uint8_t hid_report_descriptor[] = {
    0x05, 0x01,                    // USAGE_PAGE (Generic Desktop)
    0x09, 0x04,                    // USAGE (Joystick)
    0xa1, 0x01,                    // COLLECTION (Application)
    0x05, 0x01,                    //   USAGE_PAGE (Generic Desktop)
    0xa1, 0x00,                    //   COLLECTION (Physical)
    0x09, 0x30,                    //     USAGE (X)
    0x09, 0x31,                    //     USAGE (Y)
    0x09, 0x32,                    //     USAGE (Z)
    0x09, 0x33,                    //     USAGE (Rx)
    0x09, 0x34,					   //	  USAGE (Ry)
    0x09, 0x35,					   //	  USAGE (Rz)
	0x09, 0x36,					   //	  USAGE (Slider)
	0x09, 0x36,					   //	  USAGE (Slider)
	0x09, 0x36,					   //	  USAGE (Slider)
	0x09, 0x36,					   //	  USAGE (Slider)
	0x09, 0x36,					   //	  USAGE (Slider)
	0x09, 0x36,					   //	  USAGE (Slider)
	0x09, 0x36,					   //	  USAGE (Slider)
	0x15, 0x00,        //     Logical Minimum (0)
	0x26, 0xFF, 0x00,  //     Logical Maximum (255)
	0x75, 0x08,        //     Report Size (8)
    0x95, 0x0D,                    //     REPORT_COUNT (13)					13 byte
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
    0x09, 0x39,                    //   USAGE (Hat switch)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x25, 0x07,                    //   LOGICAL_MAXIMUM (7)
    0x35, 0x00,                    //   PHYSICAL_MINIMUM (0)
    0x46, 0x0e, 0x01,              //   PHYSICAL_MAXIMUM (270)
    0x65, 0x14,                    //   UNIT (Eng Rot:Angular Pos)
    0x75, 0x04,                    //   REPORT_SIZE (4)
    0x95, 0x01,                    //   REPORT_COUNT (1)					1/2 byte
    0x81, 0x02,                    //   INPUT (Data,Var,Abs)
    0x05, 0x09,                    //   USAGE_PAGE (Button)   //add anothign lot of buttons 
    0x19, 0x01,                    //   USAGE_MINIMUM (Button 1)
    0x29, 0x28,                    //   USAGE_MAXIMUM (Button 40)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x25, 0x01,                    //   LOGICAL_MAXIMUM (1)
    0x75, 0x01,                    //   REPORT_SIZE (1)
    0x95, 0x28,                    //   REPORT_COUNT (40)					5 byte
    0x55, 0x00,                    //   UNIT_EXPONENT (0)
    0x65, 0x00,                    //   UNIT (None)
    0x81, 0x02,                    //   INPUT (Data,Var,Abs)
    0xc0                           // END_COLLECTION 
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
	.bInterfaceProtocol = 2, /* joystick */
	.iInterface = 0,

	.endpoint = &hid_endpoint,

	.extra = &hid_function,
	.extralen = sizeof(hid_function),
};


const struct usb_interface ifaces[] = {{
	.num_altsetting = 1,
	.altsetting = &hid_iface,
}};

const struct usb_config_descriptor config[] = {{
	.bLength = USB_DT_CONFIGURATION_SIZE,
	.bDescriptorType = USB_DT_CONFIGURATION,
	.wTotalLength = 0,
	.bNumInterfaces = 1,
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
};

static const char *usb_strings[] = {
	"Black Sphere Technologies",
	"JOYSTICK",
	"DEMO",
};
























static int hid_control_request(usbd_device *dev, struct usb_setup_data *req, uint8_t **buf, uint16_t *len,
			void (**complete)(usbd_device *dev, struct usb_setup_data *req))
{
	(void)complete;
	(void)dev;

	if((req->bmRequestType != 0x81) ||
	   (req->bRequest != USB_REQ_GET_DESCRIPTOR) ||
	   (req->wValue != 0x2200))
		return 0;

	/* Handle the HID report descriptor. */
	*buf = (uint8_t *)hid_report_descriptor;
	*len = sizeof(hid_report_descriptor);

	return 1;
}



static void hid_set_config(usbd_device *dev, uint16_t wValue)
{
	(void)wValue;

	//usbd_ep_setup(dev, 0x81, USB_ENDPOINT_ATTR_INTERRUPT, 4, NULL);
	usbd_ep_setup(usbd_dev, 0x81, USB_ENDPOINT_ATTR_BULK, 64, fun_ptr);  //64 = BULK_EP_MAXPACKET in examples = wMaxPacketSize
			
	usbd_register_control_callback(
				dev,
				USB_REQ_TYPE_STANDARD | USB_REQ_TYPE_INTERFACE,
				USB_REQ_TYPE_TYPE | USB_REQ_TYPE_RECIPIENT,
				hid_control_request);
				
	fun_ptr(); //prime callback by doing it once
	

}

void __attribute__((weak))
usbhid_target_usbd_after_init_and_before_first_poll(void) { /* empty */ }























//USB is all tucked away now - therefore things like usbd_dev are not in
//the namespace of the main file. Therefore need some helper functins.


void usbSetup(void){
	usbhid_target_init();

	usbd_dev = usbd_init(usbhid_target_usb_driver(), &dev_descr, config, usb_strings, 3,
		usbd_control_buffer, sizeof(usbd_control_buffer));

	usbd_register_set_config_callback(usbd_dev, hid_set_config);
	
	//Enable interrupt, so don't need to pull in main loop
	nvic_enable_irq(NVIC_USB_LP_CAN_RX0_IRQ);

	
}


void usbInLoop(void){
	usbd_poll(usbd_dev);
}


void writeToEndpoint(uint8_t endpoint, uint8_t buf[], uint16_t len){
	usbd_ep_write_packet(usbd_dev, endpoint, buf, len);
}


void usbCallback(void) {
	uint8_t buf[13 + 1 + 5];
	writeToEndpoint(0x81, buf, sizeof(buf)); 
}

void setUSBCallback(void (*fun)(void) ){
	fun_ptr = fun;
}


void usb_lp_can_rx0_isr(void){
	// Interrupt handler for low priority USB events.
	usbd_poll(usbd_dev);
}








