#ifndef USBSIDE
#define USBSIDE

#include <stdlib.h>
#include <stdbool.h>
#include <string.h>


#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/usb/usbd.h>
#include <libopencm3/usb/hid.h>
#include "usbhid-target.h"

/* Define this (in the Makefile CFLAGS) to include the DFU APP interface. */



static usbd_device *usbd_dev;

//static int hid_control_request(usbd_device *dev, struct usb_setup_data *req, uint8_t **buf, uint16_t *len,
//			void (**complete)(usbd_device *dev, struct usb_setup_data *req));

//static void hid_set_config(usbd_device *dev, uint16_t wValue);


void usbSetup(void);

void usbInLoop(void);

void writeToEndpoint(uint8_t endpoint, uint8_t buf[], uint16_t len);

void usbCallback(void);

void setUSBCallback(void (*fun)(void) );

void usb_lp_can_rx0_isr(void);











#endif
