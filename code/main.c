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
#include <stdbool.h>
#include <unicore-mx/stm32/rcc.h>
#include <unicore-mx/stm32/gpio.h>
#include <unicore-mx/stm32/adc.h>



#include <unicore-mx/cm3/nvic.h>
#include <unicore-mx/cm3/systick.h>
#include <unicore-mx/usbd/usbd.h>
#include <unicore-mx/usb/class/hid.h>
#include <unicore-mx/usbd/misc/string.h>
#include "lowlevel/usbhid-target.h"
#include "lowlevel/usb.h"

#include "lowlevel/analogMux.h"


struct AnalogMuxConfig mux1;


uint8_t testValue = 0;

void nonUSBSetup(void);
void readAndPackButtons(uint8_t buttons[], uint8_t numButtons);
void pollSensors(uint8_t inputs[], uint8_t numInputs);
void testOutputs(uint8_t inputs[], uint8_t numInputs);

void nonUSBSetup(void){
	
	//Turn off JTAG - ie. allow PA15 (JTDI) & PB3 (JTDO) to be used as
	// GPIO
	#define AFIO_MAPR_SWJ_CFG_NO_JTAG_SW            (0x2 << 24)
	AFIO_MAPR |= AFIO_MAPR_SWJ_CFG_JTAG_OFF_SW_ON;//AFIO_MAPR_SWJ_CFG_NO_JTAG_SW;
	
	
	
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
	gpio_set(GPIOA, GPIO10 | GPIO9 | GPIO8); //enable pull UP resistors for bank A
	gpio_set(GPIOB, GPIO13 | GPIO12 | GPIO10 | GPIO11); //for bank B - the thumbsticks are active LOW
	gpio_set(GPIOC, GPIO13); //for bank C
	
	
	
	
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
	
	
	

	
	
	
	//SPI:
	//Using only simple SPI (MOSI & MISO only)
	//	PB14:	SPI2_MISO
	//	PB15:	SPI2_MOSI
	
	
	//Setup analog mux 1
	mux1.enPort = GPIOB; //it is just grounded in hardware - overwrite the i2c stuff  just now
	mux1.enPin = GPIO9;
	
	mux1.s0Port = GPIOB;
	mux1.s1Port = GPIOB;
	mux1.s2Port = GPIOB;
	mux1.s3Port = GPIOB;
	
	mux1.s0Pin = GPIO12;
	mux1.s1Pin = GPIO13;
	mux1.s2Pin = GPIO14;
	mux1.s3Pin = GPIO15;
	
	mux1.sigPort = GPIOA;
	mux1.sigPin = GPIO3;
	
	mux1.con = 0xFFFF; //want to look at all pins
	
	setupMux(mux1);
	enable(mux1);
	
	//I2C:
	//Two I2C interfaces available: Use I2C1
	//	PB8:	I2C1_SCL
	//	PB9:	I2C1_SDA
	
	
	
	
}


int main(void)
{
	
	nonUSBSetup();
	
	usbSetup();


	usbhid_target_usbd_after_init_and_before_first_poll();

	while (1) {
		usbInLoop();//usbd_poll(usbd_dev);
	}
}

void buttonReadTest(void){
	uint16_t aInput = gpio_port_read(GPIOA);
	uint16_t status = aInput & (GPIO10<<1);
	uint16_t testGPIOVal = GPIO10<<1;
}



void readAndPackButtons(uint8_t buttons[], uint8_t numButtons){
	//Contains the button mapping
	//Note: Does not correspond to the buttons in the HID descriptor.
	//Instead corresponds to the buttons in the output bytes (buttons[])
	//that are given in.
	//Starts counting buttons at 1
	
	
	for (int i=0;i<numButtons/8; i++){
		buttons[i] = 0x00;
	}
	
	//Store the button mapping in arrays to make it MUCH easier to 
	//	change, and to reduce code duplication
	//0xFF represents no switch connected
	//The buttons are what port
	int portMapping[40];
	//Buttons are active high?
	//The bits of levelMapping are what logic level an active button is
	uint32_t levelMapping = 0;
	levelMapping = 0xFFFFFFFF;
	//What pin is what button. Local to the port
	uint16_t pinMapping[40];
	
	//Button 0
	// NC
	portMapping[0] = GPIOC; //red PB
	pinMapping[0] = GPIO13;
	//Button 1
	// NC
	portMapping[1] = GPIOB; //red PB
	pinMapping[1] = GPIO5; 
	//Button 2
	// NC
	portMapping[2] = 0xFF; //analog mux 1 - red PB
	pinMapping[2] = 2; //
	//Button 3
	// NC
	portMapping[3] = GPIOB; //red PB
	pinMapping[3] = GPIO4; 
	//Button 4
	portMapping[4] = GPIOB; //red PB
	pinMapping[4] = GPIO3;
	//Button 5
	portMapping[5] = GPIOB; //red PB
	pinMapping[5] = GPIO6;
	//Button 6
	portMapping[6] = GPIOA; //red PB
	pinMapping[6] = GPIO3;
	//Button 7
	portMapping[7] = 0xFF; //analog mux 1, red PB
	pinMapping[7] = 0x01;
	//Button 8
	portMapping[8] = GPIOA; //red PB
	pinMapping[8] = GPIO10;
	//Button 9
	portMapping[9] = GPIOA; //red PB
	pinMapping[9] = GPIO11;
	//Button 10
	portMapping[10] = GPIOA; //red PB
	pinMapping[10] = GPIO12;
	//Button 11
	portMapping[11] = GPIOA; //red PB
	pinMapping[11] = GPIO15;
	//Button 12
	portMapping[12] = GPIOB; //red PB
	pinMapping[12] = GPIO6;
	//Button 13
	portMapping[13] = GPIOB; //red PB
	pinMapping[13] = GPIO7;
	//Button 14
	portMapping[14] = 0xFF; //analog mux 1, red PB
	pinMapping[14] = 0x00;
	//Button 15
	portMapping[15] = 0xFF; //none
	pinMapping[15] = 0xFF; //none
	//Button 16
	portMapping[16] = 0xFF; //analog mux1, toggle
	pinMapping[16] = 17;
	//Button 17
	portMapping[17] = 0xFF; //analog mux1, toggle
	pinMapping[17] = 18;
	//Button 18
	portMapping[18] = 0xFF;//analog mux1, toggle
	pinMapping[18] = 9;
	//Button 19
	portMapping[19] = 0xFF;//analog mux1, toggle
	pinMapping[19] = 12;
	//Button 20
	portMapping[20] = 0xFF;//analog mux1, toggle
	pinMapping[20] = 15;
	//Button 21
	portMapping[21] = 0xFF;//analog mux1, toggle
	pinMapping[21] = 8;
	//Button 22
	portMapping[22] = 0xFF;//analog mux1, toggle
	pinMapping[22] = 13;
	//Button 23
	portMapping[23] = 0xFF;//analog mux1, toggle
	pinMapping[23] = 14;
	
	uint16_t aInput = gpio_port_read(GPIOA);
	uint16_t bInput = gpio_port_read(GPIOB);
	uint16_t cInput = gpio_port_read(GPIOC);
	uint16_t GPIOInput;
	for (int i=0;i<24;i++){
		if (portMapping[i] != 0xFF){
			//Have a mapping here
			switch (portMapping[i]){
				case GPIOA:
					GPIOInput = aInput;
					break;
				case GPIOB:
					GPIOInput = bInput;
					break;
				case GPIOC:
					GPIOInput = cInput;
					break;
				default:
					GPIOInput = 0;
			}
					
			if (pinMapping[i] != 0xFF){
				//have a mapping here
				//check active high or active low
				if ((levelMapping & (1 << i)) != 0){
					//Active High
					if ((GPIOInput & (pinMapping[i]<<1)) != 0) {
						buttons[i/8] |= 1 << (i - 8*(i/8)); //FIX ME replace with  i % 8 ?
						}
				} else{
					//Active low
					if ((GPIOInput & (pinMapping[i]<<1)) == 0) {
						buttons[i/8] |= 1 << (i - 8*(i/8)); //FIX ME replace with  i % 8 ?
						}
				}
			} else{
				//Either have no mapping, or a mux
				if (pinMapping[i] != 0xFF) {
					// it's a mux!
					uint8_t buttonVal = 0x00;//readOne(mux1, pinMapping[i]);
					if (buttonVal >= 0x7F){
						//set the button
						buttons[i/8] |= 1 << (i - 8*(i/8)); //FIX ME replace with  i % 8 ?
					}
				} else{
					//no button mapping in this section
				}
			}

		}	
	}
}


void pollSensors(uint8_t inputs[], uint8_t numInputs){
	
	bool hatSwitch = true;
	uint8_t threshold = 50;
	uint8_t half = 127;
	
	uint8_t deadzone[13];
	//+- deadzone amount gets put to zero
	// [] limits
	deadzone[0] = 50;
	deadzone[1] = 50;
	deadzone[2] = 50;
	deadzone[3] = 50;
	deadzone[4] = 50;
	deadzone[5] = 50;
	deadzone[6] = 50;
	deadzone[7] = 50;
	deadzone[8] = 50;
	deadzone[9] = 50;
	deadzone[10] = 50;
	deadzone[11] = 50;
	deadzone[12] = 50;

	
	uint8_t correctionStart[13];
	for (int i=0;i<13;i++){
		correctionStart[i] = deadzone[i];
	}
	
	
	uint16_t joyRaw;
	uint8_t joy;
	//read buttons
	for (int i=0;i<numInputs;i++){
		inputs[i] = 0x00;
	}
	//readAndPackButtons(&inputs[13+1], numInputs-1-13);
	uint8_t *hat = &inputs[13];
	
	//return;
	//Read ADC1
	
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
	
	uint8_t channel_array[16]; //array of channels to read this time

	//Read thumb/joystick L
	//read ADC
	channel_array[0] = 0; //want to read channel 0 -(as it is PA0)
	adc_set_regular_sequence(ADC1, 1, channel_array); //tell ADC1 to read the channels in channel_array. Also tell it is there is one of those
	//start the ADC conversion directly (not trigger mode)
	adc_start_conversion_direct(ADC1);
	//Wait until it's finished
	while (!(ADC_SR(ADC1) & ADC_SR_EOC));
	//Hence read it
	joyRaw = ADC_DR(ADC1);
	joy = (uint8_t) ((joyRaw >> 4 ) & 0xFF); //truncate the 12 bit ADC to fit in a uint8
	inputs[0] = joy; 
	
	//read channel 1 - PA1
	channel_array[0] = 1; //want to read channel 1 -(as it is PA1)
	adc_set_regular_sequence(ADC1, 1, channel_array); //tell ADC1 to read the channels in channel_array. Also tell it is there is one of those
	//start the ADC conversion directly (not trigger mode)
	adc_start_conversion_direct(ADC1);
	//Wait until it's finished
	while (!(ADC_SR(ADC1) & ADC_SR_EOC));
	//Hence read it
	joyRaw = ADC_DR(ADC1);
	joy = (uint8_t) ((joyRaw >> 4 ) & 0xFF); //truncate the 12 bit ADC to fit in a uint8
	inputs[1] = joy;
	


	//read PSP thumbstick L
	//read channel 4 PA4
	channel_array[0] = 4; 
	adc_set_regular_sequence(ADC1, 1, channel_array); //tell ADC1 to read the channels in channel_array. Also tell it is there is one of those
	//start the ADC conversion directly (not trigger mode)
	adc_start_conversion_direct(ADC1);
	//Wait until it's finished
	while (!(ADC_SR(ADC1) & ADC_SR_EOC));
	//Hence read it
	joyRaw = ADC_DR(ADC1);
	joy = (uint8_t) ((joyRaw >> 4 ) & 0xFF); //truncate the 12 bit ADC to fit in a uint8
	inputs[2] = joy;
	//read channel 5 - PA5
	channel_array[0] = 5; 
	adc_set_regular_sequence(ADC1, 1, channel_array); //tell ADC1 to read the channels in channel_array. Also tell it is there is one of those
	//start the ADC conversion directly (not trigger mode)
	adc_start_conversion_direct(ADC1);
	//Wait until it's finished
	while (!(ADC_SR(ADC1) & ADC_SR_EOC));
	//Hence read it
	joyRaw = ADC_DR(ADC1);
	joy = (uint8_t) ((joyRaw >> 4 ) & 0xFF); //truncate the 12 bit ADC to fit in a uint8
	inputs[3] = joy;
	
	//Convert to buttons or hat switch
	uint8_t thumbstatus = 0x00;
	if (inputs[2] > threshold + half){
		//up
		thumbstatus |= 0b0001;
	} else if (inputs[2] < half -threshold){
		//down
		thumbstatus |= 0b0010;
	} 
	if (inputs[3] > half + threshold){
		//left
		thumbstatus |= 0b0100;
	} else if (inputs[3] < half -threshold){
		//right
		thumbstatus |= 0b1000;
	}
	
	//The 4 cardinal directions are provided as the 4 bits of thumbstatus
	//but hat switches number the directions (however many you tell it)
	//as numbers, going clockwise (?)
	//hence must convert from one to the other
	//mapping[cardinal direction/thumbstatus bits] = hat switch number
	//hat switch numbers start from 0
	uint8_t mapping[11];
	mapping[0] = 0x0F; //nothing
	mapping[1] = 0; //N
	mapping[2] = 4; //S
	mapping[3] = 0x0F; //nothing
	mapping[4] = 6; //W
	mapping[5] = 7; //NW
	mapping[6] = 5; //SW
	mapping[7] = 0x0F; //nothing
	mapping[8] = 2; //E
	mapping[9] = 1;//NE
	mapping[10] = 3;//SE
	
	if (hatSwitch){
		hat[0] |= mapping[thumbstatus];
	} else{
		//Double this thumbstick as 4 separate buttons
		//hat[0] = thumbstatus << 4;		
	}
	
	
	
	//Read thumb/joystick 2
	//read channel 8 - PB0
	channel_array[0] = 8; //want to read channel 8 -(as it is PB0)
	adc_set_regular_sequence(ADC1, 1, channel_array); //tell ADC1 to read the channels in channel_array. Also tell it is there is one of those
	//start the ADC conversion directly (not trigger mode)
	adc_start_conversion_direct(ADC1);
	//Wait until it's finished
	while (!(ADC_SR(ADC1) & ADC_SR_EOC));
	//Hence read it
	joyRaw = ADC_DR(ADC1);
	joy = (uint8_t) ((joyRaw >> 4 ) & 0xFF); //truncate the 12 bit ADC to fit in a uint8
	inputs[4] = joy;
	//read channel 9 - PB1
	channel_array[0] = 9; //want to read channel 9 -(as it is PB1)
	adc_set_regular_sequence(ADC1, 1, channel_array); //tell ADC1 to read the channels in channel_array. Also tell it is there is one of those
	//start the ADC conversion directly (not trigger mode)
	adc_start_conversion_direct(ADC1);
	//Wait until it's finished
	while (!(ADC_SR(ADC1) & ADC_SR_EOC));
	//Hence read it
	joyRaw = ADC_DR(ADC1);
	joy = (uint8_t) ((joyRaw >> 4 ) & 0xFF); //truncate the 12 bit ADC to fit in a uint8
	inputs[5] = joy;
	
	
	
	
	//read PSP thumbstick R
	//read channel 6 PA6
	channel_array[0] = 6; 
	adc_set_regular_sequence(ADC1, 1, channel_array); //tell ADC1 to read the channels in channel_array. Also tell it is there is one of those
	//start the ADC conversion directly (not trigger mode)
	adc_start_conversion_direct(ADC1);
	//Wait until it's finished
	while (!(ADC_SR(ADC1) & ADC_SR_EOC));
	//Hence read it
	joyRaw = ADC_DR(ADC1);
	joy = (uint8_t) ((joyRaw >> 4 ) & 0xFF); //truncate the 12 bit ADC to fit in a uint8
	inputs[6] = joy;
	//read channel 7 - PA7
	channel_array[0] = 7; 
	adc_set_regular_sequence(ADC1, 1, channel_array); //tell ADC1 to read the channels in channel_array. Also tell it is there is one of those
	//start the ADC conversion directly (not trigger mode)
	adc_start_conversion_direct(ADC1);
	//Wait until it's finished
	while (!(ADC_SR(ADC1) & ADC_SR_EOC));
	//Hence read it
	joyRaw = ADC_DR(ADC1);
	joy = (uint8_t) ((joyRaw >> 4 ) & 0xFF); //truncate the 12 bit ADC to fit in a uint8
	inputs[7] = joy;
	
	//Convert to buttons or hat switch
	thumbstatus = 0x00;
	if (inputs[6] > threshold + half){
		//up
		thumbstatus |= 0b0001;
	} else if (inputs[6] < half -threshold){
		//down
		thumbstatus |= 0b0010;
	} 
	if (inputs[7] > half + threshold){
		//left
		thumbstatus |= 0b0100;
	} else if (inputs[7] < half -threshold){
		//right
		thumbstatus |= 0b1000;
	}
	
	if (hatSwitch){
		hat[0] |= mapping[thumbstatus] <<4;
	} else{
		//Double this thumbstick as 4 separate buttons
		//hat[0] = thumbstatus << 8;		
	}

	

	
	
	//Read the analogue inputs that are via the analog multiplexer
	inputs[8] = readOne(mux1, 3);
	inputs[9] = readOne(mux1, 4);
	//inputs[10] = readOne(mux1, 5); //this pot is faulty
	inputs[11] = readOne(mux1, 6);
	inputs[12] = readOne(mux1, 7);
	
	//trim
	uint8_t trimOutMap[] = {0, 1, 4, 5};
	uint8_t trimInMap[] = {8, 9, 11, 12};
	
 	int testVal;
	for (int i=0;i<4;i++){
		testVal = inputs[trimOutMap[i]] + (inputs[trimInMap[i]]-0x7F)/2;
		//have to check against overflow because wrapping around is not desirable behaviour
		if (testVal > 0xFF){
			inputs[trimOutMap[i]] = 0xFF;
		} else if (testVal < 0) {
			inputs[trimOutMap[i]] = 0;
		} else {
			inputs[trimOutMap[i]] = (uint8_t) testVal;
		}
	} 
	
	
	//apply deadzones
	for (int i=0;i<13;i++){
		if ((inputs[i] <= (0xFF/2 + deadzone[i])) && (inputs[i] >= (0xFF/2 - deadzone[i]))){
			//apply the deadzone
			inputs[i] = 0xFF/2;
		} else {
			//apply deadzone correction
			if (inputs[i] > 0xFF/2) {
				// |-----|----x-----|
				// a     b          c
				// val = (x-b)*(c-a)/(c-b) + a
				inputs[i] = 0xFF/2 + (inputs[i] - (0xFF/2 + correctionStart[i])) * (0xFF-0xFF/2)/(0xFF - (0xFF/2+correctionStart[i]));
			} else {
				// |-----x-----|-----|
				// c           b     a
				// val = (x-b)*(c-a)/(c-b) + a
				inputs[i] = 0xFF/2 + (inputs[i] - (0xFF/2 - correctionStart[i])) * (0 - 0xFF/2)/(0 - (0xFF/2-correctionStart[i]));
			}
		}
	}
	
	
	
	
	
}

void testOutputs(uint8_t inputs[], uint8_t numInputs){
	//for (int i=0;i<numInputs;i++){
	//	inputs[i] = testValue;
	//}
	//inputs[0] = testValue;
	inputs[6] = testValue;
	inputs[7] = testValue;
	testValue++;
	if (testValue >255){ //not sure how a uint8_t overflows
		testValue = 0;
	}
}

void sys_tick_handler(void)
{
	uint8_t buf[13 + 1 + 5];
	for (int i=0;i<(13+1+5);i++){
		buf[i] = 0xFF;
	}
	//buttonReadTest();
	
	pollSensors(buf, 13+1+5);
	//testOutputs(buf, 13+1+5);
	buf[6] = buf[0];
	buf[7] = buf[1];

	writeToEndpoint(0x81, buf, sizeof(buf));
}
