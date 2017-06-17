#include "analogMux.h"


//struct AnalogMuxConfig {
	//uint32_t enPort;
	//uint16_t enPin;
	//uint32_t s0Port;
	//uint16_t s0Pin;
	//uint32_t s1Port;
	//uint16_t s1Pin;
	//uint32_t s2Port;
	//uint16_t s2Pin;
	//uint32_t s3Port;
	//uint16_t s3Pin;
	//uint32_t sigPort;
	//uint32_t sigPin;
	//uint16_t con;
//};


void setupMux(struct AnalogMuxConfig config){
	//Pin configuration. Turns on clocks, does pin config.
	//NOTE: Whilst you can change what port/pin bank the ADC is on, this
	//	code assumes it is ADC1
	//	Also assumes that pin number = ADC channel number
	
	//turn on clocks
	//	Note: Does nothing if the clock is already turned on
	uint32_t clock;
	
	//Digital out pins
	clock = whichClock(config.s0Port);
	rcc_periph_clock_enable(clock);

	clock = whichClock(config.s1Port);
	rcc_periph_clock_enable(clock);
	
	clock = whichClock(config.s2Port);
	rcc_periph_clock_enable(clock);
	
	clock = whichClock(config.s3Port);
	rcc_periph_clock_enable(clock);
	
	clock = whichClock(config.enPort);
	rcc_periph_clock_enable(clock);
	
	//ADC signal pin
	clock = whichClock(config.sigPort);
	rcc_periph_clock_enable(clock);
	
	
	//Setup pins
	
	//Digital out pins
	gpio_set_mode(config.s0Port, GPIO_MODE_OUTPUT_50_MHZ, 
					GPIO_CNF_OUTPUT_PUSHPULL, config.s0Pin);
	gpio_set_mode(config.s1Port, GPIO_MODE_OUTPUT_50_MHZ, 
					GPIO_CNF_OUTPUT_PUSHPULL, config.s1Pin);
	gpio_set_mode(config.s2Port, GPIO_MODE_OUTPUT_50_MHZ, 
					GPIO_CNF_OUTPUT_PUSHPULL, config.s2Pin);
	gpio_set_mode(config.s3Port, GPIO_MODE_OUTPUT_50_MHZ, 
					GPIO_CNF_OUTPUT_PUSHPULL, config.s3Pin);
	
	gpio_set_mode(config.enPort, GPIO_MODE_OUTPUT_50_MHZ, 
					GPIO_CNF_OUTPUT_PUSHPULL, config.enPin);
	gpio_set(config.enPort, config.enPin); //turn it off, so it is in a known state. HC4067 is active LOW
					
	//ADC signal pin
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
	
}

void read(struct AnalogMuxConfig config, uint8_t retVals[]){
	//Read all the pins that are connected. 
	//	Note: only read connected ones for time. Does not change the
	//		position of the data in the retVals array that is output
	uint8_t channel_array[16]; //array of channels to read this time

	uint16_t joyRaw; //ADC is 12bit. Only are about 8 bit, because HID

	for (uint8_t i=0;i<16;i++){
		if (config.con >> i){
			//tell mux looking at these pins - ie. write i as a 4 bit
			//	binary number to the s0->s3 pins
			//because they are (potentially) on different banks, no real
			//	neat way of doing this.
			writePin(config.s0Port, config.s0Pin, ~(i^0b0001) & i);
			writePin(config.s1Port, config.s1Pin, ~(i^0b0010) & i);
			writePin(config.s2Port, config.s2Pin, ~(i^0b0100) & i);
			writePin(config.s3Port, config.s3Pin, ~(i^0b1000) & i);

			//Now have to wait a max of a few hundred nanoseconds
			for (int j=0;j<2000;j++){
				__asm__("nop");
			}
			
			//read ADC
			channel_array[0] = i; //want to read channel i
			adc_set_regular_sequence(ADC1, 1, channel_array); //tell ADC1 to read the channels in channel_array. Also tell it is there is one of those
			//start the ADC conversion directly (not trigger mode)
			adc_start_conversion_direct(ADC1);
			//Wait until it's finished
			while (!(ADC_SR(ADC1) & ADC_SR_EOC));
			//Hence read it
			joyRaw = ADC_DR(ADC1);
			retVals[i] = (uint8_t) ((joyRaw >> 4 ) & 0xFF); //truncate the 12 bit ADC to fit in a uint8
		}
	}
}

void enable(struct AnalogMuxConfig config){
	//Enable the analog mux by writing a 0 to its enable pin
	//	The HC4067 is active LOW	
	gpio_clear(config.enPort, config.enPin);
}

void disable(struct AnalogMuxConfig config){
	//Disable the analog mux by writing a 1 to its enable pin	
	//	The HC4067 is active LOW	
	gpio_set(config.enPort, config.enPin);
}





uint32_t whichClock(uint32_t bank){
	//What clock (RCC_GPIOx) does the bank GPIOx correspond to?	
	switch (bank){
		case GPIOA:
			return RCC_GPIOA;
		case GPIOB:
			return RCC_GPIOB;
		case GPIOC:
			return RCC_GPIOC;
		case GPIOD:
			return RCC_GPIOD;
		case GPIOE:
			return RCC_GPIOE;
	}
	return 0x0;
}


void writePin(uint32_t port, uint16_t pin, uint8_t value){
	//Write a 1 or 0 to a pin
	if (value){
		gpio_set(port, pin);
	} else{
		gpio_clear(port, pin);
	}
}



