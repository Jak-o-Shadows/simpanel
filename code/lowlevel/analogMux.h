
#include <stdint.h>

#include <unicore-mx/stm32/gpio.h>
#include <unicore-mx/stm32/adc.h>
#include <unicore-mx/stm32/rcc.h>

struct AnalogMuxConfig {
	uint32_t enPort;
	uint16_t enPin;
	uint32_t s0Port;
	uint16_t s0Pin;
	uint32_t s1Port;
	uint16_t s1Pin;
	uint32_t s2Port;
	uint16_t s2Pin;
	uint32_t s3Port;
	uint16_t s3Pin;
	uint32_t sigPort;
	uint32_t sigPin;
	uint16_t con;
};


//external
void setupMux(struct AnalogMuxConfig config);

void readAll(struct AnalogMuxConfig config, uint8_t retVals[]);

uint8_t readOne(struct AnalogMuxConfig config, uint8_t pin);

void enable(struct AnalogMuxConfig config);

void disable(struct AnalogMuxConfig config);



//internal
uint32_t whichClock(uint32_t bank);
void writePin(uint32_t port, uint16_t pin, uint8_t value);
uint8_t adcChanLookup(uint32_t port, uint16_t sigPin);



