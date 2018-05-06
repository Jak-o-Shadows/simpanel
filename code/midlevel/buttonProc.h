#ifndef BPROC
#define BPROC
#include <stdint.h>
#include <stdbool.h>

#include "bitoperators.h"


void debounceButtons(uint8_t buttons[], uint8_t numButtons, uint8_t buttonState[], uint8_t buttonIntegratorCount[]);

#endif