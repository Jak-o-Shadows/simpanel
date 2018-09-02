#ifndef BPROC
#define BPROC
#include <stdint.h>
#include <stdbool.h>

#include "bitoperators.h"

#ifdef __cplusplus
extern "C" {
#endif



void debounceButtons(uint8_t buttons[], uint8_t numButtons, uint8_t buttonState[], uint8_t buttonIntegratorCount[]);

#ifdef __cplusplus
}
#endif

#endif