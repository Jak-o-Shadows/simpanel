#ifndef BITOPS
#define BITOPS
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

void setBit(uint8_t A[], uint8_t k);

void clearBit(uint8_t A[], uint8_t k);

bool getBit(uint8_t A[], uint8_t k);

#ifdef __cplusplus
}
#endif

#endif