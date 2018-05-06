// Bit operations
#include "bitoperators.h"


void setBit(uint8_t A[], uint8_t k) {
	// Set the kth bit in array A of uint8
	A[k/8] |= 1 << (k%8);
}

void clearBit(uint8_t A[], uint8_t k){
	// Clear the kth bit in array A of uint8
	A[k/8] &= ~(1 << (k%8));
}

bool getBit(uint8_t A[], uint8_t k){
	// Get the kth bit in array A of uint8
	return (bool) (A[k/8] & (1 << (k%8)) );
}