
#include <iostream>

#include "CppUTest/TestHarness.h"

#include "../midlevel/bitoperators.h"


TEST_GROUP(BitOperatorsTest)
{
};


TEST(BitOperatorsTest, all)
{
	uint8_t halfword[] = {0x00, 0x00};
	
	// Test setting bit
	setBit(halfword, 1);
	BYTES_EQUAL(0b10, halfword[0]);
	BYTES_EQUAL(1, getBit(halfword, 1));
	BYTES_EQUAL(0, halfword[1]);
	
	clearBit(halfword, 1);
	BYTES_EQUAL(0, halfword[0]);
	BYTES_EQUAL(0, getBit(halfword, 1));
	BYTES_EQUAL(0, halfword[1]);
	
	// Test just before changing over byte
	setBit(halfword, 7);
	BYTES_EQUAL(0b10000000, halfword[0]);
	BYTES_EQUAL(1, getBit(halfword, 7));
	BYTES_EQUAL(0, halfword[1]);
	
	clearBit(halfword, 7);
	BYTES_EQUAL(0, halfword[0]);
	BYTES_EQUAL(0, getBit(halfword, 7));
	BYTES_EQUAL(0, halfword[1]);
	
	// Test after changing over byte
	setBit(halfword, 13);
	BYTES_EQUAL(0, halfword[0]);
	BYTES_EQUAL(1, getBit(halfword, 13));
	BYTES_EQUAL(0b100000, halfword[1]);
	
	clearBit(halfword, 13);
	BYTES_EQUAL(0, halfword[0]);
	BYTES_EQUAL(0, getBit(halfword, 0));
	BYTES_EQUAL(0, halfword[1]);
	
	// Test near the end of the second byte
	setBit(halfword, 15);
	BYTES_EQUAL(0, halfword[0]);
	BYTES_EQUAL(1, getBit(halfword, 15));
	BYTES_EQUAL(0b10000000, halfword[1]);
	
	clearBit(halfword, 15);
	BYTES_EQUAL(0, halfword[0]);
	BYTES_EQUAL(0, getBit(halfword, 15));
	BYTES_EQUAL(0, halfword[1]);
	
} 