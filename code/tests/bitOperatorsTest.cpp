
#include <iostream>

#include "CppUTest/TestHarness.h"

#include "../midlevel/bitoperators.h"


TEST_GROUP(BitOperatorsTest)
{
};


TEST(BitOperatorsTest, all)
{
	uint8_t halfword[] = {0x00, 0x00};
	
	setBit(byte, 1);
	BITS_EQUAL(halfword[0], 0b1);
	BITS_EQUAL(halfword[1], 0);
	
	clearBit(byte, 1);
	BITS_EQUAL(halfword[0], 0);
	BITS_EQUAL(halfword[1], 0);
	
	FAIL();
	
} 