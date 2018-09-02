
#include <iostream>

#include "CppUTest/TestHarness.h"
//#include "CppUTest/TestOutput.h"

#include "../midlevel/buttonProc.h"



#define NUMBUTTONS 16
uint8_t buttonsRaw[NUMBUTTONS/8];
uint8_t buttonsState[NUMBUTTONS/8];
uint8_t buttonIntegratorCount[NUMBUTTONS];
uint8_t numButtons = NUMBUTTONS;



TEST_GROUP(DebounceTest)
{
	void setup()
	{
		//	Called before each test
		//
		// Zero out button integrator, and buttons, and everything
		for (int i=0;i<NUMBUTTONS/8;i++){
			buttonsRaw[i] = 0x00;
			buttonsState[i] = 0x00;
		}
		for (int i=0;i<NUMBUTTONS;i++){
			buttonIntegratorCount[i] = 0x00;
		}
	}
};


TEST(DebounceTest, noInput)
{
	// Test that debounce does not erronousely return true if all zeros are on
	
	uint8_t expectedButtonState[NUMBUTTONS/8];
	for (int i=0;i<NUMBUTTONS/8;i++){
		expectedButtonState[i] = 0x00;
	}
	
	uint8_t expectedIntegratorCount[NUMBUTTONS];
	for (int i=0;i<NUMBUTTONS;i++){
		expectedIntegratorCount[i] = 0x00;
	}
	
	
	for (int i=0;i<50;i++){
		debounceButtons(buttonsRaw, numButtons, buttonsState, buttonIntegratorCount);
		for (int button_ind=0;button_ind<NUMBUTTONS/8;button_ind++){
			CHECK_EQUAL(expectedButtonState[button_ind], buttonsState[button_ind]);
		}
		for (int button_num=0;button_num<NUMBUTTONS;button_num++){
			CHECK_EQUAL(expectedIntegratorCount[button_num], buttonIntegratorCount[button_num]);
		}
	}
} 


TEST(DebounceTest, simple_incrementing)
{
	// Test whether it will count up to being triggered
	
	//Set the buttons to be high
	uint8_t buttonsRaw[NUMBUTTONS/8];
	for (int i=0;i<NUMBUTTONS;i++){
		buttonsRaw[i] = 0xFF;
	}
	// 3x high
	debounceButtons(buttonsRaw, numButtons, buttonsState, buttonIntegratorCount);
	debounceButtons(buttonsRaw, numButtons, buttonsState, buttonIntegratorCount);
	debounceButtons(buttonsRaw, numButtons, buttonsState, buttonIntegratorCount);
	// Then check status
	for (int button_ind=0;button_ind<NUMBUTTONS/8;button_ind++){
		CHECK_EQUAL(0, buttonsState[button_ind]);
	}
	// Then check integrator count
	for (int button_num=0;button_num<NUMBUTTONS;button_num++){
		CHECK_EQUAL(3, buttonIntegratorCount[button_num]);
	}
	
	// Add another 2x to show it should now be high
	debounceButtons(buttonsRaw, numButtons, buttonsState, buttonIntegratorCount);
	debounceButtons(buttonsRaw, numButtons, buttonsState, buttonIntegratorCount);
	// Then check status
	for (int button_ind=0;button_ind<NUMBUTTONS/8;button_ind++){
		CHECK_EQUAL(0xFF, buttonsState[button_ind]);
	}
	// Then check integrator count
	for (int button_num=0;button_num<NUMBUTTONS;button_num++){
		CHECK_EQUAL(5, buttonIntegratorCount[button_num]);
	}
	
	// Check now to see if it goes over
	//	Adding more high's should not change status further
	debounceButtons(buttonsRaw, numButtons, buttonsState, buttonIntegratorCount);
	debounceButtons(buttonsRaw, numButtons, buttonsState, buttonIntegratorCount);
	// Then check status
	for (int button_ind=0;button_ind<NUMBUTTONS/8;button_ind++){
		CHECK_EQUAL(0xFF, buttonsState[button_ind]);
	}
	// Then check integrator count
	for (int button_num=0;button_num<NUMBUTTONS;button_num++){
		CHECK_EQUAL(5, buttonIntegratorCount[button_num]);
	}
}






