// button processing
#include "buttonProc.h"


void debounceButtons(uint8_t buttons[], uint8_t numButtons, uint8_t buttonState[], uint8_t buttonIntegratorCount[]){
	// Debounce the given buttons
	//	Simply checks if the button state is consistent for a given number of counts
	//INPUTS:
	//	buttons
		//the array of uint8_t's. Each bit is the button state
	//	numButtons
		//number of buttons. The array "buttons" is 1/8th of this
	//	buttonState
		//Each bit is button state. is the output
	//	buttontIntegratorCount
	//		The count of the integrator for each button.
	//Inspired by http://www.kennethkuhn.com/electronics/debounce.c
	//	ie. that, but in a loop
	
	uint8_t MAXIMUM = 5;
	
	for (int i=0;i<numButtons;i++){
		bool input  = getBit(buttons, i);
		//step 1: Update the integrator based on the input signal. Note that the
		// integrator follows the input, decreasing or increasing towards the
		// limits as determined by the input state (0 or 1). 
		if (!input){
			if (buttonIntegratorCount[i] > 0){
				buttonIntegratorCount[i]--;
			}
		} else if (buttonIntegratorCount[i] < MAXIMUM) {
			buttonIntegratorCount[i]++;
		}
		
		//Step 2: Update the output state based on the integrator.  Note that
		// the output will only change states if the integrator has reached a
		// limit, either 0 or MAXIMUM.
		if (buttonIntegratorCount[i] == 0){
			clearBit(buttonState, i); //clear the button
		} else if (buttonIntegratorCount[i] >= MAXIMUM){
			buttonIntegratorCount[i] = MAXIMUM; //just in case it is >MAXIMUM by corruption
			setBit(buttonState, i); //set it
		}
				
		
		
	}

}