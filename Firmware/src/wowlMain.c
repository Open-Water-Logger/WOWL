/*
	wowlMain.c
	
	Main loop for the WOWL.
	
	2020-09-22  WHF  Created.
*/

#include "wowl.h"

//********************************  Constants  *******************************//

//*******************************  Module Data  ******************************//
static uint16_t 
	status = 0u;
	
//***********************  Local Function Declarations  **********************//

//****************************  Global Functions  ****************************//

void main(void)
{
	// Initialize components, software and hardware:
	wowlInit();
	
	for (;;) {
		if (wowlSmVisit()) {
			// A state-transition occurred.
		} else {
			// No transition.
		}
				
		// Visit sub-modules.
		// Perform battery management tasks.		
		wowlBattVisit();	

		// Write samples to Flash as needed, or read-back to the user.
		wowlFlashVisit();
		
		// Update the LED state.
		wowlLedVisit();
		
		// Sample the attached sensors.
		wowlSampleVisit();	
		
		// Perform power management.
		wowlPmVisit();	
	}
}

uint16_t wowlMainGetStatus(void) { return status; }
void wowlMainClearStatus(uint16_t toClear)
{
	status &= ~toClear; 
}
void wowlMainSetStatus(uint16_t toSet) 
{
	status |= toSet;
}

