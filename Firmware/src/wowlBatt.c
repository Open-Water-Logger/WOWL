/*
	wowlBatt.c

	Implementation of the Batt module for the WOWL.

	2020-09-22  WHF  Created.
*/

#include "wowl.h"

#include <nrf_gpio.h>

//*******************************  Module Data  ******************************//

//***********************  Local Function Declarations  **********************//

//****************************  Global Functions  ****************************//

void wowlBattInit(void)
{
	///// Configure Inputs  /////
	// Set the nCHRG pin to be an input with a pull-up:
	nrf_gpio_cfg_input(
			PIN_nCHRG,
			NRF_GPIO_PIN_PULLUP
	);
}

void wowlBattVisit(void)
{
	if (nrf_gpio_pin_read(PIN_nCHRG)) {
		// If pin high, charging is off.
		wowlMainClearStatus(WOWL_STATUS_CHARGING);
	} else {
		// We are charging.
		wowlMainClearStatus(WOWL_STATUS_CHARGING);
	}
}

//***********************  Local Function Definitions  ***********************//

