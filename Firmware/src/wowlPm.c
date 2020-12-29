/*
	wowlPm.c

	Implementation of the Power Management module for the WOWL.

	2020-09-22  WHF  Created.
*/

#include "wowl.h"

#include <nrf_clock.h>
#include <nrf_sdm.h>

//*******************************  Module Data  ******************************//

//***********************  Local Function Declarations  **********************//

//****************************  Global Functions  ****************************//

void wowlPmInit(void)
{
	// TODO
}

void wowlPmVisit(void)
{
	// Enter low power state based on state machine:
	static wowl_state_t prevState;
	
	const wowl_state_t state = wowlSmGetState();
	bool doSleep = true;
	
	if (state != prevState) {
		if (state == STATE_UNDER_SLEEP || state == STATE_LIMP) {
			// Entering the sleep state.

			uint8_t sdEn;
			sd_softdevice_is_enabled(&sdEn);

			if (sdEn) {			
				// Disable the DC-DC converter:
				(void) sd_power_dcdc_mode_set(NRF_POWER_DCDC_DISABLE);
				
				// Shutdown the soft-device:
				sd_softdevice_disable();
			} else {
				// Soft device already stopped.
			}
			
			// Stop the high frequency clock:
			nrf_clock_task_trigger(NRF_CLOCK_TASK_HFCLKSTOP);	
		} else if (prevState == STATE_UNDER_SLEEP || prevState == STATE_LIMP) {
			// When we awaken, restore the high frequency clock.
			nrf_clock_task_trigger(NRF_CLOCK_TASK_HFCLKSTART);
			if (state == STATE_SURFACE) {
				// We have returned to the surface from underwater.
				// Restore the soft device:
				//sd_softdevice_enable();
				wowlBleInit();
				// Restore the DC-DC converter.
				(void) sd_power_dcdc_mode_set(NRF_POWER_DCDC_ENABLE);
			} else {
				// Ignore other transitions.
				// TODO: could also get to the surface through LIMP.
			}
		} else {
			// Ignore other transitions.
		}
		// Cache:
		prevState = state;
	} else {
		// No state transition.
	}
	
	if (state == STATE_UNDER_WRITE) {
		// Needs to poll.  Do not sleep.
		doSleep = false;
	} else {
		// Sleep in other states.
	}
	
	if (wowlSmGetEvents() == 0u && doSleep) {
		// Idle the processor until the next event:
		wowlBleIdle();
	} else {
		// Do not sleep.
	}
}

//***********************  Local Function Definitions  ***********************//

#if 0
/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_enter(void)
{ 
	// Shutdown the environmental sensor:
	wowlBme280_Shutdown();
	
	// Go to system-off mode.  This function will not return unless the debugger
	//  is attached; instead, wakeup will cause a reset.
	const uint32_t err_code = sd_power_system_off();
	assert(err_code == NRF_ERROR_SOC_POWER_OFF_SHOULD_NOT_RETURN);
}
#endif

