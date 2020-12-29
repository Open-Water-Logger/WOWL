/*
	wowlSample.c

	Implementation of the Sample module for the WOWL.

	2020-09-22  WHF  Created.
*/

#include "wowl.h"
#include "wowlSample.h"

#include <crc.h>

#include <timer/app_timer.h>


//********************************  Constants  *******************************//
/////  Intervals  /////
// Time to wait for external ADCs to stabilize.
#define WAKEUP_TIME_MSEC                                                50u

// Interval between surface triggers (10 seconds).
#define SURFACE_TRIG_MSEC                              (10u * MSEC_PER_SEC)

// Interval between underwater triggers (once per minute).
#define UNDER_TRIG_MSEC                                (60u * MSEC_PER_SEC)

// Interval between limp mode checks (once every 30 seconds).
#define LIMP_TRIG_MSEC                                 (30u * MSEC_PER_SEC)

// Intervals in ticks.
#define SURFACE_TRIG_TICKS       MSEC_TO_APP_TIMER_TICKS(SURFACE_TRIG_MSEC)
#define UNDER_TRIG_TICKS           MSEC_TO_APP_TIMER_TICKS(UNDER_TRIG_MSEC)
#define LIMP_TRIG_TICKS             MSEC_TO_APP_TIMER_TICKS(LIMP_TRIG_MSEC)



/////  Thresholds  /////

// Pressure in atmospheres above which we are considered submersed.
#define PRESS_THRESH_UNDER_ATM                                       1.050f

// Pressure in atmospheres below which we are considered surfaced.
//  Note the hysteresis.
#define PRESS_THRESH_SURF_ATM                                        1.025f

// Conversion factor, from atmospheres to mbar.
#define ATM_TO_MBAR                                              1.01325e3f

// Thresholds in millibar.
#define PRESS_THRESH_UNDER_MBAR      (PRESS_THRESH_UNDER_ATM * ATM_TO_MBAR)
#define PRESS_THRESH_SURF_MBAR        (PRESS_THRESH_SURF_ATM * ATM_TO_MBAR)


// Critical battery voltage.  This is set by the minimum voltage of the ADS1120.
#define BATT_VOLT_THRESH_LIMP                                          2.3f

// Battery voltage to leave limp mode.
#define BATT_VOLT_THRESH_CHARGED                                       2.6f


/////  Sample Aggregation Flags  /////
// Bits that are set as each channel comes in.
#define READY_FLAG_NTC                                (1u << SI_TEMP_NTC_C)
#define READY_FLAG_RTD                                (1u << SI_TEMP_RTD_C)
#define READY_FLAG_SI                                  (1u << SI_TEMP_SI_C)
#define READY_FLAG_PRESS                              (1u << SI_PRESS_MBAR)
#define READY_FLAG_PRESS_T                          (1u << SI_PRESS_TEMP_C)
#define READY_FLAG_INT_NTC                        (1u << SI_TEMP_INT_NTC_C)
#define READY_FLAG_INT_RTD                        (1u << SI_TEMP_INT_RTD_C)

// Masks for each set of measurements.
#define TRIG1_READY   (READY_FLAG_NTC | READY_FLAG_RTD                    \
		| READY_FLAG_PRESS | READY_FLAG_PRESS_T)
#define TRIG2_READY (READY_FLAG_INT_NTC | READY_FLAG_INT_RTD | READY_FLAG_SI)


//*******************************  Module Data  ******************************//
static enum {
	SS_SLEEP,
	SS_WAKEUP,
	SS_TRIG1,
	SS_TRIG2,
} sampleState;

static uint32_t readyFlags;

static SAMPLE sample;
static bool 
	limpSample,
	wakeup;

// NRF macro to allocate space for a timer instance.
APP_TIMER_DEF(wakeupTimer);
APP_TIMER_DEF(triggerTimer);
APP_TIMER_DEF(limpTimer);

//***********************  Local Function Declarations  **********************//
static void checkSampleEvents(void);
static void finalizeSample(void);
static void limpTimerCallback(void * pUnused);
static void triggerAppTimerCallback(void *pUnused);
static void wakeupAppTimerCallback(void *pUnused);

//****************************  Global Functions  ****************************//
const SAMPLE* wowlSampleGet(void) { return &sample; }

void wowlSampleInit(void)
{
	// Initialize sub-modules:
	wowlExtAdcInit();
	wowlPressInit();
	wowlTempInit();
	
	// Create an application timer to generate the trigger for sampling:
	uint32_t err = app_timer_create(
			&triggerTimer,
			APP_TIMER_MODE_SINGLE_SHOT,
			triggerAppTimerCallback
	);
	assert(err == NRF_SUCCESS);

	// Create an application timer to help with wakeup timing.
	err = app_timer_create(
			&wakeupTimer,
			APP_TIMER_MODE_SINGLE_SHOT,
			wakeupAppTimerCallback
	);
	assert(err == NRF_SUCCESS);
	
	// Create a timer to sample the battery while in limp-mode.
	err = app_timer_create(
			&limpTimer,
			APP_TIMER_MODE_SINGLE_SHOT,
			limpTimerCallback
	);
	assert(err == NRF_SUCCESS);
}

// Submodules use to submit data for aggregation.
void wowlSampleSubmit(sample_index_t idx, float value)
{
	readyFlags |= 1u << idx;
	sample.values[idx] = value;
}

void wowlSampleVisit(void)
{
	static wowl_state_t prevState = STATE_INIT;
	const wowl_state_t state = wowlSmGetState();
	
	// Visit child modules:
	wowlExtAdcVisit();
	wowlPressVisit();
	wowlTempVisit();	
	
	if (state == STATE_UNDER_SAMPLE || state == STATE_SURFACE_SAMPLE) {
		if (prevState != state) {
			// Just started.  Wake up the external ADC.
			assert(sampleState == SS_SLEEP);
			wowlExtAdcWakeup();
			sampleState = SS_WAKEUP;
			wakeup = false;
			const uint32_t err = app_timer_start(
					wakeupTimer,
					// interval, in app timer ticks:
					APP_TIMER_FREQ_HZ * WAKEUP_TIME_MSEC / MSEC_PER_SEC, 
					NULL                               // callback context
			);			
			assert(err == NRF_SUCCESS);
			// Acquire internal ADC measurements.
			wowlAdcTrigger();
						
		} else {
			// We have been in the state.  Perform tasks as per the local
			//  state machine.
			switch (sampleState) {
				case SS_WAKEUP:
				if (wakeup) {
					// Start conversions.
					sampleState = SS_TRIG1;
					readyFlags = 0u;
					// Trigger the main set of external ADC values:
					wowlExtAdcTriggerMain();
					// Trigger the pressure sensor:
					wowlPressTrigger();
				} else {
					// Continue to wait.
				}
				break;
				
				case SS_TRIG1:
				if (readyFlags == TRIG1_READY) {
					sampleState = SS_TRIG2;
					readyFlags = 0u;
					// Trigger the second set of external ADC values:
					wowlExtAdcTriggerTempSense();
					// Trigger the temperature sensor.
					wowlTempTrigger();
				} else {
					// Continue to wait for readiness.
					// TODO timeout
				}
				break;
				
				case SS_TRIG2:
				if (readyFlags == TRIG2_READY) {
					// Sampling complete.  Finalize the sample.
					finalizeSample();
					// Return to sleep:					
					sampleState = SS_SLEEP;
					// Enter low power state:
					wowlExtAdcPowerDown();
					// Set event.
					wowlSmSetEvent(EVENT_SAMPLE_READY);
					if (state == STATE_SURFACE_SAMPLE) {
						// Update the Bluetooth data frame.
						wowlServiceTransmitResults(&sample);
					} else {
						// Do not.
					}
					checkSampleEvents();
				} else {
					// Continue to wait TODO timeout
				}
				break;
			}
		}
	} else if (state == STATE_SURFACE || state == STATE_UNDER_SLEEP) {	
		if (prevState != state) {
			// We've entered a sleepy state.  Set the trigger timer.
			const uint32_t err = app_timer_start(
					triggerTimer,
					// interval, in app timer ticks:
					state == STATE_SURFACE ? SURFACE_TRIG_TICKS 
							: UNDER_TRIG_TICKS,
					NULL                               // callback context
			);			
			assert(err == NRF_SUCCESS);
		} else {
			// Not a transition.  No action.
		}
	} else if (state == STATE_LIMP) {
		// TODO: minimize power consumption in this state
		if (prevState != state) {
			// Just entered limp mode.  Sample.
			limpSample = true;
		} else {
			// Already limping.
		}
		if (limpSample) {
			limpSample = false;
			
			// Sample the internal ADC.  This is synchronous.
			wowlAdcTrigger();
			
			if (sample.values[SI_BATTERY_VOLTAGE] > BATT_VOLT_THRESH_CHARGED) {
				// Leave the limping state.
				wowlSmSetEvent(EVENT_BATTERY_CHARGED);
			} else {
				// Start the next timer iteration.
				const uint32_t err = app_timer_start(
						limpTimer,
						// interval, in app timer ticks:
						LIMP_TRIG_TICKS,
						NULL                             // callback context
				);			
				assert(err == NRF_SUCCESS);
			}
		} else {
			// Wait for the timer.
		}
	} else {
		// No action in other states.
	}
	// Cache state.
	prevState = state;
}

//***********************  Local Function Definitions  ***********************//
static void checkSampleEvents(void)
{
	const float press_mbar = sample.values[SI_PRESS_MBAR];
	const wowl_state_t state = wowlSmGetState();
	
	if (state == STATE_SURFACE_SAMPLE && press_mbar > PRESS_THRESH_UNDER_MBAR) {
		// We have submerged.
		wowlSmSetEvent(EVENT_SUBMERGE);
	} else if (state == STATE_UNDER_SAMPLE 
			&& press_mbar < PRESS_THRESH_SURF_MBAR) {
		// We have surfaced.
		wowlSmSetEvent(EVENT_SURFACE);
	} else {
		// No pressure events.
	}
	
	// Check for low battery condition.
	if (sample.values[SI_BATTERY_VOLTAGE] < BATT_VOLT_THRESH_LIMP) {
		// The battery is critical.
		// Clear all events:
		wowlSmClearEvents(UINT32_MAX);		
		// Enter limp mode.
		wowlSmSetEvent(EVENT_BATTERY_LOW);
	} else {
		// Battery ok.
	}
}

static void finalizeSample(void)
{
	// Increment count:
	++sample.count;
	// Record status bits:
	sample.status = wowlMainGetStatus();
	// Compute the CRC of the sample, excluding the CRC field itself:
	sample.crc = crc32(
			(const uint8_t *) &sample,
			sizeof(SAMPLE) - sizeof(sample.crc)
	);
}

static void limpTimerCallback(void * pUnused)
{
	limpSample = true;	
}

static void triggerAppTimerCallback(void *pUnused)
{
	UNUSED(pUnused);
	
	wowlSmSetEvent(EVENT_TRIGGER);
}
	
static void wakeupAppTimerCallback(void *pUnused)
{
	UNUSED(pUnused);
	
	wakeup = true;
}


