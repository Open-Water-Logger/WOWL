/*
	wowlSample.c

	Implementation of the Sample module for the WOWL.

	2020-09-22  WHF  Created.
*/

#include "wowl.h"
#include "wowlSample.h"

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
#define READY_FLAG_TI                                  (1u << SI_TEMP_TI_C)
#define READY_FLAG_PRESS                              (1u << SI_PRESS_MBAR)

// Masks for each set of measurements.
#define TRIG1_READY                                      (READY_FLAG_PRESS)
#define TRIG2_READY                                         (READY_FLAG_TI)


static const float MIN_SENSOR[] = {
	1.5f,                                 // SI_BATTERY_VOLTAGE,  
	-5.0f,                                // SI_TEMP_TI_C,
	0.0f,                                 // SI_PRESS_MBAR,
}, MAX_SENSOR[] = {                       
	3.05f,                                // SI_BATTERY_VOLTAGE,
	35.95f,                               // SI_TEMP_TI_C,
	32767.0f,                             // SI_PRESS_MBAR,
};                                        
static_assert(_countof(MIN_SENSOR) == _countof(MAX_SENSOR), "Inconsistent");
static_assert(_countof(MIN_SENSOR) == N_SI, "Inconsistent");


//*********************************  Macros  *********************************//
// Binds a variable between a low and high point:
#define BIND(x, lo, hi) ((x) = (x) < (lo) ? (lo) : ((x) > (hi) ? (hi) : (x)))



//*******************************  Module Data  ******************************//
/**
  * Uncompressed sensor values.
  */
static SURFACE_SAMPLE surfaceSample;

// TODO: are there optimizations here?
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
static void gatherSurfaceSample(void);
static void limpTimerCallback(void * pUnused);
static void triggerAppTimerCallback(void *pUnused);
static void wakeupAppTimerCallback(void *pUnused);

//****************************  Global Functions  ****************************//
const SAMPLE* wowlSampleGet(void) { return &sample; }
const SURFACE_SAMPLE* wowlSampleGetSurface(void) { return &surfaceSample; }

void wowlSampleInit(void)
{
	// Initialize sub-modules:
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
void wowlSampleSubmitSubmerged(submerged_sample_index_t idx, float value)
{
	readyFlags |= 1u << idx;
	// Store in the raw sample buffer:
	surfaceSample.submerged[idx] = value;
}
void wowlSampleSubmitSurface(surface_sample_index_t idx, float value)
{
	// Store in the raw sample buffer:
	surfaceSample.surface[idx] = value;
}

void wowlSampleVisit(void)
{
	static wowl_state_t prevState = STATE_INIT;
	const wowl_state_t state = wowlSmGetState();
	
	// Visit child modules:
	wowlPressVisit();
	wowlTempVisit();	
	
	if (state == STATE_UNDER_SAMPLE || state == STATE_SURFACE_SAMPLE) {
		if (prevState != state) {
			// Just started.  Start the wakeup timer.
			assert(sampleState == SS_SLEEP);
			sampleState = SS_WAKEUP;
			wakeup = false;
			const uint32_t err = app_timer_start(
					wakeupTimer,
					// interval, in app timer ticks:
					APP_TIMER_FREQ_HZ * WAKEUP_TIME_MSEC / MSEC_PER_SEC, 
					NULL                               // callback context
			);			
			assert(err == NRF_SUCCESS);
			// Acquire internal ADC measurements.  This is synchronous.
			if (state == STATE_UNDER_SAMPLE) {
				wowlAdcTriggerSubmerged();
			} else {
				wowlAdcTriggerSurface();
			}
						
		} else {
			// We have been in the state.  Perform tasks as per the local
			//  state machine.
			switch (sampleState) {
				case SS_SLEEP:
				// NOP.
				break;
				
				case SS_WAKEUP:
				if (wakeup) {
					// Start conversions.
					sampleState = SS_TRIG1;
					readyFlags = 0u;
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
					// Set event.
					wowlSmSetEvent(EVENT_SAMPLE_READY);
					if (state == STATE_SURFACE_SAMPLE) {
						gatherSurfaceSample();
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
			wowlAdcTriggerSubmerged();
			
			if (surfaceSample.submerged[SI_BATTERY_VOLTAGE] > BATT_VOLT_THRESH_CHARGED) {
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
	const float press_mbar = surfaceSample.submerged[SI_PRESS_MBAR];
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
	if (surfaceSample.submerged[SI_BATTERY_VOLTAGE] < BATT_VOLT_THRESH_LIMP) {
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
	for (size_t iVal = 0u; iVal < _countof(surfaceSample.submerged); ++iVal) {
		BIND(surfaceSample.submerged[iVal], MIN_SENSOR[iVal], MAX_SENSOR[iVal]);
	}
	// Pack the sample into fixed point encodings.
	//  Note the 0.5 before the cast is for rounding.
	sample.packed
			= (uint32_t) (surfaceSample.submerged[SI_PRESS_MBAR] + 0.5f) << 17
			| (uint32_t) ((surfaceSample.submerged[SI_TEMP_TI_C]+5.0f)*100.0f + 0.5f) << 5
			| (uint32_t) ((surfaceSample.submerged[SI_BATTERY_VOLTAGE]-1.5f)*20.0f + 0.5f);
}

static void gatherSurfaceSample(void)
{
	// Increment count:
	++surfaceSample.count;
	// Record status bits:
	surfaceSample.status = wowlMainGetStatus();
	// Update the Bluetooth data frame.
	wowlServiceTransmitResults(&surfaceSample);
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


