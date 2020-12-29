/*
	wowlLed.c
	
	Module to manage the state of the LED.
	
	2020-09-22  WHF  Created.
*/

#include "wowl.h"


// nRF SDK:
#include <nrf_gpio.h>
#include <timer/app_timer.h>

//**********************************  Types  *********************************//
typedef enum tag_led_state {
	LED_STATE_OFF,
	LED_STATE_LIMP,
	LED_STATE_CHG_NO_BLE,
	LED_STATE_NO_CHG_NO_BLE,
	LED_STATE_CHG_BLE,
	LED_STATE_NO_CHG_BLE,
	LED_STATE_DOWNLOAD,
	LED_STATE_ERASE,
	
	N_LED_STATES
} led_state_t;

//********************************  Constants  *******************************//

#define TIMER_FREQ_HZ                                                  100U
#define TIMER_INTERVAL_TICKS            (APP_TIMER_FREQ_HZ / TIMER_FREQ_HZ)
#define TIMER_INTERVAL_MSEC                         (1000U / TIMER_FREQ_HZ)

#define RED_MASK                                        (1u << PIN_LED_RED)
#define GRN_MASK                                        (1u << PIN_LED_GRN)
#define YEL_MASK                                        (1u << PIN_LED_YEL)

// Mask for all LEDs on.
#define ALL_LEDS_MASK                      (RED_MASK | GRN_MASK | YEL_MASK)

/////  LED Display Timing  /////
#define ADVERT_PERIOD_MSEC                                             100U
#define ADVERT_DUTY_MSEC                                                10U

#define CONNECTED_PERIOD_MSEC                                         1000U
#define CONNECTED_DUTY_MSEC                                             10U

#define DEFAULT_PERIOD_MSEC                                           1000U
#define DEFAULT_DUTY_MSEC                                                0U

//*******************************  Module Data  ******************************//
static const struct tag_led_modes {
	uint32_t
		duty_msec,       // on-time
		period_msec,
		mask;
} LED_MODES[] = {
	{ 0u, UINT32_MAX, 0u },                     // LED_STATE_OFF,
	{ 10u, 30000u, RED_MASK            },       // LED_STATE_LIMP,
	{ 10u, 100u,   GRN_MASK            },       // LED_STATE_CHG_NO_BLE,
	{ 10u, 10000u, GRN_MASK            },       // LED_STATE_NO_CHG_NO_BLE,
	{ 10u, 100u,   YEL_MASK | GRN_MASK },       // LED_STATE_CHG_BLE,
	{ 10U, 10000U, YEL_MASK | GRN_MASK },       // LED_STATE_NO_CHG_BLE,
	{ 10u, 500u,   YEL_MASK            },       // LED_STATE_DOWNLOAD,   
	{ 10u, 1000u,  ALL_LEDS_MASK       },       // LED_STATE_ERASE,
};                                              
static_assert(
		_countof(LED_MODES) == N_LED_STATES,
		"LED mode mapping incomplete."
);

static uint32_t
	now_msec,        // current time
	start_msec;      // last period start
	
// The handle to the 'application timer', which runs off of RTC1 for a low
//  power time-base.
APP_TIMER_DEF(hAppTimer);	

static led_state_t ledState;

static const led_state_t LED_STATE_MAP[] = {
	LED_STATE_OFF,                        // STATE_INIT,
	LED_STATE_OFF,                        // STATE_SURFACE,
	LED_STATE_OFF,                        // STATE_SURFACE_SAMPLE,
	LED_STATE_LIMP,                       // STATE_LIMP,
	LED_STATE_OFF,                        // STATE_UNDER_SLEEP,
	LED_STATE_OFF,                        // STATE_UNDER_SAMPLE,
	LED_STATE_OFF,                        // STATE_UNDER_WRITE,
	LED_STATE_DOWNLOAD,                   // STATE_DOWNLOAD,
	LED_STATE_ERASE,                      // STATE_ERASE,

};
static_assert(
		_countof(LED_STATE_MAP) == WOWL_N_STATES,
		"Led state map incomplete."
);

//*********************************  Macros  *********************************//
#define LED_INACTIVE_STATE(s)                                             \
		(1u << (s) & (1u << STATE_UNDER_SLEEP                             \
		| 1u << STATE_UNDER_SAMPLE | 1u << STATE_UNDER_WRITE))
	

//***********************  Local Function Declarations  **********************//
static void ledTimerCallback(void* pContext);

//****************************  Global Functions  ****************************//

void wowlLedInit(void) 
{
	///// Configure Outputs  /////
	// Note that the LEDs are active low.
	static const int8_t outputPins[] = {
		PIN_LED_RED,
		PIN_LED_YEL,
		PIN_LED_GRN,
	};
	
	for (uint32_t iPin = 0u; iPin < _countof(outputPins); ++iPin) {
		// Latch each pin high, and then set as an output pin:
		nrf_gpio_pin_set(outputPins[iPin]);
		nrf_gpio_cfg_output(outputPins[iPin]);
	}	
	
	// Create an application timer, to manage the LED state machine.
	uint32_t err = app_timer_create(
			&hAppTimer,
			APP_TIMER_MODE_REPEATED,
			ledTimerCallback
	);
	
	assert(err == 0U);
	
	// Start the timer.
	err = app_timer_start(
			hAppTimer,
			TIMER_INTERVAL_TICKS,
			NULL                         // callback context
	);
	
	assert(err == 0U);
}

void wowlLedVisit(void)
{
	// Reconstruct LED state from global and BLE states.
	static wowl_state_t prevState = STATE_INIT;
	const wowl_state_t state = wowlSmGetState();
	const uint32_t status = wowlMainGetStatus();
	
	if (state != prevState) {
		// Transition.
		if (state == STATE_UNDER_SLEEP) {
			// Stop the LED timer.
			app_timer_stop(hAppTimer);
		} else if (LED_INACTIVE_STATE(prevState) && !LED_INACTIVE_STATE(state)){
			// We need the LED again.  Restart the timer.
			const uint32_t err = app_timer_start(
					hAppTimer,
					TIMER_INTERVAL_TICKS,
					NULL                         // callback context
			);
			assert(err == 0u);
		} else {
			// Ignore other transitions.
		}
			
		// Cache state.
		prevState = state;
	} else {
		// No transition.
	}
	
	if (state == STATE_SURFACE || state == STATE_SURFACE_SAMPLE) {
		if (status & WOWL_STATUS_CONNECTED) {
			if (status & WOWL_STATUS_CHARGING) { 
				ledState = LED_STATE_CHG_BLE;
			} else {
				ledState = LED_STATE_NO_CHG_BLE;
			}
		} else {
			if (status & WOWL_STATUS_CHARGING) { 
				ledState = LED_STATE_CHG_NO_BLE;
			} else {
				ledState = LED_STATE_NO_CHG_NO_BLE;
			}
		}		
	} else {
		ledState = LED_STATE_MAP[state];
	}
}

//***********************  Local Function Definitions  ***********************//
static void ledTimerCallback(void* pContext)
{
	UNUSED(pContext);
	
	const uint32_t status = wowlMainGetStatus();
	
	// Increment now:
	now_msec += TIMER_INTERVAL_MSEC;
	
	if (now_msec - start_msec >= LED_MODES[ledState].period_msec) {
		// Period has ended.  Enter on-time.  Reset start:
		start_msec = now_msec;
		
		// Set all the LED pins to turn them off:
		nrf_gpio_pins_set(ALL_LEDS_MASK);
		// Turn LEDs on to match this mode:
		nrf_gpio_pins_clear(LED_MODES[ledState].mask);
	} else if (now_msec - start_msec >= LED_MODES[ledState].duty_msec) {
		// On time has ended.  Turn all LEDs off:
		nrf_gpio_pins_set(ALL_LEDS_MASK);
	} else {
		// No action.
	}	
}

