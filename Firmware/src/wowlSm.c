/*
	wowlSm.c

	Implementation of the Sm module for the WOWL.

	2020-09-23  WHF  Created.
*/

#include "wowl.h"

#include <app_util_platform.h>   // for crit region
#include <state_mach.h>

//********************************  Constants  *******************************//
// Use the state count as the 'any source state' wildcard.
#define STATE_ANY                                             WOWL_N_STATES

//*******************************  Module Data  ******************************//
static const struct tag_transition_table {
	wowl_state_t
		srcState,
		destState;
	wowl_event_t
		event;
} transitionTable[] = { // priority high-to-low
	{ STATE_INIT, STATE_SURFACE, EVENT_INIT_COMPLETE,                 },
	{ STATE_ANY, STATE_LIMP, EVENT_BATTERY_LOW,                       },
	{ STATE_LIMP, STATE_SURFACE, EVENT_BATTERY_CHARGED,               },
	{ STATE_SURFACE, STATE_DOWNLOAD, EVENT_DOWNLOAD                   },
	{ STATE_DOWNLOAD, STATE_SURFACE, EVENT_DOWNLOAD_DONE              },
	{ STATE_SURFACE, STATE_ERASE, EVENT_ERASE                         },
	{ STATE_ERASE, STATE_SURFACE, EVENT_ERASE_DONE                    },
	{ STATE_SURFACE, STATE_UNDER_SLEEP, EVENT_SUBMERGE,               },
	{ STATE_SURFACE, STATE_SURFACE_SAMPLE, EVENT_TRIGGER,             },
	{ STATE_SURFACE_SAMPLE, STATE_SURFACE, EVENT_SAMPLE_READY,        },
	{ STATE_UNDER_SLEEP, STATE_UNDER_SAMPLE, EVENT_TRIGGER,           },
	{ STATE_UNDER_SLEEP, STATE_SURFACE, EVENT_SURFACE                 },
	{ STATE_UNDER_SAMPLE, STATE_UNDER_WRITE, EVENT_SAMPLE_READY,      },
	{ STATE_UNDER_WRITE, STATE_UNDER_SLEEP, EVENT_WRITE_DONE,         },
};

static wowl_state_t wowlState = STATE_INIT;
static uint32_t pendingEvents;

//***********************  Local Function Declarations  **********************//

//****************************  Global Functions  ****************************//

bool wowlSmVisit(void)
{
	bool result;
	
	// We do not want additional events triggered during the search.
	CRITICAL_REGION_ENTER();
	
	// Cache the current event set.
	const uint32_t eventsBefore = pendingEvents;
	
	// Perform state machine transitions.
	STATE_MACHINE_TRANSITION(
			transitionTable,
			wowlState,
			pendingEvents,
			STATE_ANY
	);

	// Take note if an event occurred.	
	result = eventsBefore != pendingEvents;
	
	CRITICAL_REGION_EXIT();
	
	return result;
}

wowl_state_t wowlSmGetState(void) { return wowlState; }
void wowlSmSetEvent(wowl_event_t event)
{
	CRITICAL_REGION_ENTER();
	
	pendingEvents |= 1u << event;
	
	CRITICAL_REGION_EXIT();
}

void wowlSmClearEvents(uint32_t toClear)
{
	CRITICAL_REGION_ENTER();
	
	pendingEvents &= ~toClear;
	
	CRITICAL_REGION_EXIT();
}

uint32_t wowlSmGetEvents(void) { return pendingEvents; }

//***********************  Local Function Definitions  ***********************//

