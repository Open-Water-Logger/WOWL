/*
	wowlTemp.c

	Implementation of the Temperature module for the WOWL.  It interfaces to the
	SI7051-A20-IMR digital temperature sensor.

	2020-09-22  WHF  Created.
*/

#include "wowl.h"
#include "wowlSample.h"

#include "si705x.h"

#include <twi_master/nrf_drv_twi.h>


//*******************************  Module Data  ******************************//
static volatile int8_t i2cEvent = -1;

// Receives the results of the A/D conversion.
static uint16_t adcRes;

static bool sampleReady;

//***********************  Local Function Declarations  **********************//
static bool waitI2cDone(void);
static void i2cCallback(const struct tag_nrf_drv_twi_evt *pEvent);

//****************************  Global Functions  ****************************//

void wowlTempInit(void)
{
	bool ok;
	uint8_t cmd, part_id;
	uint16_t cmd2;
	
	// Reset the temperature sensor:
	cmd = SI705X_CMD_RESET;
	ok = wowlInitI2cXfer(SI705X_I2C_ADDR, &cmd, 1, NULL, 0, i2cCallback);

	if (ok) {
		ok = waitI2cDone();
	}
	
	// Wait for the reset.  The datasheet specifies a maximum time of 15 msec.
	WAIT_USEC(15000);  // 15 msec
	
	if (ok) {
		// Read device ID.
		cmd2 = SI705X_CMD2_READ_ID_2;
		ok = wowlInitI2cXfer(SI705X_I2C_ADDR, &cmd2, sizeof(cmd2),
				&part_id, sizeof(part_id), i2cCallback);
	}
		
	if (ok) {
		ok = waitI2cDone();
	}
	
	if (ok) {
		if (part_id != SI7051_ID_CODE) {
			ok = false;
		} else {
			// Part matches.
		}
	}
	
	if (!ok) {
		wowlMainSetStatus(WOWL_STATUS_SI7051_FAIL);
	} else {
		// Ok.
	}
}

void wowlTempTrigger(void)
{
	sampleReady = false;
	// Convert Temperature, stretching the clock.  This is convenient,
	//  but ties up the bus for 7 msec.
	const uint8_t cmd = SI705X_CMD_MEAS_TEMP_HOLD;
	const bool ok = wowlInitI2cXfer(
			SI705X_I2C_ADDR, &cmd, 1,
			&adcRes, 2, i2cCallback
	);
	
	if (!ok) {
		wowlMainSetStatus(WOWL_STATUS_SI7051_FAIL);
	} else {
		// Ok.
	}
}

void wowlTempVisit(void)
{
	if (sampleReady) {
		// The temperature measurement has completed.
		sampleReady = false;
		// Convert:
		const float T = SI705X_CONVERT_CELSIUS(adcRes);
		// Submit:
		wowlSampleSubmit(SI_TEMP_SI_C, T);
	} else {
		// Not yet ready.
	}
}

//***********************  Local Function Definitions  ***********************//
static bool waitI2cDone(void)
{
	bool ok;
	while (i2cEvent == -1) {
		IDLE();
	}
	
	ok = i2cEvent == NRF_DRV_TWI_EVT_DONE;
	i2cEvent = -1; // reset for next time
	return ok;
}
	
static void i2cCallback(const struct tag_nrf_drv_twi_evt *pEvent)
{
	i2cEvent = pEvent->type;
	
	if (i2cEvent == NRF_DRV_TWI_EVT_DONE) {
		sampleReady = true;
	} else {
		assert(false);
		// Not a successful operation.
		wowlMainSetStatus(WOWL_STATUS_SI7051_FAIL);
	}
}


