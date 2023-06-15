/*
	wowlTemp.c

	Implementation of the Temperature module for the WOWL.  It interfaces to the
	SI7051-A20-IMR digital temperature sensor.

	2020-09-22  WHF  Created.
*/

#include "wowl.h"
#include "wowlSample.h"

#include "tmp117.h"

#include <ByteSwap.h>
#include <nrf_gpio.h>
#include <twi_master/nrf_drv_twi.h>

//********************************  Constants  *******************************//
// Desired configuration mode register value, with the conversion mode withheld.
//  Note that it is important that the ALERT pin be active low, or it will be
//  fighting our pullup in the default state.
#define CONFIG_REG_SETTING                                                \
		( TMP117_CONFIG_CONV_15_5_MSEC /* not applicable in one-shot */   \
		/* 8 averages to achieve accuracy target: */                      \
		| TMP117_CONFIG_AVG_8                                             \
		| TMP117_CONFIG_POL_ACTIVE_LO  /* active low alert pin  */       \
		| TMP117_CONFIG_ALERT_PIN_DRDY)/* alert-pin is data-ready */



//*******************************  Module Data  ******************************//
static volatile int8_t i2cEvent = -1;

// Flag, set true when the ALERT pin has gone high, and cleared when the
//  sample is read.
static bool sampleReady;

//***********************  Local Function Declarations  **********************//
static void alertCallback(uint32_t pin, nrf_gpiote_polarity_t action);
static bool waitI2cDone(void);
static void i2cCallback(const struct tag_nrf_drv_twi_evt *pEvent);

// TODO: experiment with reading the temperature without writing 
//  the 'register pointer' every time.
static bool readRegister(uint8_t reg, uint16_t *pVal);
static bool writeRegister(uint8_t reg, uint16_t value);

//****************************  Global Functions  ****************************//

void wowlTempInit(void)
{
	bool ok;
	uint16_t part_id;
	
	// Reset the temperature sensor:
	ok = writeRegister(TMP117_REG_CONFIG, TMP117_CONFIG_RESET);
	
	if (ok) {
		// Wait for the reset.  The datasheet specifies a max time of 2 msec.
		WAIT_USEC(2000);  // 2 msec
		
		// Read device ID.
		ok = readRegister(TMP117_REG_DEVICE_ID,	&part_id);
	} else {
		// Already failed.
	}
	
	if (ok) {
		if (part_id != TMP117_DEV_ID) {
			ok = false;
		} else {
			// Part matches.
		}
	} else {
		// Already failed.
	}
	
	if (ok) {
		// Set the configuration register.
		ok = writeRegister(
				TMP117_REG_CONFIG,
				TMP117_CONFIG_MOD_SHUTDOWN | CONFIG_REG_SETTING
		);
	} else {
		// Already failed, so do not write.
	}
	
	if (!ok) {
		assert(false);
		wowlMainSetStatus(WOWL_STATUS_TEMP_IC_FAIL);
	} else {
		// Ok.  Setup GPIO interrupt for the alert pin.  Note that the alert
		//  pin on the sensor is open-drain, so requires a pull-up.
		wowlInitSetupGpioInterrupt(
				PIN_T_ALERT,
				alertCallback,
				NRF_GPIOTE_POLARITY_HITOLO,  // falling edge
				NRF_GPIO_PIN_PULLUP
		);
	}
}

void wowlTempTrigger(void)
{
	sampleReady = false;
	
	// Trigger a one-shot conversion by writing to the configuration register.
	const bool ok = writeRegister(
			TMP117_REG_CONFIG,
			TMP117_CONFIG_MOD_ONESHOT | CONFIG_REG_SETTING
	);
		
	if (!ok) {
		wowlMainSetStatus(WOWL_STATUS_TEMP_IC_FAIL);
		
		// Submit a false temperature, so that the sample module does not
		//  block waiting for us.
		wowlSampleSubmitSubmerged(SI_TEMP_TI_C, 999.0f);
	} else {
		// Ok.
	}
}

void wowlTempVisit(void)
{
	if (sampleReady) {
		// The temperature measurement is ready.
		sampleReady = false;
		
		uint16_t temp_degC_Q7;
		const bool ok = readRegister(
				TMP117_REG_TEMP_RESULT,
				&temp_degC_Q7
		);
		if (ok) {
			// Submit the value after converting to Celsius.
			wowlSampleSubmitSubmerged(
					SI_TEMP_TI_C,
					TMP117_CONVERT_CELSIUS(temp_degC_Q7)
			);
		} else {
			// Not a successful operation.
			wowlMainSetStatus(WOWL_STATUS_TEMP_IC_FAIL);
			
			// Submit a false temperature, so that the sample module does not
			//  block waiting for us.
			wowlSampleSubmitSubmerged(SI_TEMP_TI_C, 999.9f);
		}
	} else {
		// Not yet ready.
	}
}

//***********************  Local Function Definitions  ***********************//
static void alertCallback(uint32_t pin, nrf_gpiote_polarity_t action)
{
	assert(pin == PIN_T_ALERT);
		
	// A sample is ready to be read.
	sampleReady = true;
	
	// The action input merely reflects what we requested, not the current pin
	//   state.
	UNUSED(action);	
}

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
		// Ok.
	} else {
		// Not a successful operation.
		wowlMainSetStatus(WOWL_STATUS_TEMP_IC_FAIL);
		
		// Submit a false temperature, so that the sample module does not
		//  block waiting for us.
		wowlSampleSubmitSubmerged(SI_TEMP_TI_C, 999.9f);
	}
}

static bool readRegister(uint8_t reg, uint16_t *pVal)
{
	bool ok = wowlInitI2cXfer(
			TMP117_I2C_ADDR, &reg, sizeof(reg), pVal, sizeof(*pVal), i2cCallback
	);

	if (ok) {
		ok = waitI2cDone();
	} else {
		// Already failed so do not wait.
	}
	
	// Swap the byte order, since the register is returned MSB.
	*pVal = htons(*pVal);

	return ok;
}

static bool writeRegister(uint8_t reg, uint16_t value)
{
	uint8_t cmd[] = {
		reg,
		(uint8_t) (value >> 8),  // value MSB
		(uint8_t) value          // value LSB
	};
	
	bool ok = wowlInitI2cXfer(
			TMP117_I2C_ADDR, cmd, sizeof(cmd), NULL, 0, i2cCallback
	);

	if (ok) {
		ok = waitI2cDone();
	}

	return ok;
}

