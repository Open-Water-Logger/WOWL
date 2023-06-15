/*
	wowlPress.c

	Implementation of the Pressure module for the WOWL.  It interfaces to the
	MS583730BA01-50 digital pressure sensor.

	2020-09-22  WHF  Created.
*/

#include "wowl.h"
#include "wowlSample.h"
#include "ms5837-30ba.h"

#include <byteswap.h>
#include <nrf_gpio.h>
#include <timer/app_timer.h>
#include <twi_master/nrf_drv_twi.h>


//********************************  Constants  *******************************//
// Dependent on the Over-Sampling Ratio.  See the datasheet.
//  Over 9 msec is required for an OSR of 4096, so we use 10. 
#define MEAS_TIME_MSEC                                                  10u

//*******************************  Module Data  ******************************//
static enum tag_press_state {
	PS_READY,
	PS_PRESS_TRIG_OK,   // wait for I2C op to complete
	PS_WAIT_PRESS,      // wait for timer to expire
	PS_PRESS_SAMP_OK,   // wait for I2C op to complete
	PS_TEMP_TRIG_OK,    // wait for I2c op to complete
	PS_WAIT_TEMP,       // wait for timer to expire
	PS_TEMP_SAMP_OK     // wait for I2c op to complete
} pressState;

static volatile int8_t i2cEvent = -1;

// Receives the results of the A/D conversion.
static uint32_t adcRes;

static bool timerExpired;

// Coefficients read from the device's PROM.
static uint16_t C[MS5837_CRC_LENGTH];

// NRF macro to allocate space for a timer instance.
APP_TIMER_DEF(measTimer);	


//***********************  Local Function Declarations  **********************//
static bool waitI2cDone(void);
static void i2cCallback(const struct tag_nrf_drv_twi_evt *pEvent);

static void convertAndSubmit(uint32_t rawPress, uint32_t rawT);
static unsigned char crc4(const uint16_t *n_prom);
static void pressAppTimerCallback(void *pUnused);

//****************************  Global Functions  ****************************//

void wowlPressInit(void)
{
	bool ok;
	uint8_t cmd;
	
	// Configure the submerge test button pin as an input with a pullup:
	nrf_gpio_cfg_input(
			PIN_SUBMERGE_TEST,
			NRF_GPIO_PIN_PULLUP
	);	
	
	// Reset the pressure sensor:
	cmd = MS5837_CMD_RESET;
	ok = wowlInitI2cXfer(MS5837_I2C_ADDR, &cmd, 1, NULL, 0, i2cCallback);

	if (ok) {
		// We ignore the return result, as the device may NACK the address
		//  for this command.
		waitI2cDone();
	}
	
	// Wait for the reset.  The datasheet is unclear on how long this takes.
	WAIT_USEC(1000);  // 1 msec
	
	if (ok) {
		// Read the contents of the device's PROM.
		// Zero them initially:
		memset(C, 0, sizeof(C));
		for (int iProm = 0; iProm < MS5837_PROM_LENGTH; ++iProm) {
			cmd = MS5837_CMD_READ_PROM(MS5837_PROM_ADDR_CRC_ID + iProm);
			ok = wowlInitI2cXfer(MS5837_I2C_ADDR, &cmd, 1,
					&C[iProm], sizeof(C[iProm]), i2cCallback);
		
			if (ok) {
				ok = waitI2cDone();
			}
		
			if (ok) {
				C[iProm] = htons(C[iProm]);
			}
		}
	}
	
	if (ok) {
		const uint8_t out = crc4(C);
		// If successful, the CRC should be zero.
		ok = out == 0u;
	} else {
		wowlMainSetStatus(WOWL_STATUS_PRESS_FAIL);
	}
	
	// Setup the measurement timer.
	uint32_t err = app_timer_create(
			&measTimer,
			APP_TIMER_MODE_SINGLE_SHOT,
			pressAppTimerCallback
	);
	
	assert(err == NRF_SUCCESS);
}

void wowlPressTrigger(void)
{
	bool ok;
	uint8_t cmd;
	
	assert(pressState == PS_READY);
	
	cmd = MS5837_CMD_CONVERT_PRESS(MS5837_OSR_4096);
	i2cEvent = -1; // reset event
	ok = wowlInitI2cXfer(MS5837_I2C_ADDR, &cmd, 1,
			NULL, 0, i2cCallback);
			
	if (!ok) {
		wowlMainSetStatus(WOWL_STATUS_PRESS_FAIL);
	} else {
		// Ok.
		pressState = PS_PRESS_TRIG_OK;
	}
}

void wowlPressVisit(void)
{
	static uint32_t rawPress;
	
	switch (pressState) {
		case PS_READY:
		// Nop.
		break;
		
		case PS_PRESS_TRIG_OK:
		if (i2cEvent == NRF_DRV_TWI_EVT_DONE) {
			// The trigger transfer has completed.
			
			// Start measurement timer.
			timerExpired = false;
			const uint32_t err = app_timer_start(
					measTimer,
					// interval, in app timer ticks:
					APP_TIMER_FREQ_HZ * MEAS_TIME_MSEC / MSEC_PER_SEC, 
					NULL                               // callback context
			);			
			assert(err == NRF_SUCCESS);
			
			// Enter next state:
			pressState = PS_WAIT_PRESS;
		} else {
			// Continue to wait.
		}
		break;

		case PS_WAIT_PRESS:   
		if (timerExpired) {
			// Our measurement timer has expired.
			// Get result:
			const uint8_t cmd = MS5837_CMD_READ_ADC;
			i2cEvent = -1; // reset event
			const bool ok = wowlInitI2cXfer(MS5837_I2C_ADDR, &cmd, 1,
					&adcRes, 3,   // 24-bit data 
					i2cCallback
			);
			if (!ok) {
				wowlMainSetStatus(WOWL_STATUS_PRESS_FAIL);
				// Retry (re-visit).
			} else {
				// Ok.
				pressState = PS_PRESS_SAMP_OK;
			}
		} else {
			// Timer still counting.
		}
		break;
		
		case PS_PRESS_SAMP_OK:
		if (i2cEvent == NRF_DRV_TWI_EVT_DONE) {
			// The pressure ADC result has been read.
			// Byte swap and shift output:
			rawPress = htonl(adcRes) >> 8;
		
			// Request temperature scan:
			const uint8_t cmd = MS5837_CMD_CONVERT_TEMP(MS5837_OSR_4096);
			i2cEvent = -1; // reset event
			const bool ok = wowlInitI2cXfer(MS5837_I2C_ADDR, &cmd, 1,
					NULL, 0, i2cCallback);
			if (!ok) {
				wowlMainSetStatus(WOWL_STATUS_PRESS_FAIL);
				// Retry (revisit).
			} else {
				// Ok.
				pressState = PS_TEMP_TRIG_OK;
			}
		} else {
			// I2C operation continues.
		}
		break;
		
		case PS_TEMP_TRIG_OK: 
		if (i2cEvent == NRF_DRV_TWI_EVT_DONE) {
			// The temperature scan went through.  Set conversion timer.
			timerExpired = false;
			const uint32_t err = app_timer_start(
					measTimer,
					// interval, in app timer ticks:
					APP_TIMER_FREQ_HZ * MEAS_TIME_MSEC / MSEC_PER_SEC, 
					NULL                               // callback context
			);			
			assert(err == NRF_SUCCESS);
			// Enter next state:
			pressState = PS_WAIT_TEMP;
		} else {
			// I2C operation continues.
		}
		break;
			
		case PS_WAIT_TEMP:
		if (timerExpired) {
			// Our measurement timer has expired.
			// Get result:
			const uint8_t cmd = MS5837_CMD_READ_ADC;
			i2cEvent = -1; // reset event
			const bool ok = wowlInitI2cXfer(MS5837_I2C_ADDR, &cmd, 1,
					&adcRes, 3,   // 24-bit data 
					i2cCallback
			);
			if (!ok) {
				wowlMainSetStatus(WOWL_STATUS_PRESS_FAIL);
				// Retry (re-visit).
			} else {
				// Ok.  Enter state to wait for operation completion:
				pressState = PS_TEMP_SAMP_OK;
			}
		} else {
			// Timer still counting.
		}
		break;
		
		case PS_TEMP_SAMP_OK:  		
		if (i2cEvent == NRF_DRV_TWI_EVT_DONE) {
			// Byte swap and shift output:
			const uint32_t rawT = htonl(adcRes) >> 8;
			
			convertAndSubmit(rawPress, rawT);
			pressState = PS_READY;
		} else {
			// Operation not yet complete.
		}
		break;
		
		default:
		assert(false);
		pressState = PS_READY;
		break;
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
}

// Note that C[0] is the checksum, so C[1] is C1, not an off-by-one error.
// These conversion formulae are strictly from the MS5837 datasheet.
static void convertAndSubmit(uint32_t rawP, uint32_t rawT)
{
	const int64_t
		dT = (int32_t) rawT - (int32_t) C[5] * (1 << 8),
		TEMP = 2000 + (int32_t) (dT * (int64_t) C[6] >> 23),
		Tm20 = TEMP - 2000,
		OFF =  ((int64_t) C[2] << 16) + ((int64_t) C[4] * dT >> 7),
		SENS = ((int64_t) C[1] << 15) + ((int64_t) C[3] * dT >> 8);

	// 1st order pressure.  Unused.		
	//const int32_t
	//	P = (int32_t) ((((int64_t) rawP * SENS) >> 21) - (OFF >> 13));
		
	int64_t Ti, OFFi, SENSi;
		
	if (TEMP < 2000) {
		// Low temperature compensation.
		Ti    = (3 * dT * dT) >> 33;
		OFFi  = (3 * Tm20 * Tm20) >> 1;
		SENSi = (5 * Tm20 * Tm20) >> 3;
		
		if (TEMP < -1500) {
			// Very low temperature compensation.
			const int64_t Tp15 = TEMP + 1500;
			OFFi  += 7 * Tp15 * Tp15;
			SENSi += 4 * Tp15 * Tp15;
		} else {
			// Do not apply very low temperature compensation.
		}
	} else {
		// High temperature compensation.
		Ti    = (2 * dT * dT) >> 37;
		OFFi  = (1 * Tm20 * Tm20) >> 4;
		SENSi = 0;
	}
	
	// Apply the compensation.
	const int64_t
		OFF2  = OFF - OFFi,
		SENS2 = SENS - SENSi;
		
	const float
		TEMP2 = (float) (TEMP - Ti) * 0.01f,
		P2 = (float) ((((int64_t) rawP * SENS2 >> 21) - OFF2) >> 13) * 0.1f;
		
	// Add a pressure offset to simulate submersion, if the button is
	//  depressed.
	const float offset = nrf_gpio_pin_read(PIN_SUBMERGE_TEST)
			? 0.0f    // pulled up, no offset
			: 10e3f;  // pulled down, 10 bar offset
		
	// Submit, adding user-test offset.
	wowlSampleSubmitSurface(SS_PRESS_TEMP_C, TEMP2);
	wowlSampleSubmitSubmerged(SI_PRESS_MBAR, P2 + offset);
}

// CRC of 8 values, the last which must be forced to zero.  If correct,
//  the output will be zero.
static unsigned char crc4(const uint16_t *n_prom)
{
	int cnt; // simple counter
	unsigned int n_rem=0; // crc remainder
	unsigned char n_bit;

	// operation is performed on bytes:
	for (cnt = 0; cnt < MS5837_CRC_LENGTH*2; cnt++) 
	{ // choose LSB or MSB
		if (cnt%2==1) n_rem ^= (unsigned short) ((n_prom[cnt>>1]) & 0x00FF);
		else n_rem ^= (unsigned short) (n_prom[cnt>>1]>>8);
		for (n_bit = 8; n_bit > 0; n_bit--)
		{
			if (n_rem & (0x8000)) n_rem = (n_rem << 1) ^ 0x3000;
			else n_rem = (n_rem << 1);
		}
	}
	n_rem= ((n_rem >> 12) & 0x000F); // final 4-bit remainder is CRC code
	return n_rem;
}

static void pressAppTimerCallback(void *pUnused)
{
	UNUSED(pUnused);
	
	timerExpired = true;
}

