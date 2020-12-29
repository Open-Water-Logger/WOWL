/*
	wowlTests.c
	
	Unit testing of hardware and software for the WOWL.
	
	2020-09-22  WHF  Created.
*/

#include "wowl.h"

#include <ads1120.h>
#include <byteswap.h>
#include <ms5837-30ba.h>
#include <si705x.h>
#include <spi_flash.h>
#include <stdio.h>
#include <timer.h>

#include <nrf_delay.h>
#include <nrf_gpio.h>
#include <nrf_pwm.h>
#include <nrf_soc.h>

#include <saadc/nrf_drv_saadc.h>
#include <twi_master/nrf_drv_twi.h>
#include <timer/app_timer.h>

// Turn off warning (defined but not used)
#pragma diag_suppress=Pe177

//********************************  Constants  *******************************//

// Entire file excluded if tests not performed:
#if DO_TESTS

#ifdef NDEBUG
//#	error Cannot set DO_TESTS in Release build.
#define fputs(...)
#undef stderr
#define stderr
#endif

// The period, in Ticks at 125 kHz.
#define PWM_PERIOD                                                    1000U

//**********************************  Types  *********************************//
// Pointer to test function:
typedef bool (*test_fun_t)(void);

typedef struct {
	test_fun_t testFun;
	const char* name;
} TEST;
	

//***********************  Local Function Declarations  **********************//
static bool testAdc(void);
static bool testAppTimer(void);
static bool testBme280(void);
static bool testConf(void);
static bool testFlash(void);
static bool testGpioLeds(void);
static bool testGpioTps(void);
static bool testPress(void);
static bool testSample(void);
static bool testSystemOff(void);
static bool testPwm(void);
static bool testTempInternal(void);
static bool testTempNtc(void);
static bool testTempRtd(void);
static bool testTempSi7501(void);
static bool testTimer(void);


//*****************************  Global Function  ****************************//
void wowlTests(void)
{
	static const char *resultStr[] = { "FAILED\n", "PASSED\n" };
	
	TEST tests[] = {
//		{ testAdc, "ADC",                    },
//		{ testAppTimer, "AppTimer",          },
//		{ testBme280, "BME280"               },
//		{ testConf, "Conf",                  },
//		{ testFlash, "Flash",                },
//		{ testGpioLeds, "GPIO LEDs",         },
		{ testGpioTps,  "GPIO TPs",          },
//		{ testSample, "Sample",              },
//		{ testSystemOff, "System OFF"        },
//		{ testPress, "PRESSURE"              },
//		{ testPwm, "PWM",                    },
//		{ testTempInternal, "TEMP",          },
//		{ testTempNtc, "TEMP NTC",           },
//		{ testTempRtd, "TEMP RTD",           },
//		{ testTempSi7501, "TEMP Si",         },
//		{ testTimer, "Timer"                 },
	};
	int i;	
	bool result;
	
	fputs("*** WOWL TESTS ***\n", stderr);
	
	for (i = 0; i < _countof(tests); ++i) {
		fputs(tests[i].name, stderr);
		fputs(": ", stderr);
		result = tests[i].testFun();
		(void) result;
		fputs(resultStr[result], stderr);
	}

	DEBUG_BREAK();
	
	for (;;) ;
}

//****************************  Test Definitions  ****************************//
#if 0
static bool testAdc(void)
{
	static const char* CHAN_DESC[] = {
		"Vchg",
		"Ichg",
		"Vpdc",
		"Vbat",
		"Tcpu"
	};

	wowlAdcInit();
	const INT_ADC* const pAdc = wowlAdcGet();
	uint32_t prevCount = 0;
	
	for (;;) {
		IDLE();
		wowlAdcVisit();
		
		if (pAdc->count != prevCount) {
			prevCount = pAdc->count;
			for (int iChan = 0; iChan < _countof(CHAN_DESC); ++iChan) {
				printf("%s=%.3f, ", CHAN_DESC[iChan], pAdc->measurements[iChan]);
			}
			puts("");
		} else {
			// Wait for sample
		}
	}
}
#endif

static void testAppTimerCallback(void *pContext)
{
	nrf_gpio_pin_toggle(PIN_LED_RED);
	++*(uint32_t*) pContext;	
}
static bool testAppTimer(void)
{
	const uint32_t timerFreqHz = 1;
	uint32_t count = 0;
	
	nrf_gpio_cfg_output(PIN_LED_RED);

	APP_TIMER_DEF(hAppTimer);	
	
	// Create an application timer:
	uint32_t err = app_timer_create(
			&hAppTimer,
			APP_TIMER_MODE_REPEATED,
			testAppTimerCallback
	);
	
	assert(err == 0U);
	
	// Start the timer.
	err = app_timer_start(
			hAppTimer,
			(APP_TIMER_FREQ_HZ / timerFreqHz), // interval, app timer ticks
			&count                             // callback context
	);
	
	assert(err == 0U);
	
	while (count < 10) {
		IDLE();
	}
	
	app_timer_stop(hAppTimer);
	
	return true;
}

#if 0
static bool testBme280(void)
{
	bool result;
	
	wowlBme280_Init();

	result = wowlMainGetStatus() & WOWL_STATUS_HUMIDITY_INIT_FAIL ? false : true;
	
	for (int iMeas = 0; result && iMeas < 1000; ++iMeas) {
		wowlBme280_Read();
		
		result = wowlMainGetStatus() & WOWL_STATUS_HUMIDITY_ERROR ? false : true;
		
		if (result) {
			const WOWL_BME280 *pS = wowlBme280_Get();
			printf(
					"%5.1f °C  %8.0f Pa %4.1f%%RH\n",
					pS->temp_C,
					pS->press_Pa,
					pS->rel_hum_pc
			);
		} else {
			// Failed.
		}
	}
	
	return result;
}
#endif

#if 0
static bool testConf(void)
{
	static const char TEST_ID[] = "OAE-1234";
	bool result = wowlConfInit();

	if (result) {
		// Activate the soft device.  This is necessary for most APIs to 
		//  function.
		wowlBleInit();

		result = wowlConfSetId(TEST_ID);
	} else {
		// Failed.
	}
	
	if (result) {
		// Call the visitor, as this does the actual write:
		wowlConfVisit();
		
		const char* pId = wowlConfGetIdFlash();
		
		int cmp = strcmp(TEST_ID, pId);
		
		if (cmp != 0) {
			result = false;
		} else {
			// Compare ok.
		}
	} else {
		// Failed.
	}
	
	return result;
}
#endif

static bool testFlash(void)
{
	uint8_t pageData[256];
	const uint32_t nPages = 4096u / sizeof(pageData);
	int err = 0;
	bool result = true;
	
	wowlFlashInit();
	
	// Erase the 4k sector.
	err = spiFlashSectorErase(0u);
	if (!err) {
		// Wait for it to complete.
		while (spiFlashIsBusy()) ;
	}
		
	for (uint32_t iPage = 0u; !err && iPage < nPages; ++iPage) {
		// Fill the page with data:
		for (int iD = 0; iD < sizeof(pageData); ++iD) {
			pageData[iD] = iPage + iD;
		}
		
		// Write the page to the device:
		err = spiFlashWrite(iPage*sizeof(pageData), pageData, sizeof(pageData));
		
		if (!err) {
			// Wait for it to complete.
			while (spiFlashIsBusy()) ;
		} else {
			result = false;
		}
	}
	
	memset(pageData, 0, sizeof(pageData));
	
	// Read back and verify:
	for (uint32_t iPage = 0u; result && iPage < nPages; ++iPage) {
		// Read the page from the device:
		err = spiFlashRead(iPage*sizeof(pageData), pageData, sizeof(pageData));
		
		if (!err) {
			// Compare
			for (int iD = 0; iD < sizeof(pageData); ++iD) {
				if (pageData[iD] != ((iPage + iD) & 0xFF)) {
					result = false;
					break;
				} else {
					// Continue verification
				}
			}
		} else {
			result = false;
		}
	}
		
	return result;
}

// Also tests delay function.
static bool testGpioLeds(void)
{
	static const uint8_t ledPins[] = {
		PIN_LED_RED,
		PIN_LED_GRN,
		PIN_LED_YEL,
	};
		
	// Set the pin as an output, in case the LED module has not been
	//  initialized:
	for (int iLed = 0; iLed < _countof(ledPins); ++iLed) {
		nrf_gpio_cfg_output(ledPins[iLed]);
	}

	for (;;) { 
		for (int iLed = 0; iLed < _countof(ledPins); ++iLed) {
			// Turn the LED on (active low):
			nrf_gpio_pin_clear(ledPins[iLed]);
			WAIT_USEC(500000UL); // wait ½ a sec
			// Turn the LED off:
			nrf_gpio_pin_set(ledPins[iLed]);
			WAIT_USEC(500000UL); // wait ½ a sec
		}
	}
}

static bool testGpioTps(void)
{
	for (;;) {
		WAIT_USEC(1000u); // 1msec
		nrf_gpio_pin_toggle(PIN_TP21);
	}
}	

static volatile int8_t i2cEvent = -1;
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

// Note that C[0] is the checksum, so C[1] is C1, not an off-by-one error.
static void convert(const uint16_t *C, uint32_t rawP, uint32_t rawT)
{
	const int64_t
		dT = (int32_t) rawT - (int32_t) C[5] * (1 << 8),
		TEMP = 2000 + (int32_t) (dT * (int64_t) C[6] >> 23),
		Tm20 = TEMP - 2000,
		OFF =  ((int64_t) C[2] << 16) + ((int64_t) C[4] * dT >> 7),
		SENS = ((int64_t) C[1] << 15) + ((int64_t) C[3] * dT >> 8);
		
	const int32_t
		P = (int32_t) ((((int64_t) rawP * SENS) >> 21) - (OFF >> 13));
		
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
		
	printf("T = %6.2f, P = %7.1f\n", TEMP2, P2);	
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
static void i2cCallback(const struct tag_nrf_drv_twi_evt *pEvent)
{
	i2cEvent = pEvent->type;
}
static bool testPress(void)
{
	bool ok;
	uint8_t cmd;
	uint16_t C[MS5837_CRC_LENGTH];  // prom data
	uint32_t adcRes;
	
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
		printf("\n");
		// Read prom contents.
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
				printf("C[%d] = %04x\n", iProm, C[iProm]);
			}
		}
	}
	if (ok) {
		const uint8_t out = crc4(C);
		printf("CRC = %02x\n", out);
		// If successful, the CRC should be zero.
		ok = out == 0u;
	}
	// Enter loop, converting and reading samples:
	while (ok) {
		// Pressure
		cmd = MS5837_CMD_CONVERT_PRESS(MS5837_OSR_4096);
		ok = wowlInitI2cXfer(MS5837_I2C_ADDR, &cmd, 1,
				NULL, 0, i2cCallback);
		assert(ok);
		ok = waitI2cDone();
		assert(ok);
		WAIT_USEC(10000);  // time needed varies with OSR; see pg2
		// Get result:
		cmd = MS5837_CMD_READ_ADC;
		ok = wowlInitI2cXfer(MS5837_I2C_ADDR, &cmd, 1,
				&adcRes, 3,   // 24-bit data 
				i2cCallback);
		assert(ok);
		ok = waitI2cDone();
		assert(ok);
		// Byte swap and shift output:
		const uint32_t rawP = htonl(adcRes) >> 8;
//		printf("P = %10d  ", rawP);
//		printf("P = %06x  ", rawP);
		
		// Temperature
		cmd = MS5837_CMD_CONVERT_TEMP(MS5837_OSR_4096);
		ok = wowlInitI2cXfer(MS5837_I2C_ADDR, &cmd, 1,
				NULL, 0, i2cCallback);
		assert(ok);
		ok = waitI2cDone();
		assert(ok);
		WAIT_USEC(10000);  // time needed varies with OSR; see pg2
		// Get result:
		cmd = MS5837_CMD_READ_ADC;
		ok = wowlInitI2cXfer(MS5837_I2C_ADDR, &cmd, 1,
				&adcRes, 3,  // 24-bit data
				i2cCallback);
		assert(ok);
		ok = waitI2cDone();
		assert(ok);
		// Byte swap and shift output:
		const uint32_t rawT = htonl(adcRes) >> 8;		
//		printf("T = %10d\n", rawT);
//		printf("T = %06x\n", rawT);
		convert(C, rawP, rawT);
	}
	
	return ok;
}
static void printSample(void)
{
	const SAMPLE* const pS = wowlSampleGet();
	
	printf(
			"\n#%d\n\t"
			"V_chg    = %4.2f\n\t"
			"I_chg    = %5.3f\n\t"
			"V_pdc    = %6.2f\n\t"
			"V_batt   = %6.2f\n\t"
			"T_cpu    = %6.2f\n\t"
			"T_ntc    = %6.2f\n\t"
			"T_rtd    = %6.2f\n\t"
			"T_si     = %6.2f\n\t"
			"P        = %6.2f\n\t"
			"T_press  = %6.2f\n\t"
			"T_ntcAdc = %6.2f\n\t"
			"T_rtdAdc = %6.2f\n\t"
			"\n",
			pS->count,
			pS->values[SI_VCHG],
			pS->values[SI_ICHG],
			pS->values[SI_PDC],
			pS->values[SI_BATTERY_VOLTAGE],
			pS->values[SI_CPU_TEMP],
			pS->values[SI_TEMP_NTC_C],
			pS->values[SI_TEMP_RTD_C],
			pS->values[SI_TEMP_SI_C],
			pS->values[SI_PRESS_MBAR],
			pS->values[SI_PRESS_TEMP_C],
			pS->values[SI_TEMP_INT_NTC_C],
			pS->values[SI_TEMP_INT_RTD_C]
	);
}			
			
static bool testSample(void)
{
	bool result = true;
	
	// Initializes all dependent modules:
	wowlSampleInit();

	// Cycle the state machine:
	wowlSmSetEvent(EVENT_INIT_COMPLETE);
	wowlSmVisit();
	wowlSampleVisit();
	wowlSmSetEvent(EVENT_SUBMERGE);
	
	while (result) {
		wowlSmVisit();
		wowlSampleVisit();
		switch (wowlSmGetState()) {
			case STATE_UNDER_SLEEP:
			// Wait a second, to represent time between samples:
			WAIT_USEC(1000000u);			
			// Trigger another sample:
			wowlSmSetEvent(EVENT_TRIGGER);
			break;
			
			case STATE_UNDER_SAMPLE:
			// Wait for it to complete.
			break;
			
			case STATE_UNDER_WRITE:
			// Print the sample to the console:
			printSample();
			// Pretend that we've written the sample to flash:
			wowlSmSetEvent(EVENT_WRITE_DONE);
			break;
			
			default:
			// We shouldn't see any other states.
			result = false;
			break;
		}
	}
	
	return result;
}

// Tests the ability of the push-button to wake the device from system-off.
static bool testSystemOff(void)
{
	wowlBleInit();	
	
	// Pulse 100 times over 10 seconds
	for (int iPulse = 0; iPulse < 100; ++iPulse) {
		nrf_gpio_pin_set(PIN_LED_RED);
		WAIT_USEC(50'000u); // wait 50 msec
		nrf_gpio_pin_clear(PIN_LED_RED);
		WAIT_USEC(50'000u); // wait 50 msec
	}

	// Enter System Off mode.  The pushbutton should be used to reset.	
	uint32_t err_code = sd_power_system_off();
	assert(err_code == NRF_SUCCESS);

	// If the system fails to power down, indicate by slower blinking of the
	//  LED:
	for (;;) {
		nrf_gpio_pin_set(PIN_LED_RED);
		WAIT_USEC(250'000u); // wait 250 msec
		nrf_gpio_pin_clear(PIN_LED_RED);
		WAIT_USEC(250'000u); // wait 250 msec
	}
	
}

static bool testPwm(void)
{
	static const uint32_t out_pins[4] = {
		PIN_LED_RED,
		NRF_PWM_PIN_NOT_CONNECTED,
		NRF_PWM_PIN_NOT_CONNECTED,
		NRF_PWM_PIN_NOT_CONNECTED
	};
	
	// The compare registers are 15 bit.  The MSB represents the polarity, 
	//   with 0x8000 representing rising-edge-on-compare-match.  The output is
	//   automatically inverted on period match.
	uint16_t compare_values[4] = {
		1 | 0x8000,
		0,
		0,
		0
	};
	
	// Setup GPIO as outputs.
	nrf_gpio_cfg_output(PIN_LED_RED);

	// Set the output pins in the module.
	nrf_pwm_pins_set(NRF_PWM0, (uint32_t*) out_pins);
	// Enable the module.
	nrf_pwm_enable(NRF_PWM0);
	// Configure the clock base, counting mode, and period.
	nrf_pwm_configure(
			NRF_PWM0,
			NRF_PWM_CLK_125kHz,
			NRF_PWM_MODE_UP,
			PWM_PERIOD
	);
	// Set the location and length of the compare values.  These will be 
	//   loaded on sequence start.
	nrf_pwm_seq_ptr_set(NRF_PWM0, 0, compare_values);
	nrf_pwm_seq_cnt_set(NRF_PWM0, 0, _countof(compare_values));
	// Setup the decoder for individual mode (each output has its own 
	//  compare register value).
	nrf_pwm_decoder_set(NRF_PWM0, NRF_PWM_LOAD_INDIVIDUAL, NRF_PWM_STEP_AUTO);
	// Begin PWM.
	nrf_pwm_task_trigger(NRF_PWM0, NRF_PWM_TASK_SEQSTART0);
//	nrf_pwm_task_trigger(NRF_PWM0, NRF_PWM_TASK_SEQSTART0);
	
	for (;;) ;
}

static bool testTempInternal(void)
{
	for (;;) {
#if 0 // The temperature is used by the Soft Device.
		// Clear ready indicator:
		NRF_TEMP->EVENTS_DATARDY = 0u;
	
		// Start temperature measurement:
		NRF_TEMP->TASKS_START = 1;
		
		// Wait for completion:
		while (NRF_TEMP->EVENTS_DATARDY == 0u) ;
		
		const float cpuTemp_C = (float) NRF_TEMP->TEMP * 0.25f;
#else
		int32_t tempRaw;
		sd_temp_get(&tempRaw);
		const float cpuTemp_C = (float) tempRaw * 0.25f;
#endif
		// Print:
		printf("%.2f\n", cpuTemp_C);
	}
}
static bool testTempNtc(void)
{
	bool ok = true;
	const ads1120_gain_t gainChoice = ADS1120_GAIN_1;
	const float 
		gain = (float) (1 << gainChoice), // gain in powers of two
		R_ref_rtd = 12e3f;   // 12 kohms
		
	// Call the external ADC initialization:
	wowlExtAdcInit();		
	
	// Initialize the device:
	ads1120_init(WOWL_ADS1120_INST_NTC);
	
	// Setup the gain and other features.
	// Set the multiplexer for conversions.
	ads1120_set_MUX(WOWL_ADS1120_INST_NTC, ADS1120_MUX_AIN1_AIN2);

	// Set the gain.
	ads1120_set_GAIN(WOWL_ADS1120_INST_NTC, gainChoice);

	// Set the data rate and operating mode.  We'll use the lowest data
	//  rate in normal mode to get the highest SNR.
	ads1120_set_DR(WOWL_ADS1120_INST_NTC, ADS1120_MODE_NORMAL, ADS1120_DR_20);

	// Setup excitation currents and output pins.
	ads1120_set_IEXEC(
			WOWL_ADS1120_INST_NTC,
			ADS1120_IEXEC_UA_50, 
			ADS1120_IMUX_AIN0,    // current out on AIN0/REFP1
			ADS1120_IMUX_NONE     // no second current
	); 

	// Set the voltage reference used for conversions.  We use the 
	//  ratiometric reference created by the current source and the reference
	//  resistor.
	ads1120_set_REF(WOWL_ADS1120_INST_NTC, ADS1120_REF_REFP0_REFN0);

	while (ok) {
		// Turn off the internal temperature sensor:
		ads1120_set_temperature_mode(WOWL_ADS1120_INST_NTC, false);
		
		// Trigger a conversion.
		ads1120_trigger_single_conversion(WOWL_ADS1120_INST_NTC);
		
		// Read nDRDY.  This would normally be implemented via an interrupt.
		while (nrf_gpio_pin_read(PIN_NTC_DRDY)) {
			// Wait
		}
		
		// Read the output.
		const int16_t raw = ads1120_retrieve_sample(WOWL_ADS1120_INST_NTC);
				
		// Convert to a resistance.
		const float R = (float) raw * (R_ref_rtd / gain / (float) (1 << 15));

		printf("T = %6d => %7.1f ohms\n", raw, R);		

		// Turn ON the internal temperature sensor:
		ads1120_set_temperature_mode(WOWL_ADS1120_INST_NTC, true);
		
		// Trigger a conversion.
		ads1120_trigger_single_conversion(WOWL_ADS1120_INST_NTC);
		
		// Read nDRDY.  This would normally be implemented via an interrupt.
		while (nrf_gpio_pin_read(PIN_NTC_DRDY)) {
			// Wait
		}
		
		// Read the output.
		const int16_t rawT = ads1120_retrieve_sample(WOWL_ADS1120_INST_NTC);
		
		// Convert to a temperature.
		const float T = ADS1120_TEMP_SENSOR_CONVERT(rawT);

		printf("T = %6d => % 7.3f °C\n", rawT, T);

		// Simulate sleep.
		ads1120_powerdown(WOWL_ADS1120_INST_NTC);
		
		WAIT_USEC(1000000u);

		// Wakeup by triggering a conversion.
		ads1120_trigger_single_conversion(WOWL_ADS1120_INST_NTC);
		// Wait for the filters to settle.  At room temperature takes about
		//  11 msec to rise.  We'll be about 4 times that resistance near
		//  freezing, so let's go with 50.
		WAIT_USEC(50'000u); 
	}
	
	return ok;
}

static bool testTempRtd(void)
{
	bool ok = true;
	const ads1120_gain_t gainChoice = ADS1120_GAIN_64;
	const float 
		gain = (float) (1 << gainChoice), // gain in powers of two
		R_ref_rtd = 12e3f;   // 12 kohms
		
	// Call the external ADC initialization:
	wowlExtAdcInit();		
	
	// Initialize the device:
	ads1120_init(WOWL_ADS1120_INST_RTD);
	
	// Setup the gain and other features.
	// Set the multiplexer for conversions.
	ads1120_set_MUX(WOWL_ADS1120_INST_RTD, ADS1120_MUX_AIN1_AIN2);

	// Set the gain.
	ads1120_set_GAIN(WOWL_ADS1120_INST_RTD, gainChoice);

	// Set the data rate and operating mode.  We'll use the lowest data
	//  rate in normal mode to get the highest SNR.
	ads1120_set_DR(WOWL_ADS1120_INST_RTD, ADS1120_MODE_NORMAL, ADS1120_DR_20);

	// Setup excitation currents and output pins.
	ads1120_set_IEXEC(
			WOWL_ADS1120_INST_RTD,
			ADS1120_IEXEC_UA_50, 
			ADS1120_IMUX_AIN0,    // current out on AIN0/REFP1
			ADS1120_IMUX_NONE     // no second current
	); 

	// Set the voltage reference used for conversions.  We use the 
	//  ratiometric reference created by the current source and the reference
	//  resistor.
	ads1120_set_REF(WOWL_ADS1120_INST_RTD, ADS1120_REF_REFP0_REFN0);

	while (ok) {
		// Trigger a conversion.
		ads1120_trigger_single_conversion(WOWL_ADS1120_INST_RTD);
		
		// Read nDRDY.  This would normally be implemented via an interrupt.
		while (nrf_gpio_pin_read(PIN_RTD_DRDY)) {
			// Wait
		}
		
		// Read the output.
		const int16_t raw = ads1120_retrieve_sample(WOWL_ADS1120_INST_RTD);

		// Convert to a resistance.
		const float R = (float) raw * (R_ref_rtd / gain / (float) (1 << 15));

		printf("T = %6d => %7.4f ohms\n", raw, R);
		
		// Simulate sleep.
		ads1120_powerdown(WOWL_ADS1120_INST_RTD);
		
		WAIT_USEC(1000000u);

		// Wakeup by triggering a conversion.
		ads1120_trigger_single_conversion(WOWL_ADS1120_INST_RTD);
		// Wait for the filters to settle.
		// 10 msec sufficient.  1 msec insufficient. 5 ok.  2 ok.  1.5 bad.
		// Let's go with 5.
		WAIT_USEC(5000u); 
	}
	
	return ok;
}
static bool testTempSi7501(void)
{
	bool ok;
	uint8_t cmd, part_id;
	uint16_t cmd2;
	uint16_t adcRes;
	
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
		printf("\nID = %04x\n", part_id);
		assert(part_id == SI7051_ID_CODE);		
	}
	// Enter loop, converting and reading samples:
	while (ok) {
		// Convert Temperature, stretching the clock.  This is convenient,
		//  but ties up the bus for 7 msec.
		cmd = SI705X_CMD_MEAS_TEMP_HOLD;
		ok = wowlInitI2cXfer(SI705X_I2C_ADDR, &cmd, 1,
				&adcRes, 2, i2cCallback);
		assert(ok);
		ok = waitI2cDone();
		assert(ok);
		const float T_degC = SI705X_CONVERT_CELSIUS(adcRes);
		printf("T = 0x%06x => %6.3f °C\n", adcRes, T_degC);
	}
	
	return ok;
}

static void timerCallback(void *pUser)
{
	// Increment count:
	++*(unsigned*) pUser;
}
static bool testTimer(void)
{
	unsigned count = 0;
	timer_err_t timer_err;
	const uint32_t TIMER_CLOCK_HZ = 16000000U;
	
	// Start interrupts and the timer:
	ENABLE_INTR();
	timer_err = timerStartRate(
			WOWL_TIMER_ALLOC_TEST,              
			TIMER_CLOCK_HZ,
			10,             // 10 Hz, or 100 msec
			timerCallback,
			&count
	);
	
	// Wait 950 msec:
	WAIT_USEC(950000UL);
	
	// Stop the timer:
	timerStop(WOWL_TIMER_ALLOC_TEST);
	
	DISABLE_INTR();
	
	// In 950 msec, a 100 msec timer should have expired exactly 9 times.
	return timer_err == TIMER_ERR_NONE && count == 9 ? true : false;
}


#else
void wowlTests(void) {}  // nop

#endif // end tests

