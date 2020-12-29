/*
	wowlExtAdc.c

	Implementation of the External ADC module for the WOWL.

	2020-09-22  WHF  Created.
*/

#include "wowl.h"
#include "wowlSample.h"

#include <ads1120.h>
#include <lookup.h>

#include <nrf_gpio.h>

#include <gpiote/nrf_drv_gpiote.h>
#include <spi_master/nrf_drv_spi.h>

//********************************  Constants  *******************************//
#define NTC_GAIN_CHOICE                                      ADS1120_GAIN_1
#define RTD_GAIN_CHOICE                                     ADS1120_GAIN_64

#define NTC_GAIN                          ((float) (1u << NTC_GAIN_CHOICE))
#define RTD_GAIN                          ((float) (1u << RTD_GAIN_CHOICE))

// R25 on the Rev. A schematic: 12 kohms.  Along with the excitation 
//  current, this resistor forms the reference voltage.
#define R_REF_RTD                                                     12e3f

// R24 on the Rev. A schematic: 12 kohms.
#define R_REF_NTC                                                     12e3f


//*******************************  Module Data  ******************************//
// These must agree with the instance selections (WOWL_ADS1120_INST_*) in
//   wowl.h.
static const uint8_t chipSelectPins[] = {
	PIN_RTD_nCS,
	PIN_NTC_nCS
};

static const uint8_t dataReadyPins[] = {
	PIN_RTD_DRDY,
	PIN_NTC_DRDY
};

static const nrf_drv_spi_t spiInstance0 = NRF_DRV_SPI_INSTANCE(0);

static LOOKUP_TABLE_F ntcLookup, rtdLookup;

static bool ntcReady, rtdReady;

//***********************  Local Function Declarations  **********************//
static void initNtcAdc(void);
static void initRtdAdc(void);

static void rtdReadyCallback(
		nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t pol);
static void ntcReadyCallback(
		nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t pol);


//****************************  Global Functions  ****************************//

void wowlExtAdcInit(void)
{
	// Setup lookup table objects.
	extern const float ga3k3a1a[];
	extern const unsigned ga3k3a1a_size_bytes;
	
	// Number of entries in each column:
	const uint32_t ntcPerCol = ga3k3a1a_size_bytes / sizeof(float) / 2u;
	
	// First input value.  As NTCs are backwards, is the last resistance
	//  value.
	ntcLookup.pX = &ga3k3a1a[1u + (ntcPerCol - 1u) * 2u];
	// First output (temperature) value.
	ntcLookup.pY = &ga3k3a1a[0u + (ntcPerCol - 1u) * 2u];
	ntcLookup.n  = ntcPerCol;
	// Increment:
	ntcLookup.xStride = -2;
	ntcLookup.yStride = -2;
	
	// For platinum RTD:
	extern const float prt_32208551[];
	extern const unsigned prt_32208551_size_bytes;
	// Number of entries in each column:
	const uint32_t rtdPerCol = prt_32208551_size_bytes / sizeof(float) / 2u;
	
	// First input (resistance) value.
	rtdLookup.pX = &prt_32208551[1u];
	// First output (temperature) value.
	rtdLookup.pY = &prt_32208551[0u];
	rtdLookup.n  = rtdPerCol;
	// Increment:
	rtdLookup.xStride = 2;
	rtdLookup.yStride = 2;
	
	
	// SPI configuration data structure:
	static const nrf_drv_spi_config_t spiConfig = {
		.sck_pin      = PIN_ADC_SCLK,       
		.mosi_pin     = PIN_ADC_DIN,      
		.miso_pin     = PIN_ADC_DOUT,       
		.ss_pin       = NRF_DRV_SPI_PIN_NOT_USED, // we use manual control       
		.irq_priority = WOWL_SPI_PRIO,
		.orc          = 0x0,                           
		.frequency    = NRF_DRV_SPI_FREQ_4M, // ADS1120 max is 6 MHz
		// Clock normally low, valid on falling edge
		.mode         = NRF_DRV_SPI_MODE_1,    // CPOL 0, CPHA 1             
		.bit_order    = NRF_DRV_SPI_BIT_ORDER_MSB_FIRST,
	};
	
	for (int iCS = 0; iCS < _countof(chipSelectPins); ++iCS) {
		// Set the chip select pins to outputs, high:
		nrf_gpio_pin_set(chipSelectPins[iCS]);
		nrf_gpio_cfg_output(chipSelectPins[iCS]);
		// Set the data ready pin as an input:
		nrf_gpio_cfg_input(dataReadyPins[iCS], false); // no pullup
	}

	int result = nrf_drv_spi_init(&spiInstance0, &spiConfig, NULL);
	
	if (result == NRF_SUCCESS) {
		// Ok.
	} else if (result == NRF_ERROR_INVALID_STATE) {
		// The SPI module is already initialized.  Ignore.
		result = NRF_SUCCESS;
	} else {
		assert(false);
		// Set the flag TODO
		//wowlMainSetStatus(WOWL_STATUS_HUMIDITY_INIT_FAIL);
	}

	if (result == NRF_SUCCESS) {
		initNtcAdc();
		initRtdAdc();
		
		// Setup GPIO interrupts.
		wowlInitSetupGpioInterrupt(PIN_RTD_DRDY, rtdReadyCallback);
		wowlInitSetupGpioInterrupt(PIN_NTC_DRDY, ntcReadyCallback);

	} else {
		// SPI init failed.
	}		
}

// Trigger the main sensor readings.
void wowlExtAdcTriggerMain(void)
{
	// NTC
	// Turn off the internal temperature sensor:
	ads1120_set_temperature_mode(WOWL_ADS1120_INST_NTC, false);
		
	// Trigger a conversion.
	ads1120_trigger_single_conversion(WOWL_ADS1120_INST_NTC);
	
	// RTD
	// Turn off the internal temperature sensor:
	ads1120_set_temperature_mode(WOWL_ADS1120_INST_RTD, false);
		
	// Trigger a conversion.
	ads1120_trigger_single_conversion(WOWL_ADS1120_INST_RTD);
}

// Trigger the internal temperature sensor readings.
void wowlExtAdcTriggerTempSense(void)
{
	// NTC
	// Turn ON the internal temperature sensor:
	ads1120_set_temperature_mode(WOWL_ADS1120_INST_NTC, true);
		
	// Trigger a conversion.
	ads1120_trigger_single_conversion(WOWL_ADS1120_INST_NTC);
	
	// RTD
	// Turn ON the internal temperature sensor:
	ads1120_set_temperature_mode(WOWL_ADS1120_INST_RTD, true);
		
	// Trigger a conversion.
	ads1120_trigger_single_conversion(WOWL_ADS1120_INST_RTD);
}

void wowlExtAdcVisit(void)
{
	if (rtdReady) {
		// Read the output.
		const int16_t raw = ads1120_retrieve_sample(WOWL_ADS1120_INST_RTD);

		if (ads1120_get_temperature_mode(WOWL_ADS1120_INST_RTD)) {
			// We just read the internal temperature sensor.
			// Convert to a temperature.
			const float T = ADS1120_TEMP_SENSOR_CONVERT(raw);
			// Submit.
			wowlSampleSubmit(SI_TEMP_INT_RTD_C, T);
			
		} else {
			// We just read the voltage across the RTD.
			// Convert to a resistance.
			const float 
				R = (float)raw * (R_REF_RTD / RTD_GAIN / (float) (1 << 15)),
				// Lookup the resistance to convert to temperature.
				T = lookupf(R, &rtdLookup);
	
			// Submit.
			wowlSampleSubmit(SI_TEMP_RTD_C, T);
		}
		
		rtdReady = false;
	} else {
		// Not ready.
	}
	
	if (ntcReady) {
		// Read the output.
		const int16_t raw = ads1120_retrieve_sample(WOWL_ADS1120_INST_NTC);
		
		if (ads1120_get_temperature_mode(WOWL_ADS1120_INST_NTC)) {
			// We just read the internal temperature sensor.
			// Convert to a temperature.
			const float T = ADS1120_TEMP_SENSOR_CONVERT(raw);
			// Submit.
			wowlSampleSubmit(SI_TEMP_INT_NTC_C, T);
			
		} else {
			// Convert to a resistance.
			const float 
				R = (float)raw * (R_REF_NTC / NTC_GAIN / (float) (1 << 15)),
				// Lookup the resistance to convert to temperature.
				T = lookupf(R, &ntcLookup);
	
			// Submit.
			wowlSampleSubmit(SI_TEMP_NTC_C, T);
		}
		
		ntcReady = false;
	}
}

void wowlExtAdcPowerDown(void)
{
	ads1120_powerdown(WOWL_ADS1120_INST_NTC);
	ads1120_powerdown(WOWL_ADS1120_INST_RTD);
}

void wowlExtAdcWakeup(void)
{
	// To wake up, we simply trigger conversions.	
	ads1120_trigger_single_conversion(WOWL_ADS1120_INST_NTC);
	ads1120_trigger_single_conversion(WOWL_ADS1120_INST_RTD);
}

//**********************  ADS1120 Driver Implementation  *********************//
// A calibrated delay function, used for initialization.
void ads1120_impl_wait_usec(uint32_t nUsec)
{
	WAIT_USEC(nUsec);
}

// Synchronous SPI interchange.  Assumes the operation cannot fail.
//  If pIn is NULL, then the read value is not required and may be discarded.
void ads1120_impl_spi(
		uint8_t instance, const uint8_t *pOut, uint8_t *pIn, uint8_t n)
{
	// Select the part:
	nrf_gpio_pin_clear(chipSelectPins[instance]);
	
	// Perform the SPI transfer synchronously.
	int result = nrf_drv_spi_transfer(
			&spiInstance0,
			pOut,
			n,
			pIn,
			pIn == NULL ? 0 : n
	);	
	assert(result == 0);
		
	// Deselect the part:
	nrf_gpio_pin_set(chipSelectPins[instance]);
}

//***********************  Local Function Definitions  ***********************//
static void initNtcAdc(void)
{
	// Initialize the device:
	ads1120_init(WOWL_ADS1120_INST_NTC);
	
	// Setup the gain and other features.
	// Set the multiplexer for conversions.
	ads1120_set_MUX(WOWL_ADS1120_INST_NTC, ADS1120_MUX_AIN1_AIN2);

	// Set the gain.
	ads1120_set_GAIN(WOWL_ADS1120_INST_NTC, NTC_GAIN_CHOICE);

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
}

static void initRtdAdc(void)
{
	// Initialize the device:
	ads1120_init(WOWL_ADS1120_INST_RTD);
	
	// Setup the gain and other features.
	// Set the multiplexer for conversions.
	ads1120_set_MUX(WOWL_ADS1120_INST_RTD, ADS1120_MUX_AIN1_AIN2);

	// Set the gain.
	ads1120_set_GAIN(WOWL_ADS1120_INST_RTD, RTD_GAIN_CHOICE);

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
}

static void rtdReadyCallback(
		nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t pol)
{
	rtdReady = true;
}
static void ntcReadyCallback(
		nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t pol)
{
	ntcReady = true;
}

