/*
	wowlAdc.c
	
	Module for interfacing with the internal ADC peripheral.
	
	2020-09-22  WHF  Created.
*/

#include "wowl.h"
#include "wowlSample.h"

#include <nrf_gpio.h>
#include <nrf_soc.h>

#include <drivers_nrf/saadc/nrf_drv_saadc.h>


//********************************  Constants  *******************************//
// VBAT must be the first entry as it is the only one used when submerged.
#define ICHAN_VBAT                                                       0u
static const nrf_saadc_channel_config_t adcConfigs[] = { 
	NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_VDD),  // VBAT
	NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN0), // VCHG
	NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN1), // ICHG
};

// Internal ADC reference.
#define VREF                                                           0.6f

// 12-bit ADC.
#define COUNTS_FULL_SCALE                                           4095.0f

// Voltage divider on Vchg.  Currently 1Meg over 1Meg, or one half, so we
//  multiply by two to compensate.
#define VCHG_DVDR                                                      2.0f

// Gain of the charging current for the LTC4079:
#define ICHG_GAIN                                                    250.0f

// Resistor used to set the charging current.
#define R_CHG                                                          5e3f


//*******************************  Module Data  ******************************//

// These must be consistent with [Vbat, surface_sample_index_t]:
static float CALIBRATIONS[] = {
	(1.0F / (1.0F/6.0F * COUNTS_FULL_SCALE / VREF)),    // Vbat
	(1.0F / (1.0F/6.0F * COUNTS_FULL_SCALE / VREF)) * VCHG_DVDR,    // Vchg
	(1.0F / (1.0F/6.0F * COUNTS_FULL_SCALE / VREF)) * ICHG_GAIN / R_CHG, // Ichg
};
static_assert(
		_countof(CALIBRATIONS) == _countof(adcConfigs),
		"Calibrations inconsistent with channel-count."
);

//***********************  Local Function Declarations  **********************//
static void saadcHandler(const nrf_drv_saadc_evt_t *pEvent);


//****************************  Global Functions  ****************************//
void wowlAdcInit(void)
{
	// Nop.
}	

void wowlAdcTriggerSurface(void)
{
	// Initialize the ADC module:
	int retCode = nrf_drv_saadc_init(NULL, saadcHandler);
	assert(retCode == NRF_SUCCESS);
				
	// Configure the channels.
	for (uint32_t iChan = 0u; iChan < _countof(adcConfigs) 
			&& retCode == NRF_SUCCESS; ++iChan) {
		retCode = nrf_drv_saadc_channel_init(
				iChan,        // entry in list
				&adcConfigs[iChan]
		);
		assert(retCode == NRF_SUCCESS);
	}			
		
	// Convert the channels:
	for (uint32_t iChan = 0u; iChan < _countof(adcConfigs) 
			&& retCode == NRF_SUCCESS; ++iChan) {
		int16_t value;

		retCode = nrf_drv_saadc_sample_convert(iChan, &value);		
		assert(retCode == NRF_SUCCESS);
		// Ok.  Convert to volts.
		//  (Internal gain (1/6), 
		//   12-bit resolution, 0.6 volt internal reference.
		const float meas = (float) value * CALIBRATIONS[iChan];
		
		// Submit:
		if (iChan == ICHAN_VBAT) {
			// Battery voltage is used both on the surface and under it.
			wowlSampleSubmitSubmerged(SI_BATTERY_VOLTAGE, meas);
		} else {
			// Other samples are only of interest when surfaced.
			//  The -1 accounts for the battery channel.
			wowlSampleSubmitSurface((surface_sample_index_t) (iChan-1u), meas);
		}
		
		// Cleanup:
		(void) nrf_drv_saadc_channel_uninit(iChan);
	}
	
	// Cleanup:
	(void) nrf_drv_saadc_uninit();

	// Get the CPU temperature, via the Soft Device:
	int32_t tempRaw;
	sd_temp_get(&tempRaw);
	wowlSampleSubmitSurface(SS_CPU_TEMP, (float) tempRaw * 0.25f);
}

// When submerged, we only care about the battery voltage.
//  TODO: what is the power consumption impact of doing this 
//  conversion synchronously?
void wowlAdcTriggerSubmerged(void)
{
	// Initialize the ADC module:
	int retCode = nrf_drv_saadc_init(NULL, saadcHandler);
	assert(retCode == NRF_SUCCESS);
				
	// Configure the channel.
	retCode = nrf_drv_saadc_channel_init(
			0,        // entry in list
			&adcConfigs[ICHAN_VBAT]
	);
	assert(retCode == NRF_SUCCESS);
		
	// Convert the channel:
	int16_t value;

	retCode = nrf_drv_saadc_sample_convert(ICHAN_VBAT, &value);		
	assert(retCode == NRF_SUCCESS);
	
	// Ok.  Convert to volts.
	//  (Internal gain (1/6), 
	//   12-bit resolution, 0.6 volt internal reference.
	const float meas = (float) value * CALIBRATIONS[ICHAN_VBAT];
		
	// Submit:
	wowlSampleSubmitSubmerged(SI_BATTERY_VOLTAGE, meas);
	
	// Cleanup the channel:
	(void) nrf_drv_saadc_channel_uninit(0);
	
	// Cleanup the ADC.
	(void) nrf_drv_saadc_uninit();	
}


//***********************  Local Function Definitions  ***********************//
static void saadcHandler(const nrf_drv_saadc_evt_t *pEvent)
{
	// A dummy handler.
	(void) pEvent;
}

