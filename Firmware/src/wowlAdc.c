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

static const nrf_saadc_channel_config_t adcConfigs[] = { 
	NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN0), // VCHG
	NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN1), // ICHG
	NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN3), // PDC
	NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_VDD),  // VBAT
};

// Internal ADC reference.
#define VREF                                                           0.6f

// 12-bit ADC.
#define COUNTS_FULL_SCALE                                           4095.0f

// Gain of the charging current for the LTC4079:
#define ICHG_GAIN                                                    250.0f

// Resistor used to set the charging current.
#define R_CHG                                                          5e3f

// Resistor ladder used for the PDC scaling.
#define PDC_R_LADDER                                ((10e3f+100.0f)/100.0f)

//*******************************  Module Data  ******************************//

// These must be consistent with sample_index_t:
static float CALIBRATIONS[] = {
	(1.0F / (1.0F/6.0F * COUNTS_FULL_SCALE / VREF)),
	(1.0F / (1.0F/6.0F * COUNTS_FULL_SCALE / VREF)) * ICHG_GAIN / R_CHG,
	(1.0F / (1.0F/6.0F * COUNTS_FULL_SCALE / VREF)) * PDC_R_LADDER,
	(1.0F / (1.0F/6.0F * COUNTS_FULL_SCALE / VREF)),
};

//***********************  Local Function Declarations  **********************//
static void saadcHandler(const nrf_drv_saadc_evt_t *pEvent);


//****************************  Global Functions  ****************************//
void wowlAdcInit(void)
{	
}	


void wowlAdcTrigger(void)
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
		wowlSampleSubmit((sample_index_t) iChan, meas);
		
		// Cleanup:
		(void) nrf_drv_saadc_channel_uninit(iChan);
	}
	
	// Cleanup:
	(void) nrf_drv_saadc_uninit();

	// Get the CPU temperature, via the Soft Device:
	int32_t tempRaw;
	sd_temp_get(&tempRaw);
	wowlSampleSubmit(SI_CPU_TEMP, (float) tempRaw * 0.25f);

}

//***********************  Local Function Definitions  ***********************//
static void saadcHandler(const nrf_drv_saadc_evt_t *pEvent)
{
	// A dummy handler.
	(void) pEvent;
}

