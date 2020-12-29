/*
	ads1120.h
	
	Declarations for the driver for the ADS1120, a 24-bit, low power sigma-delta
	ADC with integrated PGA and excitation sources.
	
	2018-06-18  WHF  Created.
*/

#ifndef __ADS1120_H__
#define __ADS1120_H__

#include <stdbool.h>
#include <stdint.h>

//********************************  Constants  *******************************//
// Maximum number of supported instances.  Can be overridden with project
//  level defines.
#ifndef ADS1120_MAX_INSTANCES
#	define ADS1120_MAX_INSTANCES                                          2
#endif

// Macro to convert raw sample to Celsius.
#define ADS1120_TEMP_SENSOR_CONVERT(raw)                                  \
		((float)(int16_t) (raw) * (0.03125f/4.0f))

//**********************************  Types  *********************************//
// Input multiplexer setting.
typedef enum tag_ads1120_mux {
	ADS1120_MUX_AIN0_AIN1,
	ADS1120_MUX_AIN0_AIN2,
	ADS1120_MUX_AIN0_AIN3,
	ADS1120_MUX_AIN1_AIN2,
	ADS1120_MUX_AIN1_AIN3,
	ADS1120_MUX_AIN2_AIN3,
	ADS1120_MUX_AIN1_AIN0,
	ADS1120_MUX_AIN3_AIN2,
	ADS1120_MUX_AIN0_AVSS,
	ADS1120_MUX_AIN1_AVSS,
	ADS1120_MUX_AIN2_AVSS,
	ADS1120_MUX_AIN3_AVSS,
	ADS1120_MUX_VREF_DIV4,
	ADS1120_MUX_AVDD_DIV4,
	ADS1120_MUX_SHORT,
	ADS1120_MUX_RESERVED
} ads1120_mux_t;

// Gain setting.
typedef enum tag_ads1120_gain {
	ADS1120_GAIN_1,
	ADS1120_GAIN_2,
	ADS1120_GAIN_4,
	ADS1120_GAIN_8,
	ADS1120_GAIN_16,
	ADS1120_GAIN_32,
	ADS1120_GAIN_64,
	ADS1120_GAIN_128
} ads1120_gain_t;

// Data rate specification, in Samples Per Second.
//  Note that these values are for Normal mode.  Duty-cycle mode will be 
//  one-fourth these values, and Turbo mode twice.
typedef enum tag_ads1120_dr {
	ADS1120_DR_20,
	ADS1120_DR_45,
	ADS1120_DR_90,
	ADS1120_DR_175,
	ADS1120_DR_330,
	ADS1120_DR_600,
	ADS1120_DR_1000
} ads1120_dr_t;

// Operating mode.
typedef enum tag_ads1120_mode {
	ADS1120_MODE_NORMAL,
	ADS1120_MODE_DUTY_CYCLE,
	ADS1120_MODE_TURBO
} ads1120_mode_t;

// Excitation current magnitude.
typedef enum tag_ads1120_iexec {
	ADS1120_IEXEC_UA_0,
	ADS1120_IEXEC_UA_RESERVED,
	ADS1120_IEXEC_UA_50,
	ADS1120_IEXEC_UA_100,
	ADS1120_IEXEC_UA_250,
	ADS1120_IEXEC_UA_500,
	ADS1120_IEXEC_UA_1000,
	ADS1120_IEXEC_UA_1500	
} ads1120_iexec_t;

// Excitation current mux.  May be used for either IDAC1 or IDAC2.
typedef enum tag_ads1120_imux {
	ADS1120_IMUX_NONE,
	ADS1120_IMUX_AIN0,
	ADS1120_IMUX_AIN1,
	ADS1120_IMUX_AIN2,
	ADS1120_IMUX_AIN3,
	ADS1120_IMUX_REFP0,
	ADS1120_IMUX_REFN0,
	ADS1120_IMUX_RESERVED
} ads1120_imux_t;

// Voltage reference for conversions.
typedef enum tag_ads1120_ref {
	ADS1120_REF_INT,    // internal 2.048 volts
	ADS1120_REF_REFP0_REFN0,
	ADS1120_REF_REFP1_REFN1,
	ADS1120_REF_AVDD,
} ads1120_ref_t;

// AC line rejection.
typedef enum tag_ads1120_rejection {
	ADS1120_REJECT_NONE,
	ADS1120_REJECT_50_60_HZ,
	ADS1120_REJECT_50_HZ,
	ADS1120_REJECT_60_HZ,
} ads1120_rejection_t;	


//********************************  Functions  *******************************//
// Call on power up to properly reset the device.  Assumes that
//   initialization of the host processor has already occurred.
void ads1120_init(uint8_t instance);

// Set the multiplexer for conversions.
void ads1120_set_MUX(uint8_t instance, ads1120_mux_t mux);

// Set the gain.
void ads1120_set_GAIN(uint8_t instance, ads1120_gain_t setting);

// Set the data rate and operating mode.
void ads1120_set_DR(uint8_t instance, ads1120_mode_t mode, ads1120_dr_t rate);

// Setup excitation currents and output pins.
void ads1120_set_IEXEC(uint8_t instance, ads1120_iexec_t magnitude, 
		ads1120_imux_t idac1, ads1120_imux_t idac2); 

// Set the voltage reference used for conversions.
void ads1120_set_REF(uint8_t instance, ads1120_ref_t ref);

// Returns true if the ADC is in temperature mode.
bool ads1120_get_temperature_mode(uint8_t instance);

// Set temperature mode on to read the internal temperature sensor.
void ads1120_set_temperature_mode(uint8_t instance, bool onoff);

// Begin continuous sampling.  nDRDY will go low after each conversion;
//   detection of that fact is outside the scope of this driver.
void ads1120_begin_continuous_conversion(uint8_t instance);

// Assumes a new sample is ready.
int16_t ads1120_retrieve_sample(uint8_t instance);

// Stop continuous sampling.  The part will enter the low power state, 
//  but the reference will not be disabled.
void ads1120_end_continuous_conversion(uint8_t instance);

// Trigger a single conversion using START.  nDRDY will notify of the response.
void ads1120_trigger_single_conversion(uint8_t instance);

// Power down the device.  This shuts down all analog compenents, opens the
//  low side switch, turns off both IDACs, but holds all register values.
// Starting the next conversion will restore all analog compoents.
void ads1120_powerdown(uint8_t instance);

//***************************  Implementation API  ***************************//
//  These functions must be implemented by the application for the specific
//   platform.

// A calibrated delay function, used for initialization.
void ads1120_impl_wait_usec(uint32_t nUsec);

// Synchronous SPI interchange.  Assumes the operation cannot fail.
//  If pIn is NULL, then the read value is not required and may be discarded.
void ads1120_impl_spi(
		uint8_t instance, const uint8_t *pOut, uint8_t *pIn, uint8_t n);
		
#endif /* __ADS1120_H__ */

