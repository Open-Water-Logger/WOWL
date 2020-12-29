/*
	ads1120.c
	
	Driver for the ADS1120, a 16-bit, low power sigma-delta ADC with integrated
	PGA and excitation sources from Texas Instruments.
	
	2020-10-06  WHF  Created, from ads1120.c.
*/

#include "ads1120.h"

#include <assert.h>
#include <string.h>

//********************************  Constants  *******************************//

/////  Commands  /////
#define CMD_POWERDOWN                                                 0x02u
#define CMD_START_SYNC                                                0x08u
#define CMD_RESET                                                     0x06u
#define CMD_NOP                                                       0x00u
#define CMD_RDATA                                                     0x10u
#define CMD_RREG(addr, nBytes)         (0x20u | ((addr)&3)<<2 | nBytes-1&3)
#define CMD_WREG(addr, nBytes)         (0x40u | ((addr)&3)<<2 | nBytes-1&3)


/////  Registers  /////
#define REG_0                                                         0x00u
#define REG_1                                                         0x01u
#define REG_2                                                         0x02u
#define REG_3                                                         0x03u


/////  Register Values  /////
#define REG_0_MUX(mux)                                     (((mux)&0xF)<<4)
#define REG_0_GAIN(gain)                                    (((gain)&7)<<1)
#define REG_0_PGA_BYPASS                                              0x01u

#define REG_1_DR(dr)                                          (((dr)&7)<<5)
#define REG_1_MODE(mode)                                    (((mode)&3)<<3)
#define REG_1_CM_SINGLE                                               0x00u
#define REG_1_CM_CONTINUOUS                                           0x04u
#define REG_1_TS_DISABLE                                              0x00u
#define REG_1_TS_ENABLE                                               0x02u
#define REG_1_BCS_ON                                                  0x01u

#define REG_2_VREF(vref)                                    (((vref)&3)<<6)
#define REG_2_REJECTION(rej)                                 (((rej)&3)<<4)
#define REG_2_PSW_OPEN                                                0x00u
#define REG_2_PSW_CLOSE                                               0x08u
#define REG_2_IDAC(current)                                   ((current)&7)

#define REG_3_IMUX_1(imux)                                  (((imux)&7)<<5)
#define REG_3_IMUX_2(imux)                                  (((imux)&7)<<2)
#define REG_3_DRDY_ON_DOUT                                            0x02u



//**********************************  Types  *********************************//
typedef struct tag_inst_data {
	ads1120_mux_t mux;
	ads1120_gain_t gain;
	ads1120_dr_t dr;
	ads1120_mode_t mode;
	ads1120_iexec_t iexec;
	ads1120_imux_t
		idac1,
		idac2;
	ads1120_ref_t vref;
	ads1120_rejection_t rej;

	bool tempSensorOn;	

} INST_DATA;

//*******************************  Module Data  ******************************//
// Storage for instance specific data elements.
static INST_DATA instData[ADS1120_MAX_INSTANCES];

//********************************  Functions  *******************************//
// Call on power up to properly reset the device.  Assumes that
//   initialization of the host processor has already occurred.
void ads1120_init(uint8_t instance)
{
	uint8_t cmd;
	assert(instance < ADS1120_MAX_INSTANCES);
	
	// Set data to reset values:
	(void) memset(&instData[instance], 0, sizeof(instData[instance]));
	
	// Send the reset command.
	cmd = CMD_RESET;
	ads1120_impl_spi(instance, &cmd, NULL, 1);
	
	// Wait 50 usec + 32 / 4.096 MHz, which we round up to 60 usec.
	ads1120_impl_wait_usec(60);	
}

// Set the multiplexer for conversions.
void ads1120_set_MUX(uint8_t instance, ads1120_mux_t mux)
{
	assert(instance < ADS1120_MAX_INSTANCES);

	instData[instance].mux = mux;
	
	const uint8_t cmd[] = {
		CMD_WREG(REG_0, 1),   // one byte
		REG_0_MUX(mux) | REG_0_GAIN(instData[instance].gain)
	};
	
	// Write via SPI:
	ads1120_impl_spi(
			instance,
			cmd,
			NULL,
			sizeof(cmd)
	);
}

// Set the gain.
void ads1120_set_GAIN(uint8_t instance, ads1120_gain_t setting)
{
	assert(instance < ADS1120_MAX_INSTANCES);

	instData[instance].gain = setting;
	
	const uint8_t cmd[] = {
		CMD_WREG(REG_0, 1),   // one byte
		REG_0_MUX(instData[instance].mux) | REG_0_GAIN(instData[instance].gain)
	};
	// Write using the SPI:
	ads1120_impl_spi(
			instance,
			cmd,
			NULL,
			sizeof(cmd)
	);
}

// Set the data rate and operating mode.
void ads1120_set_DR(uint8_t instance, ads1120_mode_t mode, ads1120_dr_t rate)
{
	assert(instance < ADS1120_MAX_INSTANCES);

	instData[instance].dr = rate;
	instData[instance].mode = mode;
	
	const uint8_t cmd[] = {
		CMD_WREG(REG_1, 1),   // one byte
		REG_1_DR(instData[instance].dr) | REG_1_MODE(mode) | REG_1_CM_SINGLE
				| REG_1_TS_ENABLE * instData[instance].tempSensorOn
	};
	
	// Write using the SPI:
	ads1120_impl_spi(
			instance,
			cmd,
			NULL,
			sizeof(cmd)
	);
}

// Setup excitation currents and output pins.
void ads1120_set_IEXEC(uint8_t instance, ads1120_iexec_t magnitude, 
		ads1120_imux_t idac1, ads1120_imux_t idac2) 
{
	assert(instance < ADS1120_MAX_INSTANCES);
	
	// Cache data:
	instData[instance].iexec = magnitude;
	instData[instance].idac1 = idac1;
	instData[instance].idac2 = idac2;
	
	const uint8_t cmd[] = {
		CMD_WREG(REG_2, 2),   // two bytes (REG 2 and 3)
		REG_2_VREF(instData[instance].vref) | REG_2_IDAC(magnitude),
		REG_3_IMUX_1(idac1) | REG_3_IMUX_2(idac2)
	};
	// Send to the device:
	ads1120_impl_spi(
			instance,
			cmd,
			NULL,
			sizeof(cmd)
	);
}

// Set the voltage reference used for conversions.
void ads1120_set_REF(uint8_t instance, ads1120_ref_t ref)
{
	assert(instance < ADS1120_MAX_INSTANCES);
	
	instData[instance].vref = ref;
	
	const uint8_t cmd[] = {
		CMD_WREG(REG_2, 1),   // one byte
		REG_2_VREF(instData[instance].vref) 
				| REG_2_IDAC(instData[instance].iexec),
	};

	// Send to the device:
	ads1120_impl_spi(
			instance,
			cmd,
			NULL,
			sizeof(cmd)
	);
}

// Returns true if the ADC is in temperature mode.
bool ads1120_get_temperature_mode(uint8_t instance)
{
	assert(instance < ADS1120_MAX_INSTANCES);

	return instData[instance].tempSensorOn;
}

void ads1120_set_temperature_mode(uint8_t instance, bool onoff)
{
	assert(instance < ADS1120_MAX_INSTANCES);

	instData[instance].tempSensorOn = onoff;
	
	const uint8_t cmd[] = {
		CMD_WREG(REG_1, 1),   // one byte
		REG_1_DR(instData[instance].dr) 
				| REG_1_MODE(instData[instance].mode) | REG_1_CM_SINGLE
				| REG_1_TS_ENABLE * instData[instance].tempSensorOn
	};
	
	// Write using the SPI:
	ads1120_impl_spi(
			instance,
			cmd,
			NULL,
			sizeof(cmd)
	);
}

// Begin continuous sampling.  nDRDY will go low after each conversion;
//   detection of that fact is outside the scope of this driver.
void ads1120_begin_continuous_conversion(uint8_t instance)
{
	assert(instance < ADS1120_MAX_INSTANCES);
	
	const uint8_t cmd[] = {
		CMD_WREG(REG_1, 1),   // one byte
		REG_1_DR(instData[instance].dr) | REG_1_MODE(instData[instance].mode)
				| REG_1_CM_CONTINUOUS 
				| REG_1_TS_ENABLE * instData[instance].tempSensorOn,
		CMD_START_SYNC        // start conversions
	};
	
	// Write using the SPI:
	ads1120_impl_spi(
			instance,
			cmd,
			NULL,
			sizeof(cmd)
	);
}

// Assumes a new sample is ready.
int16_t ads1120_retrieve_sample(uint8_t instance)
{
	assert(instance < ADS1120_MAX_INSTANCES);

	// Send two NOPs to clock out the data:	
	uint8_t cmd[] = {
		CMD_NOP,
		CMD_NOP
	};
	uint8_t incoming[sizeof(cmd)];  // receive the 16-bit sample here.

	// Retrieve the sample;
	ads1120_impl_spi(
			instance,
			cmd,
			incoming,
			sizeof(cmd)
	);
	
	// Create the return value.  Note the raw value is in MSB, two's complement.
	return (int16_t) ((uint16_t) incoming[0] << 8 | (uint16_t) incoming[1]);
}

// Stop continuous sampling.  The part will enter the low power state, 
//  but the reference will not be disabled.
void ads1120_end_continuous_conversion(uint8_t instance)
{
	assert(instance < ADS1120_MAX_INSTANCES);

	// Update REG 1, clearing the continuous mode bit.
	const uint8_t cmd[] = {
		CMD_WREG(REG_1, 1),   // one byte
		REG_1_DR(instData[instance].dr) | REG_1_MODE(instData[instance].mode)
				| REG_1_CM_SINGLE
				| REG_1_TS_ENABLE * instData[instance].tempSensorOn
	};
	// Write using the SPI:
	ads1120_impl_spi(
			instance,
			cmd,
			NULL,
			sizeof(cmd)
	);
}
	
// Trigger a single conversion using START.  nDRDY will notify of the response.
void ads1120_trigger_single_conversion(uint8_t instance)
{
	assert(instance < ADS1120_MAX_INSTANCES);

	const uint8_t cmd[] = {
		CMD_START_SYNC        // start conversions
	};
	
	// Write using the SPI:
	ads1120_impl_spi(
			instance,
			cmd,
			NULL,
			sizeof(cmd)
	);
}

// Put the device in its lowest power state.
void ads1120_powerdown(uint8_t instance)
{
	assert(instance < ADS1120_MAX_INSTANCES);

	const uint8_t cmd[] = {
		CMD_POWERDOWN
	};
	
	// Write using the SPI:
	ads1120_impl_spi(
			instance,
			cmd,
			NULL,
			sizeof(cmd)
	);
}

