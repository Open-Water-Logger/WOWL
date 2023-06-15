/*
	wowlSample.h
	
	Declarations for sub-modules of the Sample module.  This separate header
	limits their visibility.
	
	2020-09-22  WHF  Created.
*/

#ifndef __WOHL_SAMPLE_H__
#define __WOHL_SAMPLE_H__

//********************************  Functions  *******************************//
// Sample the internal ADC channels.
void wowlAdcTriggerSurface(void);
void wowlAdcTriggerSubmerged(void);

void wowlPressInit(void);
void wowlPressVisit(void);
void wowlPressTrigger(void);

void wowlTempInit(void);
void wowlTempVisit(void);
// Trigger sample.  Will tie up the I2C bus.
void wowlTempTrigger(void);

// Submodules use to submit data for aggregation.
void wowlSampleSubmitSubmerged(submerged_sample_index_t idx, float value);
void wowlSampleSubmitSurface(surface_sample_index_t idx, float value);

#endif /* __WOHL_SAMPLE_H__ */


