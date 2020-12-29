/*
	wowl.h
	
	Declarations for the WOWL sensor.
	
	2020-09-22  WHF  Created.
*/

#ifndef __WOWL_H__
#define __WOWL_H__

#include "wowlCodes.h"

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include <cmsis_compiler.h>

#include <nordic_common.h>
#include <nrf_gpiote.h>

//********************************  Constants  *******************************//

// Set to 1 to enable unit tests.
#define DO_TESTS                                                          0

// Set to 1 to enable submerged testing mode, where the pressure offset
//   parameter can be set to simulate submersion, and the radio does not
//   shut off.
#define DO_SUBMERSION_TEST                                                0


// Number of milliseconds in a second.
#define MSEC_PER_SEC                                                  1000u


/////  Pin Allocations  /////
// Pin numbers for functions.  Pins on the NRF52 are numbered 0 through 31.
//  See the schematic 1010309-900-0011 for the net names.

#define PIN_AN_VCHG                                                       2
#define PIN_AN_ICHG                                                       3
#define PIN_MOD_EN                                                        4
#define PIN_AN_PDC                                                        5
#define PIN_NTC_nCS                                                       6
#define PIN_NTC_DRDY                                                      7
#define PIN_MEM_nCS                                                       8
#define PIN_MEM_SDO                                                       9
#define PIN_MEM_SCLK                                                     10
#define PIN_MEM_SDI                                                      11
#define PIN_RTD_nCS                                                      12
#define PIN_ADC_SCLK                                                     13
#define PIN_ADC_DIN                                                      14
#define PIN_ADC_DOUT                                                     15
#define PIN_RTD_DRDY                                                     16
#define PIN_LED_RED                                                      17
#define PIN_reserved_swo                                                 18
#define PIN_LED_YEL                                                      19
#define PIN_LED_GRN                                                      20
#define PIN_reserved_reset                                               21
#define PIN_reserved_22                                                  22
#define PIN_MOD_CFG                                                      23
#define PIN_nCHRG                                                        24
#define PIN_TP21                                                         25
#define PIN_TP20                                                         26
#define PIN_TP19                                                         27
#define PIN_TP27                                                         28
#define PIN_reserved_29                                                  29
#define PIN_SDA                                                          30
#define PIN_SCL                                                          31


// The target clock rate of the processor.
#define CPU_CLOCK_HZ                                            64'000'000u

/////  Application Timer Constants  /////
// Frequency of the 'app timer', including prescaling.
#define APP_TIMER_FREQ_HZ                                            32768U
// Value of the RTC1 PRESCALER register.
#define APP_TIMER_PRESCALER												  0

// Maximum number of simultaneously created timers.
#define APP_TIMER_MAX_TIMERS											  6
// Size of timer operation queues.
#define APP_TIMER_OP_QUEUE_SIZE											  4


// Size of Flash pages on the nRF52832.
#define FLASH_PAGE_SIZE                                               4096U


/////  Event Codes  /////
typedef enum tag_wowl_event {
	EVENT_INIT_COMPLETE,
	EVENT_SURFACE,
	EVENT_SUBMERGE,
	EVENT_BATTERY_LOW,
	EVENT_BATTERY_CHARGED,
	EVENT_TRIGGER,
	EVENT_SAMPLE_READY,
	EVENT_WRITE_DONE,
	EVENT_DOWNLOAD,
	EVENT_DOWNLOAD_DONE,
	EVENT_ERASE,
	EVENT_ERASE_DONE,
} wowl_event_t;

typedef enum tag_wowl_state {
	STATE_INIT,
	STATE_SURFACE,
	STATE_SURFACE_SAMPLE,
	STATE_LIMP,
	STATE_UNDER_SLEEP,
	STATE_UNDER_SAMPLE,
	STATE_UNDER_WRITE,
	STATE_DOWNLOAD,
	STATE_ERASE,
	
	WOWL_N_STATES
} wowl_state_t;

typedef enum sample_index_t {
	SI_VCHG,
	SI_ICHG,
	SI_PDC,
	SI_BATTERY_VOLTAGE,
	SI_CPU_TEMP,
	SI_TEMP_NTC_C,
	SI_TEMP_RTD_C,
	SI_TEMP_SI_C,
	SI_PRESS_MBAR,
	SI_PRESS_TEMP_C,
	SI_TEMP_INT_NTC_C,
	SI_TEMP_INT_RTD_C,
	
	N_SI
} sample_index_t;

typedef enum tag_reprogram_err {
	WOWL_RE_NONE,
	WOWL_RE_SIZE_MISMATCH,
	WOWL_RE_CRC_ERROR,
	WOWL_RE_STACK_PLACEMENT
} wowl_reprogram_err_t;


/////  Status Flags  /////
#define WOWL_STATUS_CHARGING                                    0x00000001u

#define WOWL_STATUS_FIRMWARE_OPERATION_COMPLETE                 0x00000100u
#define WOWL_STATUS_FIRMWARE_OPERATION_FAILURE                  0x00000200u

#define WOWL_STATUS_CONNECTED                                   0x00010000u
#define WOWL_STATUS_ADVERTISING                                 0x00020000u

#define WOWL_STATUS_LAST_COMMAND_UNSUPPORTED                    0x00800000u

#define WOWL_STATUS_FLASH_FAIL                                  0x01000000u
#define WOWL_STATUS_SI7051_FAIL                                 0x02000000u
#define WOWL_STATUS_SPURIOUS_I2C_EVENT                          0x04000000u
#define WOWL_STATUS_PRESS_FAIL                                  0x08000000u


#define WOWL_STATUS_                                            0x00000000u


/////  Interrupt Priorities  /////
// The nRF52832 CM-4F instance has 3 priority bits.  Lower is higher priority.
//  The Soft Device reserves IRQ priorities 0, 1, 4, and 5, 
//  leaving 2, 3, 6, and 7 for use by the application.
#define WOWL_SPI_PRIO                                                    7u
#define WOWL_I2C_PRIO                                                    7u


/////  Timer Allocations  /////
// Used by the BLE SoftDevice and not available for the application.
#define WOWL_TIMER_ALLOC_SOFTDEVICE                                       0
// Used by the test module.
#define WOWL_TIMER_ALLOC_TEST                                             1

/////  ADC1120 Instances  /////
#define WOWL_ADS1120_INST_RTD                                             0
#define WOWL_ADS1120_INST_NTC                                             1




//*********************************  Macros  *********************************//
#define ENABLE_INTR()                                    asm(" CPSIE    i")
#define DISABLE_INTR()                                   asm(" CPSID    i")

#ifndef NDEBUG
#	define DEBUG_BREAK()                                       __BKPT(0x99)
#else
#	define DEBUG_BREAK()
#endif

#ifndef _countof
#	define _countof(x)                         (sizeof(x) / sizeof((x)[0]))
#endif


// Delay function:
// The number of cycles per loop in 'delay_cycles'.
#define CYCLES_PER_DELAY_LOOP                                            3u
#define USEC_PER_SEC                                               1000000u
#define WAIT_USEC(usec)   delay_cycles(                                   \
		(CPU_CLOCK_HZ / CYCLES_PER_DELAY_LOOP / USEC_PER_SEC) * (usec))
		
// App Timer conversion:
#define MSEC_TO_APP_TIMER_TICKS(msec)                                     \
		((uint32_t)(APP_TIMER_FREQ_HZ * (msec) / MSEC_PER_SEC))

// For unused parameters.
#define UNUSED(x)                                              ((void) (x))

#define IDLE()                                                wowlBleIdle()

//**********************************  Types  *********************************//
// Forward declaration of BLE event type:
struct tag_ble_evt;

// Forward declaration of GPIOTE event:
typedef void (*fwd_gpiote_evt_handler_t)
		(uint32_t pin, nrf_gpiote_polarity_t action);

// Forward declaration:
struct tag_nrf_drv_twi_evt;

// Callback type for I2C events.
typedef void (* wowl_i2c_callback_t)(const struct tag_nrf_drv_twi_evt *pEvent);

typedef struct tag_reprogram {
	uint32_t 
		firmwareSize,
		crc32;
} REPROGRAM;

typedef struct tag_record_dwnld_req {
	uint16_t
		start,    // storage index of first entry to stream
		count;    //  number of entries to stream
} RECORD_DWNLD_REQ;

typedef struct tag_sample {
	uint32_t
		count,
		status,
		reserved;
	float values[N_SI];
	uint32_t crc;	
} SAMPLE;

typedef struct tag_record_entry {
	uint16_t
		iRec,      // zero based index of the record
		recState;  // error code
	SAMPLE s;
} RECORD_ENTRY;

//****************************  Global Functions  ****************************//
extern void delay_cycles(uint32_t nCycles);

void wowlAdcInit(void);

void wowlBattInit(void);
void wowlBattVisit(void);	

// Idle the processor, waiting for an event.
void wowlBleIdle(void);
void wowlBleInit(void);
bool wowlBleSendNotification(uint16_t valueHandle, void* pData, uint16_t len);
bool wowlBleTxBusy(void);
void wowlBleUpdateValueSize(uint16_t valueHandle, uint16_t len);

void wowlCmdHandle(uint8_t cmd);

void wowlDfuSetup(const REPROGRAM *pReprogram);
void wowlDfuWrite(uint32_t offset, const uint32_t *pFirmware, uint32_t nBytes);
wowl_reprogram_err_t wowlDfuReprogram(void);

void wowlExtAdcInit(void);
void wowlExtAdcVisit(void);

void wowlFlashInit(void);
void wowlFlashVisit(void);	

void wowlInit(void);
void wowlInitSetupGpioInterrupt(
		uint8_t pin, fwd_gpiote_evt_handler_t pFun);
// Make an asynchronous transfer using the shared instance.
bool wowlInitI2cXfer(uint8_t addr,
		const void* pTx, uint8_t nTx, void* pRx, uint8_t nRx,
		wowl_i2c_callback_t callback);

void wowlLedInit(void);
void wowlLedVisit(void);

uint32_t wowlMainGetStatus(void);
void wowlMainClearStatus(uint32_t toClear);
void wowlMainSetStatus(uint32_t toSet);

void wowlPmInit(void);
void wowlPmVisit(void);	

const SAMPLE* wowlSampleGet(void);
void wowlSampleInit(void);
void wowlSampleVisit(void);	

// Initializes the WOWL service.
void wowlServiceInit(void);
void wowlServiceEventHandler(const struct tag_ble_evt* pBleEvt);
// MUST be 32-bit aligned.
const uint32_t* wowlServiceGetDfuData(void);
const REPROGRAM* wowlServiceGetDfuSetup(void);
const RECORD_DWNLD_REQ* wowlServiceGetDownloadReq(void);
uint8_t wowlServiceGetPressureOffset(void);
void wowlServiceSetRecordCount(uint16_t nRec);
bool wowlServiceTransmitRecord(
		uint16_t iRec, uint16_t recState, const SAMPLE* pSamp);
void wowlServiceTransmitResults(const SAMPLE *pResults);

bool wowlSmVisit(void);
wowl_state_t wowlSmGetState(void);
void wowlSmClearEvents(uint32_t toClear);
uint32_t wowlSmGetEvents(void);
void wowlSmSetEvent(wowl_event_t event);

void wowlTests(void);

#endif /* __WOWL_H__ */


