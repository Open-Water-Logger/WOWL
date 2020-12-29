/*
	wowlService.c
	
	Implementation of the GATT service for the WOWL.
	
	2020-09-22  WHF  Created.
*/

#include "wowl.h"

#include "wowlCodes.h"

// BLE Common:
#include <ble_advdata.h>
#include <ble_conn_params.h>
#include <ble_hci.h>

// System:
#include <nrf.h>
#include <nrf_gpio.h>
#include <softdevice_handler.h>

// Components:
#include <timer/app_timer.h>
#include <util/app_error.h>

//********************************  Constants  *******************************//
// Set to 1 for notification logging.
#define DO_NOTIFICATION_LOG                                               0

//**********************************  Types  *********************************//
// Used to define characteristics in this module.
typedef struct tag_char_def {
	uint16_t
		uuid,
		initialLen, 
		maxLen;	
	uint8_t *pValue;
	ble_gatts_char_handles_t * const pHandle;
	bool readOnly;	
} char_def_t;


//*******************************  Module Data  ******************************//
// Handle of local service (as provided by the BLE stack).
static uint16_t hService;


/////  Characteristic Values  /////
static uint8_t
#if DO_SUBMERSION_TEST
	pressureOffsetValue,
#endif
	commandValue;
static uint16_t
	recordCountValue;
static RECORD_DWNLD_REQ
	recordDwnldReqValue;
static SAMPLE 
	measResultsValue;
static RECORD_ENTRY
	recordDwnldDataValue;
static REPROGRAM
	reprogramValue;
// Declared as uint32_t to guarantee 32-bit alignment:
static uint32_t
	dfuDataValue[WOWL_DFU_WRITE_SIZE / sizeof(uint32_t)];

/////  Characteristic Handles  /////
static ble_gatts_char_handles_t
#if DO_SUBMERSION_TEST
	pressureOffsetChar,
#endif
	recordCountChar,
	hMeasResultsChar,
	recordDwnldReqChar,
	recordDwnldDataChar,
	commandChar,
	reprogramChar,
	dfuDataChar;

/////  Characteristic Definitions  /////
static const char_def_t CHAR_DEFS[] = {
	{
#if DO_SUBMERSION_TEST
		WOWL_PRESS_OFFSET_UUID,                 // uuid
		sizeof(pressureOffsetValue),            // initialLen 
		sizeof(pressureOffsetValue),            // maxLen	
		&pressureOffsetValue,                   // uint8_t *pValue
		&pressureOffsetChar,                    // pHandle
		false                                   // bool readOnly
	}, 	{
#endif
		WOWL_RECORD_COUNT_UUID,                 // uuid
		sizeof(recordCountValue),               // initialLen 
		sizeof(recordCountValue),               // maxLen	
		(uint8_t*) &recordCountValue,           // uint8_t *pValue
		&recordCountChar,                       // pHandle
		true                                    // bool readOnly
	}, 	{
		WOWL_MEAS_RESULTS_UUID,                 // uuid
		sizeof(measResultsValue),               // initialLen 
		sizeof(measResultsValue),               // maxLen	
		(uint8_t*) &measResultsValue,           // uint8_t *pValue
		&hMeasResultsChar,                      // pHandle
		true                                    // bool readOnly
	}, {
		WOWL_RECORD_DWNLD_REQ_UUID,             // uuid
		sizeof(recordDwnldReqValue),            // initialLen 
		sizeof(recordDwnldReqValue),            // maxLen	
		(uint8_t*) &recordDwnldReqValue,        // uint8_t *pValue
		&recordDwnldReqChar,                    // pHandle
		false                                   // bool readOnly
	}, {
		WOWL_RECORD_DWNLD_DATA_UUID,            // uuid
		sizeof(recordDwnldDataValue),           // initialLen 
		sizeof(recordDwnldDataValue),           // maxLen	
		(uint8_t*) &recordDwnldDataValue,       // uint8_t *pValue
		&recordDwnldDataChar,                   // pHandle
		true                                    // bool readOnly
	}, {
		WOWL_COMMAND_UUID,                      // uuid
		sizeof(commandValue),                   // initialLen 
		sizeof(commandValue),                   // maxLen	
		&commandValue,                          // uint8_t *pValue
		&commandChar,                           // pHandle
		false                                   // bool readOnly
	}, {	
		WOWL_DFU_SETUP_UUID,                    // uuid
		sizeof(reprogramValue),                 // initialLen 
		sizeof(reprogramValue),                 // maxLen	
		(uint8_t*) &reprogramValue,             // uint8_t *pValue
		&reprogramChar,                         // pHandle
		false                                   // bool readOnly
	}, {
		WOWL_DFU_DATA_UUID,                     // uuid
		sizeof(dfuDataValue),                   // initialLen 
		sizeof(dfuDataValue),                   // maxLen	
		(uint8_t*) dfuDataValue,                // uint8_t *pValue
		&dfuDataChar,                           // pHandle
		false                                   // bool readOnly
	},	
};	

//***********************  Local Function Declarations  **********************//
static void char_add(const char_def_t *pCharDef);
static void service_add(void);

//****************************  Global Functions  ****************************//
void wowlServiceInit(void)
{
	service_add();
}

#if DO_NOTIFICATION_LOG
uint32_t iNot;
ble_evt_t nots[256];
#endif

// Handle BLE events.  Note that this happens at Interrupt Level, so 
//  no significant processing should happen here.
void wowlServiceEventHandler(const ble_evt_t* pBleEvt)
{
#if DO_NOTIFICATION_LOG
	nots[iNot++]=*pBleEvt;
#endif

	switch (pBleEvt->header.evt_id)
	{
		case BLE_GATTS_EVT_WRITE:
		// NOTE: this event only occurs if the characteristic is written with
		//  WriteCharValue.  WriteLongCharValue does not produce the event.
		if (pBleEvt->evt.gatts_evt.params.write.handle
				== commandChar.value_handle) {
			// A command code was received.
			wowlCmdHandle(commandValue);
		} else {
			// Some other characteristic written.  Ignore.
		}
		break;
		
		default:
		// Ignore other events.
		break;
	}
}

// MUST be 32-bit aligned.
const uint32_t* wowlServiceGetDfuData(void) { return dfuDataValue; }
const REPROGRAM* wowlServiceGetDfuSetup(void) { return &reprogramValue; }

const RECORD_DWNLD_REQ* wowlServiceGetDownloadReq(void) 
{ return &recordDwnldReqValue; }

uint8_t wowlServiceGetPressureOffset(void) 
{
#if DO_SUBMERSION_TEST
	return pressureOffsetValue;
#else
	return 0u;
#endif
}

void wowlServiceSetRecordCount(uint16_t nRec)
{
	recordCountValue = nRec;
}

bool wowlServiceTransmitRecord(
		uint16_t iRec, uint16_t recState, const SAMPLE* pSamp)
{
	// Cache locally:
	recordDwnldDataValue.iRec = iRec;
	recordDwnldDataValue.recState = recState;
	recordDwnldDataValue.s = *pSamp;
	
	// Notify host of change:
	return wowlBleSendNotification(
			recordDwnldDataChar.value_handle,
			&recordDwnldDataValue,
			sizeof(recordDwnldDataValue)
	);
}

// Signal the host with the updated measurements.
void wowlServiceTransmitResults(const SAMPLE *pResults)
{
	// Cache locally:
	measResultsValue = *pResults;
	
	// Notify host of change:
	wowlBleSendNotification(
			hMeasResultsChar.value_handle,
			&measResultsValue,
			sizeof(measResultsValue)
	);
}

//***********************  Local Function Definitions  ***********************//
//  Add a characteristic to the service.
static void char_add(const char_def_t *pCharDef)
{
	uint32_t			err_code;
	ble_gatts_char_md_t char_md;
	ble_gatts_attr_t	attr_char_value;
	ble_uuid_t			char_uuid;
	ble_gatts_attr_md_t 
		attr_md,
		cccd_md;

	// Define the Client Characteristic Configuration Data meta-data: 
	memset(&cccd_md, 0, sizeof(cccd_md));
	cccd_md.vloc = BLE_GATTS_VLOC_STACK;
	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);

	// Setup the meta-data for the characteristic.
	memset(&char_md, 0, sizeof(char_md));
	char_md.char_props.read	  = 1;
	char_md.char_props.write  = !pCharDef->readOnly;
	char_md.char_props.notify = 1;
	char_md.p_char_user_desc  = NULL;
	char_md.p_char_pf		  = NULL;
	char_md.p_user_desc_md	  = NULL;
	char_md.p_cccd_md		  = &cccd_md;
	char_md.p_sccd_md		  = NULL;

	// Setup the attribute meta-data for the characteristic.
	memset(&attr_md, 0, sizeof(attr_md));
	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
	if (pCharDef->readOnly) {
		BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.write_perm);
	} else {
		BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
	}
	attr_md.vloc	= BLE_GATTS_VLOC_USER;  // our array will be used
	attr_md.rd_auth = 0;
	attr_md.wr_auth = 0;
	// The characteristic is variable length if the intial and max lengths
	//  are not identical.
	attr_md.vlen	= pCharDef->initialLen != pCharDef->maxLen;

	// Setup the characteristic.
	memset(&attr_char_value, 0, sizeof(attr_char_value));
	char_uuid.type = BLE_UUID_TYPE_BLE;
	char_uuid.uuid = pCharDef->uuid;
	attr_char_value.p_uuid	  = &char_uuid;
	attr_char_value.p_attr_md = &attr_md;
	attr_char_value.init_len  = pCharDef->initialLen;
	attr_char_value.init_offs = 0;
	attr_char_value.max_len	  = pCharDef->maxLen;
	attr_char_value.p_value	  = pCharDef->pValue;

	// Add the characteristic to the service:
	err_code = sd_ble_gatts_characteristic_add(
			hService,
			&char_md,
			&attr_char_value,
			pCharDef->pHandle
	);
	APP_ERROR_CHECK(err_code);
}

// Add the WOWL service to the application.
static void service_add(void)
{
	ble_uuid_t	service_uuid;
	uint32_t
		err_code,
		iChar;
 
	service_uuid.uuid = WOWL_SERVICE_UUID;
	service_uuid.type = BLE_UUID_TYPE_BLE;

	err_code = sd_ble_gatts_service_add(
			BLE_GATTS_SRVC_TYPE_PRIMARY,
			&service_uuid,
			&hService
	);
	APP_ERROR_CHECK(err_code);

	// Add characteristics:
	for (iChar = 0; iChar < _countof(CHAR_DEFS); ++iChar) {
		char_add(&CHAR_DEFS[iChar]);
	}
}

