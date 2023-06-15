/*
	wowlFlash.c

	Implementation of the Flash module for the WOWL.

	2020-09-22  WHF  Created.
*/

#include "wowl.h"

#include <arm/spi_flash_nrf.h>

//********************************  Constants  *******************************//
// The datasheet specifies xx23xx, but we consistently get xx28xx.
//#define MX25V1635F_PROD_ID                                      0x001523C2u
#define MX25V1635F_PROD_ID                                      0x001528C2u

// The Macronix MX25V1635F is a 16 Mbit device.
#define MX25V1635F_SIZE_BITS                          (16u * 1024u * 1024u)

#define MAX_RECORDS          (MX25V1635F_SIZE_BITS / (sizeof(SAMPLE) * 8u))

//**********************************  Types  *********************************//
typedef enum tag_read_err {
	READ_ERR_NONE,
	READ_ERR_DEVICE,
	READ_ERR_UNWRITTEN,
	READ_ERR_CRC,
} read_err_t;

//*******************************  Module Data  ******************************//
static uint16_t
	nRecords,     // number of records on Flash
	nDwnld;       // number of entries downloaded in the current request

static enum tag_flash_state_t {
	FS_READY,
	FS_WRITING,
	FS_ERASING,
	FS_DOWNLOAD,
	FS_POWERED_DOWN,	
} flashState;

static_assert(
		256u % sizeof(SAMPLE) == 0u,  // no remainder
		"Sample size inconsistent with page."
);

//***********************  Local Function Declarations  **********************//
static uint32_t countRecords(void);
static read_err_t readRecord(uint32_t iRec, SAMPLE *pS);

//****************************  Global Functions  ****************************//

void wowlFlashInit(void)
{
	static const spi_flash_nrf_init_t init = {
		.spiInstance = 0u,
		.clk = PIN_MEM_SCLK,
		.miso = PIN_MEM_SDO,
		.mosi = PIN_MEM_SDI,
		.nCS = PIN_MEM_nCS,
	};
	
	spiFlashInit((void*) &init);
	
	// Check product ID:
	const uint32_t prodId = spiFlashGetProductID();
	
	if (prodId != MX25V1635F_PROD_ID) {
		assert(false);
		// Set status bit:
		wowlMainSetStatus(WOWL_STATUS_FLASH_FAIL);
	} else {
		// Device ok.
	
		// Count the number of records that are available.
		nRecords = countRecords();
		
		// Notify the service of this count.
		wowlServiceSetRecordCount(nRecords);
	}
}

void wowlFlashVisit(void)
{
	const wowl_state_t state = wowlSmGetState();
	
	switch (flashState) {
		case FS_READY:
		switch (state) {
			case STATE_LIMP:
			case STATE_UNDER_SLEEP: {
				// Command a deep power down.
				const int err = spiFlashPowerDown();
				assert(err == 0);
				flashState = FS_POWERED_DOWN;
			}
			break;
			
			case STATE_UNDER_WRITE: {
				// Begin the write.
				const int err = spiFlashWrite(
						nRecords * sizeof(SAMPLE), // address
						wowlSampleGet(),           // pointer to data
						sizeof(SAMPLE)             // n bytes to write
				);
				
				if (err) {
					// The write failed for some reason.
					// Set a status flag:
					wowlMainSetStatus(WOWL_STATUS_FLASH_FAIL);
					// Falsely claim that the write is done.  We'll try again 
					//  next time.
					wowlSmSetEvent(EVENT_WRITE_DONE);
				} else {
					// Enter the writing state.
					flashState = FS_WRITING;
				}
			}
			break;
			
			case STATE_DOWNLOAD:
			// Begin loading and transmitting data to the host.
			flashState = FS_DOWNLOAD;
			// Zero download count:
			nDwnld = 0u;
			break;
			
			case STATE_ERASE: {
				// Start erasing the flash.
				const int err = spiFlashChipErase();
				
				if (err) {
					// The write failed for some reason.
					// Set a status flag:
					wowlMainSetStatus(WOWL_STATUS_FLASH_FAIL);
					// Falsely claim that the erase is done.
					wowlSmSetEvent(EVENT_ERASE_DONE);
				} else {
					// Enter the erasing state.
					flashState = FS_ERASING;
				}
			}
			break;
			
			default:
			// No action in other states.
			break;
		}
		break; // end FS_READY case			
		
		case FS_WRITING:
		if (!spiFlashIsBusy()) {
			// The write is complete.  Increment the record count:
			++nRecords;
			// Notify the service:
			wowlServiceSetRecordCount(nRecords);
			// Return to ready:
			flashState = FS_READY;
			// Set the event for the state machine:
			wowlSmSetEvent(EVENT_WRITE_DONE);
		} else {
			// Still writing.
		}
		break;
		
		case FS_ERASING:
		if (!spiFlashIsBusy()) {
			// The erase is complete.  Zero the record count:
			nRecords = 0u;
			// Notify the service:
			wowlServiceSetRecordCount(nRecords);
			// Return to ready:
			flashState = FS_READY;
			// Set the event for the state machine:
			wowlSmSetEvent(EVENT_ERASE_DONE);
		} else {
			// Still writing.
		}
		break;
		
		case FS_DOWNLOAD:
		if ((wowlMainGetStatus() & WOWL_STATUS_CONNECTED) == 0u) {
			// No longer connected.  Abort the download.
			// Signal the state machine:
			wowlSmSetEvent(EVENT_DOWNLOAD_DONE);
			// Return to ready.
			flashState = FS_READY;
		} else if (!wowlBleTxBusy()) {			
			const RECORD_DWNLD_REQ* pReq = wowlServiceGetDownloadReq();
			SAMPLE samp;
			// Read the next record:
			const read_err_t err = readRecord(
					pReq->start + nDwnld,
					&samp
			);
			// Transmit the record:
			const bool ok = wowlServiceTransmitRecord(
					nDwnld,
					err,
					&samp
			);
			
			if (ok) {
				// The transmission went through.  Increment and check the 
				// count:
				if (++nDwnld >= pReq->count) {
					// Download complete.  Signal state machine:
					wowlSmSetEvent(EVENT_DOWNLOAD_DONE);
					// Return to ready.
					flashState = FS_READY;
				} else {
					// Not yet complete.
				}
			} else {
				// The transmission did not complete.  Retry next iteration.
			}
		} else {
			// The transmitter is busy.  Wait.
		}
		break; // end case FS_DOWNLOAD
		
		case FS_POWERED_DOWN:
		if (state != STATE_UNDER_SLEEP && state != STATE_LIMP) {
			// Wakeup the flash device.
			(void) spiFlashWakeup();
			flashState = FS_READY;
		} else {
			// Continue to sleep.
		}
		break;		
	}
}

//***********************  Local Function Definitions  ***********************//
static uint32_t countRecords(void)
{
	uint32_t iRec;
	
	for (iRec = 0u; iRec < MAX_RECORDS; ++iRec) {
		SAMPLE rec;
		
		read_err_t err = readRecord(iRec, &rec);
		
		if (err == READ_ERR_UNWRITTEN) {
			// This record has not been written to.  End the search.
			break;
		} else {
			// We don't want other error conditions to prematurely end
			//  the search.  Continue looking.
		}
	}		
	
	return iRec;
}

static read_err_t readRecord(uint32_t iRec, SAMPLE *pS)
{
	read_err_t result;
	const int flashErr = spiFlashRead(
			iRec * sizeof(SAMPLE),
			pS,
			sizeof(SAMPLE)
	);
	
	if (flashErr) {
		result = READ_ERR_DEVICE;
	} else {
		if (pS->packed == UINT32_MAX) {
			// Although possible that all packed sample fields would come out to
			//  all ones, this is incredibly unlikely.  Much more likely is that
			//  the record is raw Flash bits that have not been written, as they
			//  erase to all ones.
			result = READ_ERR_UNWRITTEN;
		} else {
			result = READ_ERR_NONE;
		}
	}
	
	return result;
}

