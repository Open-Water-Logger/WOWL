/*
	wowlDfu.c
	
	Device Firmware Update for the WOWL.
	
	2020-10-29  WHF  Created, from btlemDfu.c.
*/

#include "wowl.h"

#include <CRC.h>
#include <fstorage.h>
#include <nrf_sdm.h>

// Use IAR specific keywords.
#pragma language=extended

//********************************  Constants  *******************************//
// Defined by SoftDevice 3.0.
#define FLASH_FIRMWARE_BASE                                        0x1F000U
#define VECTOR_TABLE_SIZE                                          0x00400U
#define FLASH_FILE_BASE                                            0x44000U
// This is a holdover, but is likely sufficient.
#define FLASH_PARAMETER_BASE                                       0x6A000U

// Maximum number of bytes allowed for Flash storage.
#define FLASH_FILE_SIZE            (FLASH_PARAMETER_BASE - FLASH_FILE_BASE)

// An address lies in RAM if after this mask the address equals RAM_ADDR_CHECK.
#define RAM_ADDR_MASK                                           0xFFFF0000U
#define RAM_ADDR_CHECK                                          0x20000000U


//*********************************  Macros  *********************************//

// Wait for the non-volatile memory controller to report not-busy.
#define WAIT_BUSY()  do {} while(NRF_NVMC->READY == NVMC_READY_READY_Busy);


//***********************  Local Function Declarations  **********************//
static void doProgram(const uint32_t nBytes);
static void dfuFlashCallback(const fs_evt_t* pEvt, fs_ret_t result);

//*******************************  Module Data  ******************************//
static REPROGRAM reprog;

// Because we explicitly set the start and end addresses, they will
//   not be auto-assigned by the framework.
static FS_REGISTER_CFG(fs_config_t dfuFsConfig) = {
	.p_start_addr = (const uint32_t*) FLASH_FILE_BASE,
	.p_end_addr = (const uint32_t*) FLASH_PARAMETER_BASE,
	.callback = dfuFlashCallback,
	.num_pages = 0U,     // unused
	.priority = 0U       // unused
};

//****************************  Global Functions  ****************************//
// Install firmware from the file written to Flash (in binary format)
//  to code space (also in Flash).
wowl_reprogram_err_t wowlDfuReprogram(void)
{
	const uint32_t *pSrc = (const uint32_t*) FLASH_FILE_BASE;
	wowl_reprogram_err_t result = WOWL_RE_NONE;
	
	// Indicate that the operation is not yet complete.
	wowlMainClearStatus(WOWL_STATUS_FIRMWARE_OPERATION_COMPLETE);
	
	// Sanity checks.
	// (1) The firmware should be larger than the vector table.  In reality, it 
	//   will be much larger.
	if (reprog.firmwareSize <= VECTOR_TABLE_SIZE) {
		// Check failed.
		result = WOWL_RE_SIZE_MISMATCH;
		
	// (2) The first word is the stack pointer, which should lie in RAM.
	} else if ((*pSrc & RAM_ADDR_MASK) != RAM_ADDR_CHECK) {
		// Check failed.
		result = WOWL_RE_STACK_PLACEMENT;
		
	// (3) The CRC should match.
	} else if (crc32((const uint8_t *) FLASH_FILE_BASE, reprog.firmwareSize)
			!= reprog.crc32) {
		// Check failed.
		result = WOWL_RE_CRC_ERROR;
		
	} else {
		// All checks passed.
		
		// Disable the SoftDevice, so that it does not interfere with 
		//  Flash accesses.  The return code is meaningless.
		(void) sd_softdevice_disable();
		
		// Will not return.
		doProgram(reprog.firmwareSize);
	}

	return result;
}

// Write firmware bytes to a temporary location.
void wowlDfuSetup(const REPROGRAM *pReprogram)
{
	// Cache reprogram information:
	reprog = *pReprogram;
	
	// Indicate that the flash is busy.
	wowlMainClearStatus(WOWL_STATUS_FIRMWARE_OPERATION_COMPLETE);
	
	// Erase necessary pages asynchronously.
	fs_ret_t fsResult = fs_erase(
			&dfuFsConfig,
			(uint32_t*) FLASH_FILE_BASE,    // address to erase
			reprog.firmwareSize / FLASH_PAGE_SIZE + 1U, // num_pages
			NULL                        // user context (unused)
	);
			
	assert(fsResult == FS_SUCCESS);			
}

void wowlDfuWrite(uint32_t offset, const uint32_t *pFirmware, uint32_t nBytes)
{
	// Indicate that the flash is busy.
	wowlMainClearStatus(WOWL_STATUS_FIRMWARE_OPERATION_COMPLETE);
	
	if ((offset & 3U) != 0U || (nBytes & 3U) != 0U) {
		// Only 32-bit word aligned accesses permitted.  Ignore the request.
	} else {
		// Queue for write to Flash.  Alignment in this cast is ok because we
		//   have checked that byteOffset is a multiple of 4 above.
		uint32_t *pDst = (uint32_t*) (FLASH_FILE_BASE + offset);
		
		assert(((uint32_t) pFirmware & 3U) == 0U);
		
		fs_ret_t fsResult = fs_store(
				&dfuFsConfig,
				pDst,                         // address to write to
				pFirmware,                    // data to write 
				nBytes >> 2,                  // n 32-bit words
				NULL                          // user context (unused)
		);

		assert(fsResult == FS_SUCCESS);
	}
}

//***********************  Local Function Definitions  ***********************//

// Copy the program data from the temporary Flash area to code space, and 
//  reset the device.
// This function and all its dependents must run from RAM. 
static __ramfunc void doProgram(const uint32_t nBytes)
{
	uint32_t index = 0U;
	const uint32_t *pSrc = (const uint32_t*) FLASH_FILE_BASE;
	uint32_t *pDst = (uint32_t*) FLASH_FIRMWARE_BASE;
		
	// Disable all interrupts.
	DISABLE_INTR();
	
	// For each page,
	while (index < nBytes) {
		uint32_t i;
		
		// (1) Erase the page.
		// Enable erase by setting the EEN bit in the config register.
		NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Een;
		
		// Begin erasing.
		NRF_NVMC->ERASEPAGE = (uint32_t) pDst;
		
		// Wait until erase is complete:
		WAIT_BUSY();
		
		// Check equal to erased value:
		assert(*pDst == 0xFFFFFFFFU);
	
		// (2) Write the new page contents.
		// First, enable writes by setting the WEN bit.
		NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Wen;
		
		// Transfer via simple copy.  The write must be a full, 32-bit word.
		//  The compiler should compile the following to STR with no type code
		//  (such as B or H).
		for (i = 0U; i < FLASH_PAGE_SIZE / sizeof(uint32_t); ++i) {
			*pDst++ = *pSrc++;
		}
	
		// (3) Advance indices.  (Only index needs to be advanced; the 
		//   pointers have already been incremented.
		index += FLASH_PAGE_SIZE;
	}
	
	// Wait until write is complete before resetting:
	WAIT_BUSY();
	
	DEBUG_BREAK();
	
	// Reset the processor; see NVIC_SystemReset() (which we do not call
	//   because it is not in RAM...)
	SCB->AIRCR  = (uint32_t)(
			(0x5FAUL << SCB_AIRCR_VECTKEY_Pos) |
			// Keep priority group unchanged:
			(SCB->AIRCR & SCB_AIRCR_PRIGROUP_Msk) |
			SCB_AIRCR_SYSRESETREQ_Msk    
	);
	
	// Block forever.
	for (;;) ;
}

static void dfuFlashCallback(const fs_evt_t* pEvt, fs_ret_t result)
{
	assert(result == FS_SUCCESS);

	if (result == FS_SUCCESS) {	
		switch (pEvt->id) {
			case FS_EVT_STORE:
			case FS_EVT_ERASE:
			// Indicate that the operation was successful.
			wowlMainSetStatus(WOWL_STATUS_FIRMWARE_OPERATION_COMPLETE);
			break;
			
			default:
			break;
		}
	} else {
		// Failure.  The host will timeout.
	}
}


