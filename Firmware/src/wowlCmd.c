/*
	wowlCmd.c
	
	Command handling module for the WOWL.  See the ICD for complete 
	descriptions of the supported commands.
	
	2020-09-22  WHF  Created.
*/

#include "wowl.h"

#include <nrf_nvic.h>

//*******************************  Module Data  ******************************//
// Accummulates the count of bytes written, as this protocol does not
//  include the offset.
static uint32_t dfuOffset;

//****************************  Global Functions  ****************************//
void wowlCmdHandle(uint8_t cmd)
{
	wowlMainClearStatus(WOWL_STATUS_LAST_COMMAND_UNSUPPORTED);
	
	switch (cmd) {
		case WOWL_CMD_DOWNLOAD:
		// The user has made a request to download one or more recordings.
		// Signal that streaming should occur.
		wowlSmSetEvent(EVENT_DOWNLOAD);
		break;
		
		case WOWL_CMD_ERASE:
		// The user wishes to the erase the contents of the flash.
		// Signal that erasing should occur.
		wowlSmSetEvent(EVENT_ERASE);
		break;
		
		case WOWL_CMD_RESET:
		// Reset the processor:
		sd_nvic_SystemReset();
		break;

		case WOWL_CMD_SETUP_FIRMWARE_UPDATE:
		// Unaligned access ok for word via LDR.
		wowlDfuSetup(wowlServiceGetDfuSetup());
		dfuOffset = 0u;
		break;
		
		case WOWL_CMD_WRITE_FIRMWARE:
		wowlDfuWrite(
				dfuOffset,  // offset
				// guaranteed 32-bit aligned:
				wowlServiceGetDfuData(),  
				WOWL_DFU_WRITE_SIZE
		);
		dfuOffset += WOWL_DFU_WRITE_SIZE;
		break;
			
		case WOWL_CMD_REPROGRAM: {
			// This will not return if successful.
			const wowl_reprogram_err_t 
				err = wowlDfuReprogram();
							
			assert(err == WOWL_RE_NONE);
			if (err != WOWL_RE_NONE) {
				wowlMainSetStatus(WOWL_STATUS_FIRMWARE_OPERATION_FAILURE);
			} else {
				// Ok.
			}
		}
		break;		

		default:
		// An unsupported command.  Set the flag.
		wowlMainSetStatus(WOWL_STATUS_LAST_COMMAND_UNSUPPORTED);
		break;
	}

}


