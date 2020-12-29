/*
	wowlCodes.h
	
	Declarations for constants and data structures shared between the WOWL
	device and host devices.
	
	2020-09-22  WHF  Created.
*/

#ifndef __WOWL_CODES_H__
#define __WOWL_CODES_H__

#include <stdint.h>

//********************************  Constants  *******************************//


// WOWL service UUID.  Used by profile and advertising data.
#define WOWL_SERVICE_UUID                                             0x3090

/////  Characteristic UUIDs (16-bit).  /////
#define WOWL_PRESS_OFFSET_UUID                                        0x3091
#define WOWL_RECORD_COUNT_UUID                                        0x3092
#define WOWL_MEAS_RESULTS_UUID                                        0x3093
#define WOWL_RECORD_DWNLD_REQ_UUID                                    0x3094
#define WOWL_RECORD_DWNLD_DATA_UUID                                   0x3095
#define WOWL_COMMAND_UUID                                             0x3096
#define WOWL_DFU_SETUP_UUID                                           0x3097
#define WOWL_DFU_DATA_UUID                                            0x3098

/////  Command Codes  /////
#define WOWL_CMD_DOWNLOAD                                             0x01u
#define WOWL_CMD_ERASE                                                0x02u
#define WOWL_CMD_RESET                                                0x03u
#define WOWL_CMD_SETUP_FIRMWARE_UPDATE                                0x04u
#define WOWL_CMD_WRITE_FIRMWARE                                       0x05u
#define WOWL_CMD_REPROGRAM                                            0x06u

// Size of Device Firmware Update chunks.  Limited by the Softdevice attribute
//  table size.
#define WOWL_DFU_WRITE_SIZE                                            256u


//**********************************  Types  *********************************//

#endif /* __WOWL_CODES_H__ */

