/*
	si705x.h
	
	Constants for the Silicon Labs Si705x series of digital temperature sensors.
	
	2020-10-06  WHF  Created.
*/

#ifndef __SI705X_H__
#define __SI705X_H__

// The 7-bit address to which the device responds.
#define SI705X_I2C_ADDR                                               0x40u


//********************************  Commands  ********************************//
// These commands are one byte.
#define SI705X_CMD_RESET                                              0xFEu
#define SI705X_CMD_MEAS_TEMP_HOLD                                     0xE3u
#define SI705X_CMD_MEAS_TEMP_NO_HOLD                                  0xF3u
#define SI705X_CMD_WRITE_REG                                          0xE6u
#define SI705X_CMD_READ_REG                                           0xE7u

// These commands are two bytes.  Note as u16's the LSB will be sent first.
#define SI705X_CMD2_READ_ID_1                                       0x0FFAu
#define SI705X_CMD2_READ_ID_2                                       0xC9FCu
#define SI705X_CMD2_READ_FIRMWARE_ID                                0xB884u

//**********************  Manufacturer Device ID Codes  **********************//
#define SI7050_ID_CODE                                                0x32u
#define SI7051_ID_CODE                                                0x33u
#define SI7053_ID_CODE                                                0x35u
#define SI7054_ID_CODE                                                0x36u
#define SI7055_ID_CODE                                                0x37u

//*********************************  Macros  *********************************//
// A macro to convert between raw counts and the output.  Note it includes 
//   byte swapping of the raw code.
#define SI705X_CONVERT_CELSIUS(code)                                      \
		((float) ((code) << 8 & 0xFF00 | (code) >> 8 & 0x00FF)                \
		* (175.72f / 65536.0f) - 46.85f)

#endif /* __SI705X_H__ */


