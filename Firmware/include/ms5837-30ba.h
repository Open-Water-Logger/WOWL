/*
	ms5837-30ba.h
	
	Declarations for the gel-filled pressure and temperature 24-bit digital 
	sensor with I2C interface.
	
	2020-10-02  WHF  Created.
*/

#ifndef __MS5837_30BA_H__
#define __MS5837_30BA_H__

// The 7-bit address to which the device responds.
#define MS5837_I2C_ADDR                                               0x76u

//********************************  Commands  ********************************//
#define MS5837_CMD_READ_ADC                                           0x00u
#define MS5837_CMD_RESET                                              0x1Eu
#define MS5837_CMD_CONVERT_PRESS(osr)               (0x40u + ((osr)&0x0Eu))
#define MS5837_CMD_CONVERT_TEMP(osr)                (0x50u + ((osr)&0x0Eu))
#define MS5837_CMD_READ_PROM(addr)              (0xA0u + ((addr)<<1&0x0Eu))

//*****************************  Command Values  *****************************//
#define MS5837_OSR_256                                                0x00u
#define MS5837_OSR_512                                                0x02u
#define MS5837_OSR_1024                                               0x04u
#define MS5837_OSR_2048                                               0x06u
#define MS5837_OSR_4096                                               0x08u

#define MS5837_PROM_ADDR_CRC_ID                                       0x00u
#define MS5837_PROM_ADDR_C1                                           0x01u
#define MS5837_PROM_ADDR_C2                                           0x02u
#define MS5837_PROM_ADDR_C3                                           0x03u
#define MS5837_PROM_ADDR_C4                                           0x04u
#define MS5837_PROM_ADDR_C5                                           0x05u
#define MS5837_PROM_ADDR_C6                                           0x06u

#define MS5837_PROM_LENGTH                                            0x07u

// The CRC calculation expects a buffer of this length; the spare byte
//  must be zero.
#define MS5837_CRC_LENGTH                                             0x08u

#endif /* __MS5837_30BA_H__ */


