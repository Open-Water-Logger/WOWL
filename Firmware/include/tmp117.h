/*
 * tmp117.h
 *
 * Constants for the Texas Instruments TMP117 series of digital temperature sensors.
 *
 * Author: Ira Ray Jenkins (IJenkins)
 * Date Created: 2022-02-23
 */

#ifndef __TMP117_H__
#define __TMP117_H__

//********************************  Constants  *******************************//

#define TMP117_I2C_ADDR                                               0x48u

///// Register Addresses  /////
#define TMP117_REG_TEMP_RESULT                                        0x00u
#define TMP117_REG_CONFIG                                             0x01u
#define TMP117_REG_THIGH_LIMIT                                        0x02u
#define TMP117_REG_TLOW_LIMIT                                         0x03u
#define TMP117_REG_EEPROM_UL                                          0x04u
#define TMP117_REG_EEPROM1                                            0x05u
#define TMP117_REG_EEPROM2                                            0x06u
#define TMP117_REG_TEMP_OFFSET                                        0x07u
#define TMP117_REG_EEPROM3                                            0x08u
#define TMP117_REG_DEVICE_ID                                          0x0Fu

/////  Register Values  /////

// Config Register Values

// Default value after reset:
#define TMP117_CONFIG_DEFAULTS                                      0x0220u

#define TMP117_CONFIG_HIGH_ALERT                                    0x8000u
#define TMP117_CONFIG_LOW_ALERT                                     0x4000u
#define TMP117_CONFIG_DATA_READY                                    0x2000u
#define TMP117_CONFIG_EEPROM_BUSY                                   0x1000u
#define TMP117_CONFIG_MOD_MASK                                      0x0C00u
#define TMP117_CONFIG_MOD_CONTINUOUS                                0x0000u
#define TMP117_CONFIG_MOD_SHUTDOWN                                  0x0400u
#define TMP117_CONFIG_MOD_reserved                                  0x0800u
#define TMP117_CONFIG_MOD_ONESHOT                                   0x0C00u
#define TMP117_CONFIG_CONV_MASK                                     0x0380u
// Note these values are for no averaging (one sample).
//  See Table 7.7 for conversion cycle values with averaging.
#define TMP117_CONFIG_CONV_15_5_MSEC                                0x0000u
#define TMP117_CONFIG_CONV_125_MSEC                                 0x0080u
#define TMP117_CONFIG_CONV_250_MSEC                                 0x0100u
#define TMP117_CONFIG_CONV_500_MSEC                                 0x0180u
#define TMP117_CONFIG_CONV_1_SEC                                    0x0200u
#define TMP117_CONFIG_CONV_4_SEC                                    0x0280u
#define TMP117_CONFIG_CONV_8_SEC                                    0x0300u
#define TMP117_CONFIG_CONV_16_SEC                                   0x0380u
#define TMP117_CONFIG_AVG_MASK                                      0x0060u
// Note that the conversion time does not necessarily scale with the number
//  of averages, except for CONV=0.  See Table 7.7.
#define TMP117_CONFIG_AVG_NONE                                      0x0000u
#define TMP117_CONFIG_AVG_8                                         0x0020u
#define TMP117_CONFIG_AVG_32                                        0x0040u
#define TMP117_CONFIG_AVG_64                                        0x0060u
#define TMP117_CONFIG_TNA                                           0x0010u
#define TMP117_CONFIG_POL_ACTIVE_HI                                 0x0008u
#define TMP117_CONFIG_POL_ACTIVE_LO                                 0x0000u
#define TMP117_CONFIG_ALERT_PIN_ALERT                               0x0000u
#define TMP117_CONFIG_ALERT_PIN_DRDY                                0x0004u
#define TMP117_CONFIG_RESET                                         0x0002u
#define TMP117_CONFIG_reserved_0001                                 0x0001u

#define TMP117_DEV_ID                                               0x0117u


//*********************************  Macros  *********************************//

// Note this is just Q7.
#define TMP117_CONVERT_CELSIUS(code) ((float)(code)*0.0078125f)

#endif /* __TMP117_H__ */
