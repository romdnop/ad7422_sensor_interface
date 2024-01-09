#ifndef ADT7422_H_
#define ADT7422_H_

#include "stm32f4xx_hal.h"
#include "main.h"

#define ADT7422_I2CADDR_DEFAULT (0x48<<1) ///< I2C address

#define ADT7422_REG__ADT7422_TEMPMSB 0x00 ///< Temp. value MSB
#define ADT7422_REG__ADT7422_TEMPLSB 0x01 ///< Temp. value LSB
#define ADT7422_REG__ADT7422_STATUS 0x02  ///< Status register
#define ADT7422_REG__ADT7422_CONFIG 0x03  ///< Configuration register
#define ADT7422_REG__ADT7422_TEMPMSB_HIGH 0x04 ///< Temp. value MSB_HIGH
#define ADT7422_REG__ADT7422_TEMPLSB_HIGH 0x05 ///< Temp. value LSB_HIGH
#define ADT7422_REG__ADT7422_TEMPMSB_LOW 0x06 ///< Temp. value MSB_LOW
#define ADT7422_REG__ADT7422_TEMPLSB_LOW 0x07 ///< Temp. value LSB_LOW
#define ADT7422_REG__ADT7422_TEMPMSB_CRIT 0x08 ///< Temp. value MSB_CRIT
#define ADT7422_REG__ADT7422_TEMPLSB_CRIT 0x09 ///< Temp. value LSB_CRIT
#define ADT7422_REG__ADT7422_THYST 0x0A      ///< Temp. HYST
#define ADT7422_REG__ADT7422_ID 0x0B      ///< Manufacturer identification
#define ADT7422_REG__ADT7422_RSVD1 0x0C      ///< Reserved
#define ADT7422_REG__ADT7422_RSVD2 0x0D      ///< Reserved
#define ADT7422_REG__ADT7422_RSVD3 0x2E      ///< Reserved
#define ADT7422_REG__ADT7422_SWRST 0x2F  ///< Software reset


#define ADT7422_MANUFACTURER_ID 0xCB

#endif