#ifndef __SHARC_HAL_H__
#define __SHARC_HAL_H__


#include "twi_simple.h"

/** @note: Timeout delay for TWI is not implemented at this layer, so HAL_MAX_DELAY value is arbitrary */
#define HAL_MAX_DELAY 0
#define USE_SYSLOG

/** @note: function currently uses a fixed memAddressSize of 1. SSD1306 device needs no further complication for this */
void HAL_I2C_Mem_Write( sTWI* twi, uint8_t i2c_addr, uint8_t memAddress, uint16_t memAdressSize, uint8_t* pData, uint16_t data_len, uint32_t timeout);
void ssd1306_twiOpen(sTWI* twi);
void HAL_Delay(uint32_t ms);

#endif // __SHARC_HAL_H__