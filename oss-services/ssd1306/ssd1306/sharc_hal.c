/**
 * @file sharc_hal.c
 * @brief Top-level HAL configuration for SHARC to use with SSD1306 displays
 * @version 0.1
 * @date 2023-07-07
 * 
 * @copyright Copyright (c) 2023
 * 
 */

/** @BEGIN: ADI CODE */

#include "twi_simple.h"
#include "sharc_hal.h"

// Kernel includes
#include "FreeRTOS.h"
#include "task.h"

#ifdef USE_SYSLOG
#include "syslog.h"
#endif


sTWI* SSD1306_I2C_PORT = NULL;


void ssd1306_twiOpen(sTWI* twi)
{
     SSD1306_I2C_PORT = twi;
}

/** @note:  currently using a fixed memAddressSize of 1. SSD1306 device needs no further complication for this.
 *          memAddress and memAddressSize are included to mirror the structure of the STM32 HAL call in the included library.
 * @note:   TWI timeout not configurable here. Implemented in twi_simple.c. Again included just to mirror the library call. 
 */
void HAL_I2C_Mem_Write( sTWI* twi, uint8_t i2c_addr, uint8_t memAddress, uint16_t memAdressSize, uint8_t* pData, uint16_t data_len, uint32_t timeout) 
{
    TWI_SIMPLE_RESULT res;
    
    if (twi == NULL)
    {
        #ifdef USE_SYSLOG
            syslog_print("SSD1306 display needs a valid TWI port.\n Please call ssd1306_twiOpen before attempting to write the display.\n ");
        #endif
        return;
    }

    //Set up the communication buffer
    uint8_t txBuff[1+data_len]; 
    txBuff[0] = memAddress;
    for (int i = 0; i < data_len; i++) 
    {
        txBuff[i+1] = pData[i];
    }
    
    //Transaction & Error-checking
    /** @note: timeout implemented in twi_simple */
    res = twi_write(twi, i2c_addr, txBuff, 1+data_len);
    if (res != TWI_SIMPLE_SUCCESS)
    {
        #ifdef USE_SYSLOG
            syslog_print("Failure with ssd1306_WriteCommand!\n");
        #endif
    }
}

void HAL_Delay(uint32_t ms)
{
    vTaskDelay(pdMS_TO_TICKS(ms));
}
