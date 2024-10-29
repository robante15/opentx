/*
 * Copyright (C) EdgeTX
 *
 * Based on code named
 *   opentx - https://github.com/opentx/opentx
 *   th9x - http://code.google.com/p/th9x
 *   er9x - http://code.google.com/p/er9x
 *   gruvin9x - http://code.google.com/p/gruvin9x
 *
 * License GPLv2: http://www.gnu.org/licenses/gpl-2.0.html
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include "opentx.h"
#include "i2c_driver.h"

void eepromPageWrite(uint8_t *pBuffer, uint16_t WriteAddr, uint8_t NumByteToWrite);
void eepromWaitEepromStandbyState(void);

/**
 * @brief Reads a block of data from EEPROM into a buffer.
 *
 * This function reads a specified number of bytes from the EEPROM starting
 * at a given address and stores them in the provided buffer. It handles
 * reading in chunks if the size exceeds the maximum transfer size that
 * I2C_TransferHandling can handle at once.
 *
 * @param buffer Pointer to the buffer where the read data will be stored.
 * @param address The starting address in the EEPROM from where data will be read.
 * @param size The number of bytes to read from the EEPROM.
 */
void eepromReadBlock(uint8_t *buffer, size_t address, size_t size)
{
  const uint8_t maxSize = 255; // I2C_TransferHandling can handle up to 255 bytes at once
  uint32_t offset = 0;
  while (size > maxSize)
  {
    size -= maxSize;
    while (!I2C_ReadBlock(I2C_ADDRESS_EEPROM, buffer + offset, address + offset, 2, maxSize))
    {
      i2cInit();
    }
    offset += maxSize;
  }
  if (size)
  {
    while (!I2C_ReadBlock(I2C_ADDRESS_EEPROM, buffer + offset, address + offset, 2, size))
    {
      i2cInit();
    }
  }
}

/**
 * @brief Writes a block of data to EEPROM.
 *
 * This function writes a block of data from the provided buffer to the EEPROM
 * starting at the specified address. It handles page boundaries and ensures
 * that data is written correctly across multiple pages if necessary.
 *
 * @param buffer Pointer to the data buffer to be written to EEPROM.
 * @param address The starting address in EEPROM where the data will be written.
 * @param size The number of bytes to write from the buffer to EEPROM.
 */
void eepromWriteBlock(uint8_t *buffer, size_t address, size_t size)
{
  I2C_WriteInPages(I2C_ADDRESS_EEPROM, buffer, address, size, I2C_FLASH_PAGESIZE);
}

uint8_t eepromIsTransferComplete()
{
  return 1;
}

/**
 * @brief  Wait for EEPROM Standby state
 * @param  None
 * @retval None
 */
// #define I2C_PROPER_WAIT // +128B
#define I2C_STANDBY_WAIT_MAX 100
bool I2C_EE_WaitEepromStandbyState(void)
{
#if defined(I2C_PROPER_WAIT)
  __IO uint32_t trials = 0;
  I2C_TransferHandling(I2C, I2C_ADDRESS_EEPROM, 0, I2C_AutoEnd_Mode, I2C_No_StartStop);
  do
  {
    I2C_ClearFlag(I2C, I2C_ICR_NACKCF | I2C_ICR_STOPCF);
    I2C_GenerateSTART(I2C, ENABLE);
    delay_ms(1);
    if (trials++ == I2C_STANDBY_WAIT_MAX)
    {
      return false;
    }
  } while (I2C_GetFlagStatus(I2C, I2C_ISR_NACKF) != RESET);

  I2C_ClearFlag(I2C, I2C_FLAG_STOPF);
#else
  delay_ms(5);
#endif
  return true;
}

void eepromWaitEepromStandbyState(void)
{
  while (!I2C_EE_WaitEepromStandbyState())
  {
    i2cInit();
  }
}
