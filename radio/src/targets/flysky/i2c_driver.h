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

#include "stdint.h"
#include "stddef.h"

void i2cInit();
bool I2C_WaitEvent(uint32_t event);
bool I2C_WaitEventCleared(uint32_t event);
bool I2C_ReadBlock(uint8_t deviceAddress, uint8_t* pBuffer, uint16_t memAddr, uint8_t memAddrSize, uint16_t numBytesToRead);
bool I2C_WriteBlock(uint8_t deviceAddress, uint8_t *pBuffer, uint16_t memAddr, uint8_t memAddrSize, uint8_t numBytesToWrite);
bool I2C_WriteInPages(uint8_t deviceAddress, uint8_t *buffer, size_t memAddr, size_t size, uint8_t pageSize);