/*
 * Copyright (C) OpenTX
 *
 * Based on code named
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

#ifndef _I2C_DRIVER_H_
#define _I2C_DRIVER_H_

#include "stdint.h"

// I2C driver: EEPROM
#define I2C_ADDRESS_EEPROM    0x50
#define EEPROM_SIZE           (16*1024)
#define EEPROM_PAGE_SIZE      (64)
#define EEPROM_SIZE           (16*1024)
#define EEPROM_BLOCK_SIZE     (64)

#endif
