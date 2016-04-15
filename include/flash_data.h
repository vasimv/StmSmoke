/* Temperature controller for vaping mod, schematics in Stm-Smoke.sch or Stm-Smoke-Inductor.sch
 *
 *    Copyright (C) 2016  Vasim V.
 *
 *    This program is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU Lesser General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU Lesser General Public License
 *    along with this program. */

#include "stm32f3xx_hal.h"

// Flash area used for data storage (2x 2048 bytes pages)
#define FLASH_DATA_PAGE_SIZE FLASH_PAGE_SIZE
#define FLASH_DATA_PAGES 2
// We do have just 64 Kb (32x2048), so last two pages will be used
#define FLASH_DATA_START ((void *) 0x08000000 + (32 - FLASH_DATA_PAGES) * FLASH_DATA_PAGE_SIZE)

// Set size of configuration array (should be less than FLASH_DATA_PAGE_SIZE)
// Must be called before first use
void FlashData_Set_Size(int csize);
// Write configuration array to the flash
void FlashData_Write(void *carray);
// Read last written configuration array from the flash
int FlashData_Read(void *carray);
