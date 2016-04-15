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

#include <string.h>
#include "stm32f3xx_hal.h"
#include "flash_data.h"
#include "smoke.h"

// Size of configuration array (with flag and checksum)
unsigned int confsize = 4;
// Size of configuration array (without additional data)
unsigned int realconfsize = 0;

// Number of arrays in one flash page
unsigned int confnum = 512;

#define FLASHPAGE(x) (FLASH_DATA_PAGE_SIZE * (x) + FLASH_DATA_START)

void FlashData_Set_Size(int csize) {
	// Adds 4 bytes as "written" flag
	realconfsize = csize;
	confsize = ((csize + 3) & 0xfffc) + 4;
	confnum = FLASH_DATA_PAGE_SIZE / confsize;
} // FlashData_Set_Size

// Check if the page needs erase before actual erase
void FlashData_Erase(void *page) {
	int i;
	int havetoerase = 0;

	for (i = 0; i < FLASH_DATA_PAGE_SIZE; i++)
		if (*((uint8_t *) page + i) != 0xff)
			havetoerase = 1;
	if (havetoerase) {
		debug("Performing real erase page %x\n", page);
		FLASH_PageErase((uint32_t) page);
	}
} // FlashData_Erase

// Calculate checksum
uint32_t FlashData_Checksum(void *caddr) {
	uint8_t byte;
	uint32_t checksum = 0;
	unsigned int i;

	for (i = 0; i < realconfsize; i++) {
		byte = *((uint8_t *) (caddr + i));
		checksum += byte;
#ifdef EXTENDED_DEBUG
		debug("byte: %x, sum: %x\n", byte, checksum);
#endif
	}
	// Zero first byte to be sure it won't equal 0xffffffff
	return (checksum & 0x00ffffff);
} // FlashData_Checksum

// Find last written configuration (starting from *addr or last flash address if NULL)
void *FlashData_Find_Last(int verify) {
	void *curaddr;
	uint32_t checksum;
	int found;
	int page;

	curaddr = FLASHPAGE(FLASH_DATA_PAGES - 1) + (confnum - 1) * confsize;
	page = (curaddr - FLASH_DATA_START) / FLASH_DATA_PAGE_SIZE;

	found = 0;
	do {
		checksum = *(uint32_t *) curaddr;
#ifdef EXTENDED_DEBUG
		debug("Checking address %x (%d, %x): %x (%x) ..", curaddr, page, (uint32_t) curaddr % FLASH_DATA_PAGE_SIZE, checksum, *(uint8_t *)curaddr);
#endif
		if (verify && (checksum != 0xffffffff)) {
			// Verify checksum if needed and continue to next if bad one
			if (checksum != FlashData_Checksum(curaddr + 4))
				checksum = 0xffffffff;
		}
		if (checksum == 0xffffffff) {
			if (((uint32_t) curaddr % FLASH_DATA_PAGE_SIZE) >= confsize)
				curaddr = curaddr - confsize;
			else {
				page--;
				curaddr = FLASHPAGE(page) + (confnum - 1) * confsize;
			}
		} else
			found = 1;
#ifdef EXTENDED_DEBUG
		debug(" next address %x\n", curaddr);
#endif
	} while (!found && (page >= 0) && (curaddr >= FLASH_DATA_START));

	debug("Config in flash - found %d, at %p\n", found, curaddr);
	if (found)
		return curaddr;
	return NULL;
} // FlashData_Find_Last

int FlashData_Read(void *carray) {
	void *found = NULL;

	found = FlashData_Find_Last(1);
	if (found) {
		memcpy(carray, found + 4, realconfsize);
		return 1;
	}
	return 0;
} // FlashData_Read

// Writing array to flash memory, erase other pages if we've wrote first one in new page
void FlashData_Write(void *carray) {
	void *found;
	unsigned int offset, i;
	unsigned int page;

	found = FlashData_Find_Last(0);
	if (!found)
		found = FLASH_DATA_START;

	// Check if the record is last one in the page
	if ((((found - FLASH_DATA_START) % FLASH_DATA_PAGE_SIZE) / confsize) == (confnum - 1)) {
		page = ((found - FLASH_DATA_START) / FLASH_DATA_PAGE_SIZE + 1) % FLASH_DATA_PAGES;
		found = FLASHPAGE(page);
	} else
			found += confsize;
	page = (found - FLASH_DATA_START) / FLASH_DATA_PAGE_SIZE;
	// Actual write to the flash
	HAL_FLASH_Unlock();

	CLEAR_BIT(FLASH->CR, FLASH_CR_PER); // ???
	debug("Writing to flash to addr %p\n", found);

	// Write checksum
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, (uint32_t) found, FlashData_Checksum(carray));
	for (offset = 0; offset < realconfsize; offset = offset + 4)
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, (uint32_t) (found + 4 + offset), *((uint32_t *) (carray + offset)));

#ifdef EXTENDED_DEBUG
	debug("Data flash dump (last write to %p\n\n", found);
	for (i = 0; i < FLASH_DATA_PAGES; i++) {
		for (offset = 0; offset < FLASH_DATA_PAGE_SIZE; offset++) {
			if ((offset % 8) == 0)
				debug("\n%p ", offset+FLASHPAGE(i));
			debug("x%02x ", *((uint8_t *) FLASHPAGE(i) + offset));
		}
		debug("\n\n");
	}
#endif

	// Uh-oh
	if (FlashData_Checksum(carray) != FlashData_Checksum(found+4)) {
		HAL_FLASH_Lock();
		debug("Checksums don't match after flash write! Possible flash wearout.");
		return;
	}

	CLEAR_BIT(FLASH->CR, FLASH_CR_PER); // ???

	// Erase other pages (with check if already erased)
	for (i = 0; i < FLASH_DATA_PAGES; i++)
		if (i != page) {
			debug("Check flash page %x (%d) for erase\n", FLASHPAGE(i), i);
			FlashData_Erase(FLASHPAGE(i));
		}

	HAL_FLASH_Lock();
} // FlashData_Write
