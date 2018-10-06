// ----------------------------------------------------------------------------
// stm_flash.c/h
// ----------------------------------------------------------------------------
// Copyright (c) 2017-2018, Jacob Peng.
// 
// Application Description:
// ========================
// Thrust Tester.
// Sample voltage, current, thrust, ppm, rpm and vibration;
// Calculate thrust efficiency and power.
//
// MCU Resources:
// ==============
// STM32F103C8T6-48pin, ARM Cortex M3, 72MHz SYSCLK, 64K ROM, 20K RAM, 37 GPIOs, 
// 16 12-bit ADCs, 3 USARTs, 2 SPIs, 2 I2Cs, 4 timers, 1 iwdg, 1 wwdg, swd supported.
// 
// File Description:
// =================
// Read and write inner flash.
//
// Revision:
// =========
// Release A0.0.1 - 2018/05/08 - Jacob Peng
//  - Add: Read and write inner flash.
//

// ----------------------------------------------------------------------------
// Includes
// ----------------------------------------------------------------------------
#include "stm_flash.h"

#include <string.h>

#include "stm32f1xx_hal.h"

// ----------------------------------------------------------------------------
// Runtime Subroutines
// ----------------------------------------------------------------------------

// ----------------------------------------------------------------------------
// bool STMFLASH_ReadBuffer(uint32_t, uint16_t [], uint16_t)
// ----------------------------------------------------------------------------
// Return Value : 0, failed; 1, successfully.
// Parameters   : 
//  addr  - start address, must be multiple of 2.
//  buf   - store address of read half word data.
//  len   - read length
// Description  : Read half word data with specified length from inner flash.
// ----------------------------------------------------------------------------
bool STMFLASH_ReadBuffer(uint32_t addr, uint16_t buf[], uint16_t len) {
  if (addr%2) return false;

  for (uint16_t i = 0; i < len; ++i) {
    buf[i] = *(__IO uint16_t *)addr;
    addr += 2;
  } // for

  return true;
} // STMFLASH_ReadBuffer()

static bool STMFLASH_WriteBufferNoCheck(uint32_t addr, 
                                        const uint16_t buf[], 
                                        uint16_t len) {
  if (addr%2) return false;

  for (uint16_t i = 0; i < len; ++i) {
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, 
                          addr, 
                          (uint64_t)buf[i]) == HAL_OK) {
      addr += 2;
    } else {
      return false;
    } // if else
  } // for

  return true;
} // STMFLASH_WriteBufferNoCheck()

// ----------------------------------------------------------------------------
// bool STMFLASH_WriteBuffer(uint32_t, uint16_t [], uint16_t)
// ----------------------------------------------------------------------------
// Return Value : 0, failed; 1, successfully.
// Parameters   : 
//  addr  - start address, must be multiple of 2.
//  buf   - store address of writed half word data.
//  len   - write length
// Description  : Write half word data with specified length to inner flash.
// ----------------------------------------------------------------------------
bool STMFLASH_WriteBuffer(uint32_t addr, const uint16_t buf[], uint16_t len) {
  if ((addr < ADDR_FLASH_PAGE_0) || 
      (addr >= FLASH_USER_END_ADDR) ||
      (addr%2)) return false;

  bool result = true;
  static uint16_t s_sector_buf[FLASH_PAGE_SIZE >> 1] = {0};
  uint16_t *p_write = (uint16_t *)buf;
  uint32_t offset_addr = addr - ADDR_FLASH_PAGE_0;
  uint8_t  sector_num  = (uint8_t)(offset_addr/FLASH_PAGE_SIZE);
  uint16_t sector_offset_size = (offset_addr%FLASH_PAGE_SIZE) >> 1; // half word.
  uint16_t sector_remain_size = (FLASH_PAGE_SIZE >> 1) - sector_offset_size;
  if (len < sector_remain_size) sector_remain_size = len;
  // Unlock the Flash to enable the flash control register access.
  (void)HAL_FLASH_Unlock();
  for (;;) {
    // Read the whole sector.
    if (!STMFLASH_ReadBuffer(sector_num*FLASH_PAGE_SIZE + ADDR_FLASH_PAGE_0,
                             s_sector_buf,
                             FLASH_PAGE_SIZE >> 1)) {
      result = false;
      break;
    } // if
    uint16_t i;
    // Check whether remained zone is empty or not.
    for (i = 0; i < sector_remain_size; ++i) {
      // Remained zone is not empty, requres to erase the whole sector.
      if (s_sector_buf[sector_offset_size + i] != 0xFFFF) break;
    } // for
    if (i >= sector_remain_size) {
      // The whole sector is empty, write data directly.
      if (!STMFLASH_WriteBufferNoCheck(addr, p_write, sector_remain_size)) {
        result = false;
        break;
      } // if
    } else {
      // Erase the whole sector.
      uint32_t page_error = 0;
      FLASH_EraseInitTypeDef EraseInitStruct;

      EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
      EraseInitStruct.PageAddress = sector_num*FLASH_PAGE_SIZE + ADDR_FLASH_PAGE_0;
      EraseInitStruct.NbPages     = 1;
      if (HAL_FLASHEx_Erase(&EraseInitStruct, &page_error) != HAL_OK) {
        result = false;
        break;
      } // if
      // Copy write data to buffer.      
      // for (i = 0; i < sector_remain_size; ++i) {
      //   s_sector_buf[sector_offset_size + i] = p_write[i];
      // } // for
      memcpy(s_sector_buf + sector_offset_size, p_write, sector_remain_size);
      // Write the whole sector.
      if (!STMFLASH_WriteBufferNoCheck(
            sector_num*FLASH_PAGE_SIZE + ADDR_FLASH_PAGE_0,
            s_sector_buf,
            FLASH_PAGE_SIZE >> 1)) {
        result = false;
        break;
      } // if
    } // if else
    if (sector_remain_size == len) {
      // Finished to write all data, len <= sector_remain_size.
      break;
    } else {
      // Unfinished to write, len > sector_remain_size.
      ++sector_num;
      sector_offset_size = 0;
      p_write += sector_remain_size;
      len -= sector_remain_size;
      sector_remain_size = (len < (FLASH_PAGE_SIZE >> 1)) ? len :
        (FLASH_PAGE_SIZE >> 1);
    } // if else    
  } // for
  // Lock the Flash to disable the flash control register access (recommended
  // to protect the FLASH memory against possible unwanted operation).
  (void)HAL_FLASH_Lock();

  return result;
} // STMFLASH_WriteBuffer()

// ----------------------------------------------------------------------------
// End Of File
// ----------------------------------------------------------------------------
