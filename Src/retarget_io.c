// ----------------------------------------------------------------------------
// retarget_io.c/h
// ----------------------------------------------------------------------------
// Copyright (c) 2017-2018, Jacob Peng.
//
// File Description:
// =================
// Retargets the C library function.
//
// Revision:
// =========
// Release A0.0.1 - 2018/01/26 - Jacob Peng
//  - Add: Retartgeted fputc() to the UART.
//

// ----------------------------------------------------------------------------
// Includes
// ----------------------------------------------------------------------------
#include "retarget_io.h"

#include "usart.h"

// ----------------------------------------------------------------------------
// Runtime Subroutines
// ----------------------------------------------------------------------------

// ----------------------------------------------------------------------------
// void fputc(int, FILE*)
// ----------------------------------------------------------------------------
//
// Return Value : None
// Parameters   :
//  ch  - data to send
//  f   - pointer of output file stream
// Description  : Retargets the C library fputc() to the UART.
//
// ----------------------------------------------------------------------------
PUTCHAR_PROTOTYPE {
#if defined (__CC_ARM)
  (void)f;
#endif  
  UARTx_Putc(&UART_PRINTF, (uint8_t)ch);

  return ch;
} // fputc()

// ----------------------------------------------------------------------------
// int fgetc(FILE *)
// ----------------------------------------------------------------------------
//
// Return Value : read data
// Parameters   :
//  f   - pointer of input file stream
// Description  : Retargets the C library fgetc() to the UART.
//
// ----------------------------------------------------------------------------
// GETCHAR_PROTOTYPE {
//   uint8_t ch = 0;

//   (void)f; 
//   (void)HAL_UART_Receive(&UART_PRINTF, &ch, 1, 0xFFFF);

//   return ch;
// } // fgetc()

// ----------------------------------------------------------------------------
// End Of File
// ----------------------------------------------------------------------------
