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

#ifndef RETARGET_IO_H_
#define RETARGET_IO_H_

// ----------------------------------------------------------------------------
// Includes
// ----------------------------------------------------------------------------
#include <stdio.h>

// ----------------------------------------------------------------------------
// High Layer Function
// ----------------------------------------------------------------------------
#ifdef __GNUC__
  // With GCC, small printf (option LD Linker->Libraries->Small printf
  // set to 'Yes') calls __io_putchar() 
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
  // #define GETCHAR_PROTOTYPE int __io_getchar(FILE *f)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
  // #define GETCHAR_PROTOTYPE int fgetc(FILE *f)
#endif

// ----------------------------------------------------------------------------
// Medium Layer Function
// ----------------------------------------------------------------------------

// ----------------------------------------------------------------------------
// Low Layer Function
// ----------------------------------------------------------------------------

#endif  // #ifndef RETARGET_IO_H_
