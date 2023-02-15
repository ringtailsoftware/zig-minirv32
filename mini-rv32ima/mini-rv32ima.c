// Copyright 2022 Charles Lohr, you may use this file or any portions herein under any of the BSD, MIT, or CC0 licenses.

#include <stdint.h>

int fail_on_all_faults = 0;

uint32_t HandleException( uint32_t ir, uint32_t retval );
uint32_t HandleControlStore( uint32_t addy, uint32_t val );
uint32_t HandleControlLoad( uint32_t addy );
void HandleOtherCSRWrite( uint8_t * image, uint16_t csrno, uint32_t value );

// This is the functionality we want to override in the emulator.
//  think of this as the way the emulator's processor is connected to the outside world.
//#define MINIRV32WARN( x... ) printf( x );
//#define MINIRV32_DECORATE  static
//#define MINI_RV32_RAM_SIZE ram_amt
#define MINIRV32_IMPLEMENTATION
#define MINIRV32_POSTEXEC( pc, ir, retval ) { if( retval > 0 ) { if( fail_on_all_faults ) { /*printf( "FAULT\n" );*/ return 3; } else retval = HandleException( ir, retval ); } }
#define MINIRV32_HANDLE_MEM_STORE_CONTROL( addy, val ) if( HandleControlStore( addy, val ) ) return val;
#define MINIRV32_HANDLE_MEM_LOAD_CONTROL( addy, rval ) rval = HandleControlLoad( addy );
#define MINIRV32_OTHERCSR_WRITE( csrno, value ) HandleOtherCSRWrite( image, csrno, value );

#include "mini-rv32ima.h"



