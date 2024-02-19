#pragma once

#include <stdlib.h>
#include <stdint.h>
#include <limits.h>

//
// User-tunable compile-time options.
//

//
// Allow the user to specify a configuration header.
//
#ifdef RV_OPT_INCLUDE_CONFIG
#include "nanorv_config.h"
#endif

//
// Allow usage of SAL annotations.
//
#if defined(RV_OPT_BUILD_SAL) || defined(_SAL_VERSION)
#include <specstrings.h>
#include <sal.h>
#else
#define _Inout_
#define _Inout_bytecount_(x)
#define _In_
#define _In_reads_bytes_(x)
#define _Outptr_
#define _Out_
#define _Success_(x)
#endif

//
// Base types used by the emulator code.
//

typedef int8_t        RV_INT8;
typedef int16_t       RV_INT16;
typedef int32_t       RV_INT32;
typedef int64_t       RV_INT64;
typedef uint8_t       RV_UINT8;
typedef uint16_t      RV_UINT16;
typedef uint32_t      RV_UINT32;
typedef uint64_t      RV_UINT64;
typedef float         RV_FLOAT;
typedef double        RV_DOUBLE;
typedef uint8_t       RV_BOOLEAN;
typedef char          RV_CHAR;
typedef unsigned char RV_UCHAR;
typedef size_t        RV_SIZE_T;
typedef int           RV_INT;
typedef unsigned int  RV_UINT;

//
// Currently RV_OPT_RV32I must always be enabled as the base instruction set.
//
#define RV_OPT_RV32I

//
// GPR-size integer types.
//
#if defined(RV_OPT_RV64I)
typedef RV_UINT64 RV_UINTR;
typedef RV_INT64  RV_INTR;
#elif defined(RV_OPT_RV32I)
typedef RV_UINT32 RV_UINTR;
typedef RV_INT32  RV_INTR;
#else
#error "Unsupported XLEN mode."
#endif

//
// FP types.
//
#if defined(RV_OPT_RV32D)
typedef RV_DOUBLE RV_FLOATR;
#elif defined(RV_OPT_RV32F)
typedef RV_FLOAT RV_FLOATR;
#endif

//
// Raw FLOAT32 union, used to access raw binary encoding of a float.
//
typedef union _RV_FLOAT32_RAW {
	RV_FLOAT  F32;
	RV_UINT32 U32;
} RV_FLOAT32_RAW;

//
// Raw FLOAT64 union, used to access raw binary encoding of a double.
//
typedef union _RV_FLOAT64_RAW {
	RV_DOUBLE F64;
	RV_UINT64 U64;
} RV_FLOAT64_RAW;

//
// Helper defines used by the emulator code.
//

#ifndef VOID
#define VOID void
#endif

#ifndef UNREFERENCED_PARAMETER
#define UNREFERENCED_PARAMETER(x)
#endif

#define RV_TRUE       ((RV_BOOLEAN)(1==1))
#define RV_FALSE      ((RV_BOOLEAN)(0==1))
#define RV_COUNTOF(a) (sizeof((a)) / sizeof((a[0])))
#define RV_MAX(a,b)   (((a) > (b)) ? (a) : (b))
#define RV_MIN(a,b)   (((a) < (b)) ? (a) : (b))

//
// UINT32 byteswap helpers.
//
#ifdef RV_OPT_BUILD_MSVC
#define RV_BYTESWAP32(x) _byteswap_ulong((x))
#else
#define RV_BYTESWAP32(x) \
  ((((x) & 0x000000FFul) << 24ul) | \
  (((x)  & 0x0000FF00ul) << 8ul)  | \
  (((x)  & 0x00FF0000ul) >> 8ul)  | \
  (((x)  & 0xFF000000ul) >> 24ul))
#endif

//
// Convert from host-endianness to little-endian (and vice versa).
//
#ifdef RV_OPT_HOST_BIG_ENDIAN
#define RV_LITTLE_ENDIAN_32(x) RV_BYTESWAP32((x))
#else
#define RV_LITTLE_ENDIAN_32(x) (x)
#endif

//
// Inline function.
//
#define RV_FORCEINLINE __forceinline

//
// Standard ABI:
// Register   ABI Name   Description                        Saver
// x0         zero       Hard-wired zero                      -
// x1         ra         Return address                     Caller
// x2         sp         Stack pointer                      Callee
// x3         gp         Global pointer                       -
// x4         tp         Thread pointer                       -
// x5-7       t0-2       Temporaries                        Caller
// x8         s0/fp      Saved register/frame pointer       Callee
// x9         s1         Saved register                     Callee
// x10-11     a0-1       Function arguments/return values   Caller
// x12-17     a2-7       Function arguments                 Caller
// x18-27     s2-11      Saved registers                    Callee
// x28-31     t3-6       Temporaries                        Caller
// f0-7       ft0-7      FP temporaries                     Caller
// f8-9       fs0-1      FP saved registers                 Callee
// f10-11     fa0-1      FP arguments/return values         Caller
// f12-17     fa2-7      FP arguments                       Caller
// f18-27     fs2-11     FP saved registers                 Callee
// f28-31     ft8-11     FP temporaries                     Caller
//
#define RV_REG_ZERO 0
#define RV_REG_RA   1
#define RV_REG_SP   2
#define RV_REG_GP   3
#define RV_REG_TP   4
#define RV_REG_T0   5
#define RV_REG_T1   6
#define RV_REG_T2   7
#define RV_REG_S0   8
#define RV_REG_S1   9
#define RV_REG_A0   10
#define RV_REG_A1   11
#define RV_REG_A2   12
#define RV_REG_A3   13
#define RV_REG_A4   14
#define RV_REG_A5   15
#define RV_REG_A6   16
#define RV_REG_A7   17
#define RV_REG_S2   18
#define RV_REG_S3   19
#define RV_REG_S4   20
#define RV_REG_S5   21
#define RV_REG_S6   22
#define RV_REG_S7   23
#define RV_REG_S8   24
#define RV_REG_S9   25
#define RV_REG_S10  26
#define RV_REG_S11  27
#define RV_REG_T3   28
#define RV_REG_T4   29
#define RV_REG_T5   30
#define RV_REG_T6   31
#define RV_REG_FT0  0
#define RV_REG_FT1  1
#define RV_REG_FT2  2
#define RV_REG_FT3  3
#define RV_REG_FT4  4
#define RV_REG_FT5  5
#define RV_REG_FT6  6
#define RV_REG_FT7  7
#define RV_REG_FS0  8
#define RV_REG_FS1  9
#define RV_REG_FA0  10
#define RV_REG_FA1  11
#define RV_REG_FA2  12
#define RV_REG_FA3  13
#define RV_REG_FA4  14
#define RV_REG_FA5  15
#define RV_REG_FA6  16
#define RV_REG_FA7  17
#define RV_REG_FS2  18
#define RV_REG_FS3  19
#define RV_REG_FS4  20
#define RV_REG_FS5  21
#define RV_REG_FS6  22
#define RV_REG_FS7  23
#define RV_REG_FS8  24
#define RV_REG_FS9  25
#define RV_REG_FS10 26
#define RV_REG_FS11 27
#define RV_REG_FT8  28
#define RV_REG_FT9  29
#define RV_REG_FT10 30
#define RV_REG_FT11 31

//
// Max GPR (X register) and FPR (F register) counts.
//
#define RVE_MAX_GPR_COUNT (32)
#define RVE_MAX_FPR_COUNT (32)

//
// RV emulator processor (hart) context.
//

typedef struct _RV_PROCESSOR {
	//
	// General purpose register context.
	//
	RV_UINTR Pc;                      /* Program counter. */
	RV_UINTR Xr[ RVE_MAX_GPR_COUNT ]; /* General purpose registers values. */

	//
	// Floating-point context.
	//
#if (defined(RV_OPT_RV32F) || defined(RV_OPT_RV32D))
	RV_FLOATR Fr[ RVE_MAX_FPR_COUNT ]; /* Floating-point register values. */
	RV_UINTR  CsrFcsr;                 /* Floating-Point Control and Status Register (frm + fflags). */
#endif

	//
	// Memory accessible to the guest processor.
	// This is currently just a flat span of memory,
	// will be changed as development furthers.
	//
	RV_UINTR  VaSpanGuestBase;
	VOID*     VaSpanHostBase;
	RV_SIZE_T VaSpanSize;

	//
	// Timer states.
	//
	RV_UINT64 CsrCycleCount;  /* Cycle counter for RDCYCLE instruction. */
	RV_UINT64 CsrTime;		  /* Timer for RDTIME instruction. */
	RV_UINT64 CsrInstRetired; /* Instructions-retired counter for RDINSTRET instruction. */

	//
	// Per-tick flags and information.
	//
	RV_UINT   ExceptionPending : 1;
	RV_UINT   ECallPending : 1;
	RV_UINT   EBreakPending : 1;
	RV_UINT32 ExceptionIndex;
} RV_PROCESSOR;

//
// General processor tick execution functions (update values/timers, fetch and execute a instruction, etc).
//

VOID
RvpTickExecute(
	_Inout_ RV_PROCESSOR* Vp
	);