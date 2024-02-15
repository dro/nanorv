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
#define _In_
#define _In_
#define _Outptr_
#define _Out_
#define _Success_(x)
#endif

//
// Allow usage of uint128 types by default on GCC/Clang.
//
#if defined(RV_OPT_BUILD_GCC) || defined(RV_OPT_BUILD_CLANG)
#define RV_OPT_BUILD_INT128_TYPES
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
// Convert from host-endianness to big-endian (and vice versa).
//
#ifdef RV_OPT_HOST_BIG_ENDIAN
#define RV_BIG_ENDIAN(x) (x)
#else
#define RV_BIG_ENDIAN(x) RV_BYTESWAP32((x))
#endif

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
	RV_FLOAT  Fr[ RVE_MAX_FPR_COUNT ]; /* Floating-point register values. */
	RV_UINT32 HostFpuCsr;
#endif

	RV_UINTR  CsrFFlags; /* Floating-Point Accrued Exceptions. */
	RV_UINTR  CsrFrm;    /* Floating-Point Dynamic Rounding Mode. */
	RV_UINTR  CsrFcsr;   /* Floating-Point Control and Status Register (frm + fflags). */

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