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
#define _Out_writes_bytes_(x)
#define _Out_writes_bytes_all_(x)
#define _Outptr_
#define _Out_
#define _Success_(x)
#define _Analysis_assume_(x)
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
// Validate float and double sizes for memory load/store and raw access.
//
_Static_assert( sizeof( RV_FLOAT ) == sizeof( RV_UINT32 ), "Invalid single-precision float size." );
_Static_assert( sizeof( RV_DOUBLE ) == sizeof( RV_UINT64 ), "Invalid double-precision float size." );

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
// Shared PTE flags (temporary).
//
#define RV_PTE_FLAG_R (1ul << 1)
#define RV_PTE_FLAG_W (1ul << 2)
#define RV_PTE_FLAG_X (1ul << 3)
#define RV_PTE_FLAG_U (1ul << 4)
#define RV_PTE_FLAG_G (1ul << 5)
#define RV_PTE_FLAG_A (1ul << 6)
#define RV_PTE_FLAG_D (1ul << 7)

//
// SV48 definitions.
//

//
// SV48 VA.
//

#define RV_SV48_VA_PGOFF_SHIFT (0ull)
#define RV_SV48_VA_PGOFF_MASK  ((1ull << (11 - 0 + 1)) - 1)

#define RV_SV48_VA_VPN0_SHIFT  (12ull)
#define RV_SV48_VA_VPN0_MASK   ((1ull << (20 - 12 + 1)) - 1)
							   
#define RV_SV48_VA_VPN1_SHIFT  (21ull)
#define RV_SV48_VA_VPN1_MASK   ((1ull << (29 - 21 + 1)) - 1)
							   
#define RV_SV48_VA_VPN2_SHIFT  (30ull)
#define RV_SV48_VA_VPN2_MASK   ((1ull << (38 - 30 + 1)) - 1)
							   
#define RV_SV48_VA_VPN3_SHIFT  (39ull)
#define RV_SV48_VA_VPN3_MASK   ((1ull << (47 - 39 + 1)) - 1)
							   
#define RV_SV48_VA_VPN_BIT_COUNT (9ull)

//
// SV48 PA.
//

#define RV_SV48_PA_PGOFF_SHIFT (0ull)
#define RV_SV48_PA_PGOFF_MASK  ((1ull << (11 - 0 + 1)) - 1)

#define RV_SV48_PA_PPN0_SHIFT  (12ull)
#define RV_SV48_PA_PPN0_MASK   ((1ull << (20 - 12 + 1)) - 1)
							   
#define RV_SV48_PA_PPN1_SHIFT  (21ull)
#define RV_SV48_PA_PPN1_MASK   ((1ull << (29 - 21 + 1)) - 1)
							   
#define RV_SV48_PA_PPN2_SHIFT  (30ull)
#define RV_SV48_PA_PPN2_MASK   ((1ull << (38 - 30 + 1)) - 1)
							   
#define RV_SV48_PA_PPN3_SHIFT  (39ull)
#define RV_SV48_PA_PPN3_MASK   ((1ull << (55 - 39 + 1)) - 1)

//
// SV48 PTE.
//

#define RV_SV48_PTE_V_SHIFT         (0ull) /* (V) Valid bit. */ 
#define RV_SV48_PTE_V_MASK          (1ull)
#define RV_SV48_PTE_V_FLAG          (RV_SV48_PTE_V_MASK << RV_SV48_PTE_V_SHIFT)

#define RV_SV48_PTE_R_SHIFT         (1ull) /* (R) Read access bit. */
#define RV_SV48_PTE_R_MASK          (1ull)
#define RV_SV48_PTE_R_FLAG          (RV_SV48_PTE_R_MASK << RV_SV48_PTE_R_SHIFT)

#define RV_SV48_PTE_W_SHIFT         (2ull) /* (W) Write access bit. */
#define RV_SV48_PTE_W_MASK          (1ull)
#define RV_SV48_PTE_W_FLAG          (RV_SV48_PTE_W_MASK << RV_SV48_PTE_W_SHIFT)

#define RV_SV48_PTE_X_SHIFT         (3ull) /* (X) Execute access bit. */
#define RV_SV48_PTE_X_MASK          (1ull)
#define RV_SV48_PTE_X_FLAG          (RV_SV48_PTE_X_MASK << RV_SV48_PTE_X_SHIFT)

#define RV_SV48_PTE_U_SHIFT         (4ull) /* (U) User-mode access bit. */
#define RV_SV48_PTE_U_MASK          (1ull)
#define RV_SV48_PTE_U_FLAG          (RV_SV48_PTE_U_MASK << RV_SV48_PTE_U_SHIFT)

#define RV_SV48_PTE_ACCESS_FLAGS    (RV_SV48_PTE_R_FLAG | RV_SV48_PTE_W_FLAG | RV_SV48_PTE_X_FLAG | RV_SV48_PTE_U_FLAG)

#define RV_SV48_PTE_G_SHIFT         (5ull) /* (G) Global mapping bit. */
#define RV_SV48_PTE_G_MASK          (1ull)
#define RV_SV48_PTE_G_FLAG          (RV_SV48_PTE_G_MASK << RV_SV48_PTE_G_SHIFT)

#define RV_SV48_PTE_A_SHIFT         (6ull) /* (A) Accessed bit. */
#define RV_SV48_PTE_A_MASK          (1ull)
#define RV_SV48_PTE_A_FLAG          (RV_SV48_PTE_A_MASK << RV_SV48_PTE_A_SHIFT)

#define RV_SV48_PTE_D_SHIFT         (7ull) /* (D) Dirty bit. */
#define RV_SV48_PTE_D_MASK          (1ull)
#define RV_SV48_PTE_D_FLAG          (RV_SV48_PTE_D_MASK << RV_SV48_PTE_D_SHIFT)

#define RV_SV48_PTE_RSW_SHIFT       (8ull) /* (RSW) Reserved for use by supervisor software. */
#define RV_SV48_PTE_RSW_MASK        ((1ull << (9 - 8 + 1)) - 1)

#define RV_SV48_PTE_PPN0_SHIFT      (10ull) /* (PPN0) Physical page number part 0. */
#define RV_SV48_PTE_PPN0_MASK       ((1ull << (18 - 10 + 1)) - 1)

#define RV_SV48_PTE_PPN1_SHIFT      (19ull) /* (PPN1) Physical page number part 1. */
#define RV_SV48_PTE_PPN1_MASK       ((1ull << (27 - 19 + 1)) - 1)

#define RV_SV48_PTE_PPN2_SHIFT      (28ull) /* (PPN2) Physical page number part 2. */
#define RV_SV48_PTE_PPN2_MASK       ((1ull << (36 - 28 + 1)) - 1)

#define RV_SV48_PTE_PPN3_SHIFT      (37ull) /* (PPN3) Physical page number part 3. */
#define RV_SV48_PTE_PPN3_MASK       ((1ull << (53 - 37 + 1)) - 1)

#define RV_SV48_PTE_RESERVED0_SHIFT (54ull) /* Reserved. */
#define RV_SV48_PTE_RESERVED0_MASK  ((1ull << (60 - 54 + 1)) - 1)

#define RV_SV48_PTE_PBMT_SHIFT      (61ull) /* (PBMT) Attributes for Svbpmt page-based memory types. */
#define RV_SV48_PTE_PBMT_MASK       ((1ull << (62 - 61 + 1)) - 1)
							        
#define RV_SV48_PTE_N_SHIFT         (63ull) /* (N) Svnapot. */
#define RV_SV48_PTE_N_MASK          (1ul)
#define RV_SV48_PTE_N_FLAG          (RV_SV48_PTE_N_MASK << RV_SV48_PTE_N_SHIFT)

#define RV_SV47_PTE_FULL_PPN_MASK ( ( RV_SV48_PTE_PPN0_MASK << (0*8)) \
									| ( RV_SV48_PTE_PPN1_MASK << (1*8) ) \
									| ( RV_SV48_PTE_PPN2_MASK << (2*8) ) \
									| ( RV_SV48_PTE_PPN3_MASK << (3*8) ) )

#define RV_SV48_PTE_FULL_PPN(PteValue) \
	(((PteValue) >> RV_SV48_PTE_PPN0_SHIFT) & RV_SV47_PTE_FULL_PPN_MASK)

//
// SV48 page table parameters (page size, level count, PTE size).
//
#define RV_SV48_PAGESIZE (0x1000ull)
#define RV_SV48_LEVELS   (4u)
#define RV_SV48_PTESIZE  (8ull)

//
// MMU radix tree page table walk result.
//
typedef struct _RV_MMU_TREE_WALK_RESULT {
	RV_UINT IsPresent : 1;
	RV_UINT IsAccessFault : 1;
	RV_UINT Level : 5;

	//
	// (IsPresent == 1).
	//
	RV_UINT64 LeafPtePpn;
	RV_UINT64 LeafPageSize;
	RV_UINT64 PhysicalAddress;
} RV_MMU_TREE_WALK_RESULT;

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
	// A flat contiguous span of memory accessible to the guest processor.
	// This currently takes precedence over the geust-to-host PTs.
	//
	RV_UINTR  MmuVaSpanGuestBase;
	VOID*     MmuVaSpanHostBase;
	RV_SIZE_T MmuVaSpanSize;

	//
	// Memory accessible to the guest processor through SV48 guest-to-host translation tables.
	//
#if defined(RV_OPT_MMU_GTH_PAGING)
	RV_UINT64  MmuGuestToHostPtPpn;
	RV_BOOLEAN MmuUseGuestToHostPt;
#endif

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
// Perform SV48 page table lookup.
// Note: this function is currently only used for guest-to-host mapping.
//
#if defined(RV_OPT_MMU_GTH_PAGING)
RV_MMU_TREE_WALK_RESULT
RvpMmuPtSv48TreeWalk(
	_Inout_ RV_PROCESSOR* Vp,
	_In_    RV_UINT64     TableRootPpn,
	_In_    RV_UINT64     LookupVa,
	_In_    RV_UINT32     AccessFlags
	);
#endif

//
// General processor tick execution function (update values/timers, fetch and execute a instruction, etc).
//
VOID
RvpTickExecute(
	_Inout_ RV_PROCESSOR* Vp
	);