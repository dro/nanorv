#include "nanorv.h"

//
// User-tunable compile-time options.
//


//
// Allow usage of MSVC-specific extensions/intrinsics.
//
#ifdef RV_OPT_BUILD_MSVC
#include <intrin.h>
#endif

//
// Allow usage of CRT/LibC for certain (math) functions.
//
#ifdef RV_OPT_BUILD_LIBC
#include <math.h>
#include <float.h>
#endif

//
// Allow usage of x86 SSE intrinsics.
//
#ifdef RV_OPT_BUILD_SSE
#include <intrin.h>
#include <xmmintrin.h>
#endif

//
// Set up builtin int128 types if supported.
//
#ifdef RV_OPT_BUILD_INT128_TYPES
#define int128_t  __int128
#define uint128_t unsigned __int128
#endif

//
// Allow usage of C11 std atomic functions.
//
#if defined(RV_OPT_BUILD_STD_ATOMIC)
#include <stdatomic.h>
#endif

//
// Relaxed memory-order atomic aligned store.
//
#if defined(RV_OPT_HOST_ATOMIC_ALIGNED_ACCESS)
	#define RV_RELAXED_ATOMIC_STORE8(Destination, Value) \
		( *( volatile RV_UINT8* )(Destination) = (Value) )
	#define RV_RELAXED_ATOMIC_STORE16(Destination, Value) \
		( *( volatile RV_UINT16* )(Destination) = (Value) )
	#define RV_RELAXED_ATOMIC_STORE32(Destination, Value) \
		( *( volatile RV_UINT32* )(Destination) = (Value) )
	#define RV_RELAXED_ATOMIC_STORE64(Destination, Value) \
		( *( volatile RV_UINT64* )(Destination) = (Value) )
#elif defined(RV_OPT_BUILD_STD_ATOMIC)
	#define RV_RELAXED_ATOMIC_STORE8(Destination, Value) \
		atomic_exchange_explicit( ( volatile _Atomic(RV_UINT8)* )(Destination), (Value), memory_order_relaxed )
	#define RV_RELAXED_ATOMIC_STORE16(Destination, Value) \
		atomic_exchange_explicit( ( volatile _Atomic(RV_UINT16)* )(Destination), (Value), memory_order_relaxed )
	#define RV_RELAXED_ATOMIC_STORE32(Destination, Value) \
		atomic_exchange_explicit( ( volatile _Atomic(RV_UINT32)* )(Destination), (Value), memory_order_relaxed )
	#define RV_RELAXED_ATOMIC_STORE64(Destination, Value) \
		atomic_exchange_explicit( ( volatile _Atomic(RV_UINT64)* )(Destination), (Value), memory_order_relaxed )
#elif defined(RV_OPT_BUILD_MSVC)
	_Static_assert( sizeof( RV_UINT8 ) == sizeof( char ), "Invalid RV_UINT8 size." );
	_Static_assert( sizeof( RV_UINT16 ) == sizeof( short ), "Invalid RV_UINT16 size." );
	_Static_assert( sizeof( RV_UINT32 ) == sizeof( long ), "Invalid RV_UINT32 size." );
	_Static_assert( sizeof( RV_UINT64 ) == sizeof( long long ), "Invalid RV_UINT64 size." );
	#define RV_RELAXED_ATOMIC_STORE8(Destination, Value) \
		_InterlockedExchange8( ( volatile char* )(Destination), (Value) )
	#define RV_RELAXED_ATOMIC_STORE16(Destination, Value) \
		_InterlockedExchange16( ( volatile short* )(Destination), (Value) )
	#define RV_RELAXED_ATOMIC_STORE32(Destination, Value) \
		_InterlockedExchange( ( volatile long* )(Destination), (Value) );
	#define RV_RELAXED_ATOMIC_STORE64(Destination, Value) \
		_InterlockedExchange64( ( volatile long long* )(Destination), (Value) );
#else
	#error "Unsupported."
#endif

//
// Relaxed memory-order atomic aligned load.
//
#if defined(RV_OPT_HOST_ATOMIC_ALIGNED_ACCESS)
	#define RV_RELAXED_ATOMIC_LOAD8(Source) \
		( *( volatile RV_UINT8* )(Source) )
	#define RV_RELAXED_ATOMIC_LOAD16(Source) \
		( *( volatile RV_UINT16* )(Source) )
	#define RV_RELAXED_ATOMIC_LOAD32(Source) \
		( *( volatile RV_UINT32* )(Source) )
	#define RV_RELAXED_ATOMIC_LOAD64(Source) \
		( *( volatile RV_UINT64* )(Source) )
#elif defined(RV_OPT_BUILD_STD_ATOMIC)
	#define RV_RELAXED_ATOMIC_LOAD8(Source) \
		atomic_fetch_add_explicit( ( volatile _Atomic(RV_UINT8)* )(Source), 0, memory_order_relaxed )
	#define RV_RELAXED_ATOMIC_LOAD16(Source) \
		atomic_fetch_add_explicit( ( volatile _Atomic(RV_UINT16)* )(Source), 0, memory_order_relaxed )
	#define RV_RELAXED_ATOMIC_LOAD32(Source) \
		atomic_fetch_add_explicit( ( volatile _Atomic(RV_UINT32)* )(Source), 0, memory_order_relaxed )
	#define RV_RELAXED_ATOMIC_LOAD64(Source) \
		atomic_fetch_add_explicit( ( volatile _Atomic(RV_UINT64)* )(Source), 0, memory_order_relaxed )
#elif defined(RV_OPT_BUILD_MSVC)
	_Static_assert( sizeof( RV_UINT8 ) == sizeof( char ), "Invalid RV_UINT8 size." );
	_Static_assert( sizeof( RV_UINT16 ) == sizeof( short ), "Invalid RV_UINT16 size." );
	_Static_assert( sizeof( RV_UINT32 ) == sizeof( long ), "Invalid RV_UINT32 size." );
	_Static_assert( sizeof( RV_UINT64 ) == sizeof( long long ), "Invalid RV_UINT64 size." );

#else
	#error "Unsupported."
#endif


//
// Internal MMU types/definitions.
//


#define RV_MMU_ACCESS_TYPE_LOAD    0
#define RV_MMU_ACCESS_TYPE_STORE   1
#define RV_MMU_ACCESS_TYPE_EXECUTE 2

typedef enum _RV_MMU_ACCESS_SIZE {
	RV_MMU_ACCESS_SIZE_BYTE        = 1,
	RV_MMU_ACCESS_SIZE_HALF_WORD   = 2,
	RV_MMU_ACCESS_SIZE_WORD        = 4,
	RV_MMU_ACCESS_SIZE_DOUBLE_WORD = 8
} RV_MMU_ACCESS_SIZE;

typedef union _RV_MEM_VALUE {
	RV_UINT8  AsU8;
	RV_UINT16 AsU16;
	RV_UINT32 AsU32;
	RV_UINT64 AsU64;
#if defined(RV_OPT_RV32F) || defined(RV_OPT_RV32D)
	RV_FLOAT  AsFloat;
	RV_DOUBLE AsDouble;
#endif
} RV_MEM_VALUE;

typedef struct _RV_MMU_ACCESS_CHUNK {
	VOID*     Data;
	RV_SIZE_T Size;
} RV_MMU_ACCESS_CHUNK;

#define RV_MMU_MIN_PAGESIZE 0x1000


//
// Instruction formats/instruction encoding helpers.
//


//
// RV32I instruction format R bit positions/masks.
//
#define RV_INST_R_FUNCT7_SHIFT      (25ul)
#define RV_INST_R_FUNCT7_MASK       ((1ul << (31 - 25 + 1)) - 1)

#define RV_INST_R_RS2_SHIFT         (20ul)
#define RV_INST_R_RS2_MASK          ((1ul << (24 - 20 + 1)) - 1)

#define RV_INST_R_RS1_SHIFT         (15ul)
#define RV_INST_R_RS1_MASK          ((1ul << (19 - 15 + 1)) - 1)

#define RV_INST_R_FUNCT3_SHIFT      (12ul)
#define RV_INST_R_FUNCT3_MASK       ((1ul << (14 - 12 + 1)) - 1)

#define RV_INST_R_RD_SHIFT          (7ul)
#define RV_INST_R_RD_MASK           ((1ul << (11 - 7 + 1)) - 1)
								    
#define RV_INST_R_OPCODE_SHIFT      (0ul)
#define RV_INST_R_OPCODE_MASK       ((1ul << (6 - 0 + 1)) - 1)

//
// RV32I instruction format I bit positions/masks.
//
#define RV_INST_I_IMM_11_0_SHIFT      (20ul)
#define RV_INST_I_IMM_11_0_MASK       ((1ul << (31 - 20 + 1)) - 1)

#define RV_INST_I_RS1_SHIFT           (15ul)
#define RV_INST_I_RS1_MASK            ((1ul << (19 - 15 + 1)) - 1)
									  
#define RV_INST_I_FUNCT3_SHIFT        (12ul)
#define RV_INST_I_FUNCT3_MASK         ((1ul << (14 - 12 + 1)) - 1)
									  
#define RV_INST_I_RD_SHIFT            (7ul)
#define RV_INST_I_RD_MASK             ((1ul << (11 - 7 + 1)) - 1)
									  
#define RV_INST_I_OPCODE_SHIFT        (0ul)
#define RV_INST_I_OPCODE_MASK         ((1ul << (6 - 0 + 1)) - 1)

//
// RV32I instruction format S bit positions/masks.
//
#define RV_INST_S_IMM_11_5_SHIFT    (25ul)
#define RV_INST_S_IMM_11_5_MASK     ((1ul << (31 - 25 + 1)) - 1)

#define RV_INST_S_RS2_SHIFT         (20ul)
#define RV_INST_S_RS2_MASK          ((1ul << (24 - 20 + 1)) - 1)

#define RV_INST_S_RS1_SHIFT         (15ul)
#define RV_INST_S_RS1_MASK          ((1ul << (19 - 15 + 1)) - 1)

#define RV_INST_S_FUNCT3_SHIFT      (12ul)
#define RV_INST_S_FUNCT3_MASK       ((1ul << (14 - 12 + 1)) - 1)

#define RV_INST_S_IMM_4_0_SHIFT     (7ul)
#define RV_INST_S_IMM_4_0_MASK      ((1ul << (11 - 7 + 1)) - 1)

#define RV_INST_S_OPCODE_SHIFT      (0ul)
#define RV_INST_S_OPCODE_MASK       ((1ul << (6 - 0 + 1)) - 1)

//
// RV32I instruction format U bit positions/masks.
//
#define RV_INST_U_IMM_31_12_SHIFT (12ul)
#define RV_INST_U_IMM_31_12_MASK  ((1ul << (31 - 12 + 1)) - 1)

#define RV_INST_U_RD_SHIFT        (7ul)
#define RV_INST_U_RD_MASK         ((1ul << (11 - 7 + 1)) - 1)

#define RV_INST_U_OPCODE_SHIFT    (0ul)
#define RV_INST_U_OPCODE_MASK     ((1ul << (6 - 0 + 1)) - 1)

//
// RV32I instruction format B bit positions/masks.
//
#define RV_INST_B_IMM_12_SHIFT   (31ul)
#define RV_INST_B_IMM_12_MASK    (1ul)

#define RV_INST_B_IMM_10_5_SHIFT (25ul)
#define RV_INST_B_IMM_10_5_MASK  ((1ul << (30 - 25 + 1)) - 1)

#define RV_INST_B_RS2_SHIFT      (20ul)
#define RV_INST_B_RS2_MASK       ((1ul << (24 - 20 + 1)) - 1)

#define RV_INST_B_RS1_SHIFT      (15ul)
#define RV_INST_B_RS1_MASK       ((1ul << (19 - 15 + 1)) - 1)

#define RV_INST_B_FUNCT3_SHIFT   (12ul)
#define RV_INST_B_FUNCT3_MASK    ((1ul << (14 - 12 + 1)) - 1)

#define RV_INST_B_IMM_4_1_SHIFT  (8ul)
#define RV_INST_B_IMM_4_1_MASK   ((1ul << (11 - 8 + 1)) - 1)

#define RV_INST_B_IMM_11_SHIFT   (7ul)
#define RV_INST_B_IMM_11_MASK    1ul  

#define RV_INST_B_OPCODE_SHIFT   (0ul)
#define RV_INST_B_OPCODE_MASK    ((1ul << (6 - 0 + 1)) - 1)

//
// RV32I instruction format J bit positions/masks.
//
#define RV_INST_J_IMM_20_SHIFT    (31ul)
#define RV_INST_J_IMM_20_MASK     (1ul)

#define RV_INST_J_IMM_10_1_SHIFT  (21ul)
#define RV_INST_J_IMM_10_1_MASK   ((1ul << (30 - 21 + 1)) - 1)

#define RV_INST_J_IMM_11_SHIFT    (20ul)
#define RV_INST_J_IMM_11_MASK     (1ul)

#define RV_INST_J_IMM_19_12_SHIFT (12ul)
#define RV_INST_J_IMM_19_12_MASK  ((1ul << (19 - 12 + 1)) - 1)

#define RV_INST_J_RD_SHIFT        (7ul)
#define RV_INST_J_RD_MASK         ((1ul << (11 - 7 + 1)) - 1)

#define RV_INST_J_OPCODE_SHIFT    (0ul)
#define RV_INST_J_OPCODE_MASK     ((1ul << (6 - 0 + 1)) - 1)

//
// RISC-V primary opcode map.
//
#define RV_OPCODE_LOAD       (3)   /* (0b00'000'11) */
#define RV_OPCODE_LOAD_FP    (7)   /* (0b00'001'11) */
#define RV_OPCODE_CUSTOM0    (11)  /* (0b00'010'11) */
#define RV_OPCODE_MISC_MEM   (15)  /* (0b00'011'11) */
#define RV_OPCODE_OP_IMM     (19)  /* (0b00'100'11) */
#define RV_OPCODE_AUIPC      (23)  /* (0b00'101'11) */
#define RV_OPCODE_OP_IMM32   (27)  /* (0b00'110'11) */ 

#define RV_OPCODE_STORE      (35)  /* (0b01'000'11) */
#define RV_OPCODE_STORE_FP   (39)  /* (0b01'001'11) */
#define RV_OPCODE_CUSTOM1    (43)  /* (0b01'010'11) */
#define RV_OPCODE_AMO        (47)  /* (0b01'011'11) */
#define RV_OPCODE_OP         (51)  /* (0b01'100'11) */
#define RV_OPCODE_LUI        (55)  /* (0b01'101'11) */
#define RV_OPCODE_OP_32      (59)  /* (0b01'110'11) */

#define RV_OPCODE_MADD       (67)  /* (0b10'000'11) */
#define RV_OPCODE_MSUB       (71)  /* (0b10'001'11) */
#define RV_OPCODE_NMSUB      (75)  /* (0b10'010'11) */
#define RV_OPCODE_NMADD      (79)  /* (0b10'011'11) */
#define RV_OPCODE_OP_FP      (83)  /* (0b10'100'11) */
#define RV_OPCODE_RESERVED0  (87)  /* (0b10'101'11) */
#define RV_OPCODE_CUSTOM2    (91)  /* (0b10'110'11) */

#define RV_OPCODE_BRANCH     (99)  /* (0b11'000'11) */
#define RV_OPCODE_JALR       (103) /* (0b11'001'11) */
#define RV_OPCODE_RESERVED1  (107) /* (0b11'010'11) */
#define RV_OPCODE_JAL        (111) /* (0b11'011'11) */
#define RV_OPCODE_SYSTEM     (115) /* (0b11'100'11) */
#define RV_OPCODE_RESERVED2  (119) /* (0b11'101'11) */
#define RV_OPCODE_CUSTOM3    (123) /* (0b11'110'11) */

//
// Instruction funct3 values.
//

//
// Base opcode RV_OPCODE_JALR funct3 values (I-type).
// imm[11:0] rs1 000 rd 1100111 JALR
//
#define RV_JALR_FUNCT3_BASE (0)

//
// Base opcode RV_OPCODE_BRANCH funct3 values (B-type).
//
#define RV_BRANCH_FUNCT3_BEQ  (0)
#define RV_BRANCH_FUNCT3_BNE  (1)
#define RV_BRANCH_FUNCT3_BLT  (4)
#define RV_BRANCH_FUNCT3_BGE  (5)
#define RV_BRANCH_FUNCT3_BLTU (6)
#define RV_BRANCH_FUNCT3_BGEU (7)

//
// Base opcode RV_OPCODE_LOAD funct3 values (I-type).
//
#define RV_LOAD_FUNCT3_LB  (0)
#define RV_LOAD_FUNCT3_LH  (1)
#define RV_LOAD_FUNCT3_LW  (2)
#define RV_LOAD_FUNCT3_LBU (4)
#define RV_LOAD_FUNCT3_LHU (5)
#define RV_LOAD_FUNCT3_LD  (3) /* RV64I */
#define RV_LOAD_FUNCT3_LWU (6) /* RV64I */

//
// Base opcode RV_OPCODE_STORE funct3 values (S-type).
//
#define RV_STORE_FUNCT3_SB (0)
#define RV_STORE_FUNCT3_SH (1)
#define RV_STORE_FUNCT3_SW (2)
#define RV_STORE_FUNCT3_SD (3) /* RV64I */


//
// Base opcode RV_OPCODE_OP_IMM funct3 values (I-type).
//
#define RV_OP_IMM_FUNCT3_ADDI  (0)
#define RV_OP_IMM_FUNCT3_SLTI  (2)
#define RV_OP_IMM_FUNCT3_SLTIU (3)
#define RV_OP_IMM_FUNCT3_XORI  (4)
#define RV_OP_IMM_FUNCT3_ORI   (6)
#define RV_OP_IMM_FUNCT3_ANDI  (7)

//
// Base opcode RV_OPCODE_OP_IMM funct3 values (R-type).
//
#define RV_OP_IMM_FUNCT3_SLLI      (1)
#define RV_OP_IMM_FUNCT3_SRLI_SRAI (5)

//
// Base opcode RV_OPCODE_OP funct3 values (R-type).
//
#define RV_OP_FUNCT3_ADD_SUB (0)
#define RV_OP_FUNCT3_SLL     (1)
#define RV_OP_FUNCT3_SLT     (2)
#define RV_OP_FUNCT3_SLTU    (3)
#define RV_OP_FUNCT3_XOR     (4)
#define RV_OP_FUNCT3_SRL_SRA (5)
#define RV_OP_FUNCT3_OR      (6)
#define RV_OP_FUNCT3_AND     (7)

//
// Base opcode RV_OPCODE_MISC_MEM funct3 values (I-type?)
//
#define RV_MISC_MEM_FUNCT3_FENCE (0)

//
// Base opcode RV_OPCODE_SYSTEM funct3 values.
//
#define RV_SYSTEM_FUNCT3_ECALL_EBREAK  (0)

//
// RV32/RV64 Zicsr Standard Extension RV_OPCODE_SYSTEM funct3 values.
//
#define RV_SYSTEM_FUNCT3_CSRRW  (1)
#define RV_SYSTEM_FUNCT3_CSRRS  (2)
#define RV_SYSTEM_FUNCT3_CSRRC  (3)
#define RV_SYSTEM_FUNCT3_CSRRWI (5)
#define RV_SYSTEM_FUNCT3_CSRRSI (6)
#define RV_SYSTEM_FUNCT3_CSRRCI (7)

//
// RV32M RV_OPCODE_OP funct3 values.
//
#define RV_OP_FUNCT3_RV32M_MUL    (0)
#define RV_OP_FUNCT3_RV32M_MULH   (1)
#define RV_OP_FUNCT3_RV32M_MULHSU (2)
#define RV_OP_FUNCT3_RV32M_MULHU  (3)
#define RV_OP_FUNCT3_RV32M_DIV    (4)
#define RV_OP_FUNCT3_RV32M_DIVU   (5)
#define RV_OP_FUNCT3_RV32M_REM    (6)
#define RV_OP_FUNCT3_RV32M_REMU   (7)

//
// RV32F RV_OPCODE_OP_FP funct3 values.
//
#define	RV_OP_FP_FUNCT3_FSGNJ_S	 (0)
#define	RV_OP_FP_FUNCT3_FSGNJN_S (1)
#define	RV_OP_FP_FUNCT3_FSGNJX_S (2)
#define	RV_OP_FP_FUNCT3_FMIN_S	 (0)
#define	RV_OP_FP_FUNCT3_FMAX_S	 (1)
#define	RV_OP_FP_FUNCT3_FMV_X_W	 (0)
#define	RV_OP_FP_FUNCT3_FEQ_S	 (2)
#define	RV_OP_FP_FUNCT3_FLT_S	 (1)
#define	RV_OP_FP_FUNCT3_FLE_S	 (0)
#define	RV_OP_FP_FUNCT3_FCLASS_S (1)
#define	RV_OP_FP_FUNCT3_FMV_W_X	 (0)

//
// RV64I RV_OPCODE_OP_IMM_32 funct3 values.
//
#define RV_OP_IMM_32_FUNCT3_ADDIW       (0)
#define RV_OP_IMM_32_FUNCT3_SLLIW       (1)
#define RV_OP_IMM_32_FUNCT3_SRLIW_SRAIW (5)

//
// RV64I RV_OPCODE_OP_32 funct3 values.
//
#define RVP_OP_32_FUNCT3_ADDW (0)
#define RVP_OP_32_FUNCT3_SUBW (0)
#define RVP_OP_32_FUNCT3_SLLW (1)
#define RVP_OP_32_FUNCT3_SRLW (5)
#define RVP_OP_32_FUNCT3_SRAW (5)

//
// RV64M RV_OPCODE_OP_32 funct3 values.
//
#define RV_OP_32_FUNCT3_RV64M_MULW  (0)
#define RV_OP_32_FUNCT3_RV64M_DIVW  (4)
#define RV_OP_32_FUNCT3_RV64M_DIVUW (5)
#define RV_OP_32_FUNCT3_RV64M_REMW  (6)
#define RV_OP_32_FUNCT3_RV64M_REMUW (7)

//
// Instruction funct7 values.
//

//
// Base opcode RV_OPCODE_OP_IMM funct7 values (R-type).
//
#define RV_OP_IMM_FUNCT7_SLLI (0)
#define RV_OP_IMM_FUNCT7_SRLI (0)
#define RV_OP_IMM_FUNCT7_SRAI (32)

//
// Base opcode RV_OPCODE_OP funct7 values (R-type).
//
#define RV_OP_FUNCT7_ADD   (0)
#define RV_OP_FUNCT7_SUB   (32)
#define RV_OP_FUNCT7_SLL   (0)
#define RV_OP_FUNCT7_SLT   (0)
#define RV_OP_FUNCT7_SLTU  (0)
#define RV_OP_FUNCT7_XOR   (0)
#define RV_OP_FUNCT7_SRL   (0)
#define RV_OP_FUNCT7_SRA   (32)
#define RV_OP_FUNCT7_OR	   (0)
#define RV_OP_FUNCT7_AND   (0)
#define RV_OP_FUNCT7_RV32M (1)

//
// RV_OPCODE_OP_IMM_32 funct7 values.
//
#define RV_OP_IMM_32_FUNCT7_SLLIW (0)
#define RV_OP_IMM_32_FUNCT7_SRLIW (0)
#define RV_OP_IMM_32_FUNCT7_SRAIW (32)

//
// RV_OPCODE_OP_32 funct7 values.
//
#define RV_OP_32_FUNCT7_ADDW  (0)
#define RV_OP_32_FUNCT7_SUBW  (32)
#define RV_OP_32_FUNCT7_SLLW  (0)
#define RV_OP_32_FUNCT7_SRLW  (0)
#define RV_OP_32_FUNCT7_SRAW  (32)
#define RV_OP_32_FUNCT7_RV64M (1)

//
// RV32F RV_OPCODE_OP_FP funct7 values.
//
#define RV_OP_FP_FUNCT7_FADD_S	          (0)
#define RV_OP_FP_FUNCT7_FSUB_S	          (4)
#define RV_OP_FP_FUNCT7_FMUL_S	          (8)
#define RV_OP_FP_FUNCT7_FDIV_S	          (12)
#define RV_OP_FP_FUNCT7_FSQRT_S	          (44)
#define RV_OP_FP_FUNCT7_FSGNJ_S	          (16)
#define RV_OP_FP_FUNCT7_FSGNJN_S          (16)
#define RV_OP_FP_FUNCT7_FSGNJX_S          (16)
#define RV_OP_FP_FUNCT7_FMIN_FMAX_S       (20)
#define RV_OP_FP_FUNCT7_FCVT_W_S_WU_S     (96)
#define RV_OP_FP_FUNCT7_FMV_X_W_FCLASS_S  (112)
#define RV_OP_FP_FUNCT7_FEQ_S_FLT_S_FLE_S (80)
#define RV_OP_FP_FUNCT7_FCVT_S_W_S_WU     (104)
#define RV_OP_FP_FUNCT7_FMV_W_X	          (120)

//
// Special RV32F rs2 values.
//
#define RV_OP_FP_RS2_FSQRT_S   (0)
#define RV_OP_FP_RS2_FCVT_W_S  (0)
#define RV_OP_FP_RS2_FCVT_WU_S (1)
#define RV_OP_FP_RS2_FMV_X_W   (0)
#define RV_OP_FP_RS2_FCLASS_S  (0)
#define RV_OP_FP_RS2_FCVT_S_W  (0)
#define RV_OP_FP_RS2_FCVT_S_WU (1)
#define RV_OP_FP_RS2_FMV_W_X   (0)
#define RV_OP_FP_RS2_FCVT_L_S  (2)
#define RV_OP_FP_RS2_FCVT_LU_S (3)
#define RV_OP_FP_RS2_FCVT_S_L  (2)
#define RV_OP_FP_RS2_FCVT_S_LU (3)


//
// Instruction funct12 values.
//

//
// Base opcode RV_OPCODE_SYSTEM funct12 values.
//
#define RV_SYSTEM_FUNCT12_ECALL  (0)
#define RV_SYSTEM_FUNCT12_EBREAK (1)

//
// Floating-point-specific values.
//

//
// RV32F 2-bit Floating-point format field fmt values.
//
#define RV_FP_FMT_S (0) /* 32-bit single-precision. */
#define RV_FP_FMT_D (1) /* 64-bit double-precision. */
#define RV_FP_FMT_H (2) /* 16-bit half-precision. */
#define RV_FP_FMT_Q (3) /* 128-bit quad-precision. */

//
// RV32F fcsr bitfield information.
//
#define RV_FCSR_FFLAGS_SHIFT    (0ul)
#define RV_FCSR_FFLAGS_MASK     ((1ul << (4 - 0 + 1)) - 1)
#define RV_FCSR_NX_SHIFT        (0ul) /* Inexact. */
#define RV_FCSR_NX_MASK         (1ul)
#define RV_FCSR_NX_FLAG         (RV_FCSR_NX_MASK << RV_FCSR_NX_SHIFT)
#define RV_FCSR_UF_SHIFT        (1ul) /* Underflow. */
#define RV_FCSR_UF_MASK         (1ul)
#define RV_FCSR_UF_FLAG         (RV_FCSR_UF_MASK << RV_FCSR_UF_SHIFT)
#define RV_FCSR_OF_SHIFT        (2ul) /* Overflow. */
#define RV_FCSR_OF_MASK         (1ul)
#define RV_FCSR_OF_FLAG         (RV_FCSR_OF_MASK << RV_FCSR_OF_SHIFT)
#define RV_FCSR_DZ_SHIFT        (3ul) /* Divide by Zero. */
#define RV_FCSR_DZ_MASK         (1ul)
#define RV_FCSR_DZ_FLAG         (RV_FCSR_DZ_MASK << RV_FCSR_DZ_SHIFT)
#define RV_FCSR_NV_SHIFT        (4ul) /* Invalid operation / Divide by Zero. */
#define RV_FCSR_NV_MASK         (1ul)
#define RV_FCSR_NV_FLAG         (RV_FCSR_NV_MASK << RV_FCSR_NV_SHIFT)
#define RV_FCSR_FRM_SHIFT       (5ul) /* Rounding Mode. */
#define RV_FCSR_FRM_MASK        ((1ul << (7 - 5 + 1)) - 1)
#define RV_FCSR_RESERVED0_SHIFT (8ul) /* Reserved. */
#define RV_FCSR_RESERVED0_MASK  ((1ul << (31 - 8 + 1)) - 1)

//
// RV32F rounding modes.
//
#define RV_ROUNDING_MODE_RNE        (0) /* Round to Nearest, ties to Even						 */
#define RV_ROUNDING_MODE_RTZ        (1) /* Round towards Zero									 */
#define RV_ROUNDING_MODE_RDN        (2) /* Round Down( towards -inf )							 */
#define RV_ROUNDING_MODE_RUP        (3) /* Round Up( towards +inf )								 */
#define RV_ROUNDING_MODE_RMM        (4) /* Round to Nearest, ties to Max Magnitude				 */
#define RV_ROUNDING_MODE_RESERVED0  (5) /* Reserved for future use.								 */
#define RV_ROUNDING_MODE_RESERVED1  (6) /* Reserved for future use.								 */
#define RV_ROUNDING_MODE_DYN        (7) /* Only valid if specified in an instruction's rm field. */

//
// RV32F classification values.
//
#define RV_FCLASS_RD_NEGATIVE_INF       (1ul << 0)
#define RV_FCLASS_RD_NEGATIVE_NORMAL    (1ul << 1)
#define RV_FCLASS_RD_NEGATIVE_SUBNORMAL (1ul << 2)
#define RV_FCLASS_RD_NEGATIVE_ZERO      (1ul << 3)
#define RV_FCLASS_RD_POSITIVE_ZERO      (1ul << 4)
#define RV_FCLASS_RD_POSITIVE_SUBNORMAL (1ul << 5)
#define RV_FCLASS_RD_POSITIVE_NORMAL    (1ul << 6)
#define RV_FCLASS_RD_POSITIVE_INF       (1ul << 7)
#define RV_FCLASS_RD_SIGNALING_NAN      (1ul << 8)
#define RV_FCLASS_RD_QUIET_NAN          (1ul << 9)

//
// RV interrupt 0 base exception indices.
//
#define RV_EXCEPTION_INSTRUCTION_ADDRESS_MISALIGNED 0
#define RV_EXCEPTION_INSTRUCTION_ACCESS_FAULT       1
#define RV_EXCEPTION_ILLEGAL_INSTRUCTION            2
#define RV_EXCEPTION_BREAKPOINT                     3
#define RV_EXCEPTION_LOAD_ADDRESS_MISALIGNED        4
#define RV_EXCEPTION_LOAD_ACCESS_FAULT              5
#define RV_EXCEPTION_STORE_AMO_ADDRESS_MISALIGNED   6
#define RV_EXCEPTION_STORE_AMO_ACCESS_FAULT         7
#define RV_EXCEPTION_ENV_CALL_FROM_U_MODE           8
#define RV_EXCEPTION_ENV_CALL_FROM_S_MODE           9
#define RV_EXCEPTION_RESERVED0                      10
#define RV_EXCEPTION_ENV_CALL_FROM_M_MODE           11
#define RV_EXCEPTION_INSTRUCTION_PAGE_FAULT         12
#define RV_EXCEPTION_LOAD_PAGE_FAULT                13
#define RV_EXCEPTION_RESERVED1                      14
#define RV_EXCEPTION_STORE_AMO_PAGE_FAULT           15

//
// CSR numbers/fields.
//

#define RV_CSR_ACCESS_READ  (1 << 0)
#define RV_CSR_ACCESS_WRITE (1 << 1)

//
// Floating-Point Control and Status Registers
// 0x001 Read/write fflags Floating-Point Accrued Exceptions.
// 0x002 Read/write frm    Floating-Point Dynamic Rounding Mode.
// 0x003 Read/write fcsr   Floating-Point Control and Status Register (frm + fflags).
//
#define RV_CSR_VALUE_FFLAGS      (0x001)
#define RV_CSR_PERMISSION_FFLAGS (RV_CSR_ACCESS_READ | RV_CSR_ACCESS_WRITE)

#define RV_CSR_VALUE_FRM         (0x002)
#define RV_CSR_PERMISSION_FRM    (RV_CSR_ACCESS_READ | RV_CSR_ACCESS_WRITE)

#define RV_CSR_VALUE_FCSR        (0x003)
#define RV_CSR_PERMISSION_FCSR   (RV_CSR_ACCESS_READ | RV_CSR_ACCESS_WRITE)

//
// Counters and Timers
// 0xC00 Read-only cycle    Cycle counter for RDCYCLE instruction.
// 0xC01 Read-only time     Timer for RDTIME instruction.
// 0xC02 Read-only instret  Instructions-retired counter for RDINSTRET instruction.
// 0xC80 Read-only cycleh   Upper 32 bits of cycle, RV32I only.
// 0xC81 Read-only timeh    Upper 32 bits of time, RV32I only.
// 0xC82 Read-only instreth Upper 32 bits of instret, RV32I only.
//
#define RV_CSR_VALUE_CYCLE         (0xC00)
#define RV_CSR_PERMISSION_CYCLE    (RV_CSR_ACCESS_READ)

#define RV_CSR_VALUE_TIME          (0xC01)
#define RV_CSR_PERMISSION_TIME     (RV_CSR_ACCESS_READ)

#define RV_CSR_VALUE_INSTRET       (0xC02)
#define RV_CSR_PERMISSION_INSTRET  (RV_CSR_ACCESS_READ)

#define RV_CSR_VALUE_CYCLEH        (0xC80)
#define RV_CSR_PERMISSION_CYCLEH   (RV_CSR_ACCESS_READ)

#define RV_CSR_VALUE_TIMEH         (0xC81)
#define RV_CSR_PERMISSION_TIMEH    (RV_CSR_ACCESS_READ)

#define RV_CSR_VALUE_INSTRETH      (0xC82)
#define RV_CSR_PERMISSION_INSTRETH (RV_CSR_ACCESS_READ)

//
// Instruction decoder helpers.
//

//
// Build an instruction classification value from opcode + funct3 + funct7 parts.
//
#define RV_INST_CLASSIFY_F3F7(Opcode, Funct3, Funct7) \
	 (((Opcode) & RV_INST_R_OPCODE_MASK)              \
	 | (((Funct3) & RV_INST_R_FUNCT3_MASK) << 7ul)    \
	 | (((Funct7) & RV_INST_R_FUNCT7_MASK) << 10ul))


//
// Exception/interrupt functions.
//


//
// Push exception interrupt.
//
static
VOID
RvpExceptionPush(
	_Inout_ RV_PROCESSOR* Vp,
	_In_    RV_UINT32     ExceptionIndex
	)
{
	//
	// Mark the given exception as pending.
	// Currently only a single exception may be queued per instruction.
	//
	Vp->ExceptionIndex   = ExceptionIndex;
	Vp->ExceptionPending = 1;
}

//
// Helper functions.
//

//
// Sign extends a 32-bit value with a variable sign bit index.
//
static
RV_UINT32
RvpSignExtend32(
	_Inout_ RV_PROCESSOR* Vp,
	_In_    RV_UINT32     Value,
	_In_    RV_UINT32     SignBitIndex
	)
{
	RV_UINT32 SxValueBitSize;
	RV_UINT32 SignBitIsSet;
	RV_UINT32 ExtensionMask;
	RV_UINT32 ValueMask;
	RV_UINT32 SxValue;

	//
	// Get sign bit boolean value, and calculate a mask of all bits below the sign bit.
	//
	SignBitIsSet   = ( ( Value & ( 1ul << SignBitIndex ) ) >> SignBitIndex );
	SxValueBitSize = ( sizeof( SxValueBitSize ) * CHAR_BIT );
	ValueMask      = ( ( ( RV_UINT32 )1ul << SignBitIndex ) - 1 );

	//
	// Calculate a mask with all word bits set to SignBitIsSet.
	//
	ExtensionMask  = ( ( ( RV_UINT32 )SignBitIsSet << ( SxValueBitSize - 1 ) ) - SignBitIsSet );
	ExtensionMask |= ( ( RV_UINT32 )SignBitIsSet << ( SxValueBitSize - 1 ) );

	//
	// Set all bits that aren't below the sign bit to the value of SignBitIsSet.
	//
	SxValue  = Value;
	SxValue |= ( ExtensionMask & ~ValueMask );

	return SxValue;
}


//
// MMU functions.
//


//
// Convert a guest physical-page-number to host virtual address.
// Used to convert the PPNs contained by page-tables to host-accessible addresses.
// Note: this function is currently only used for guest-to-host mapping.
//
_Success_( return )
static
RV_BOOLEAN
RvpMmuPtGuestPpnToHostVa(
	_Inout_ RV_PROCESSOR* Vp,
	_In_    RV_UINT64     GuestPpn,
	_Out_   VOID**        ppHostAddress
	)
{
	UNREFERENCED_PARAMETER( Vp );

	if( GuestPpn > ( UINT64_MAX / RV_SV48_PAGESIZE ) ) {
		return RV_FALSE;
	}

	*ppHostAddress = ( VOID* )( GuestPpn * RV_SV48_PAGESIZE );
	return RV_TRUE;
}

//
// Convert a guest physical address to host virtual address.
// Note: this function is currently only used for guest-to-host mapping.
//
_Success_( return )
static
RV_BOOLEAN
RvpMmuPtGuestPaToHostVa(
	_Inout_ RV_PROCESSOR* Vp,
	_In_    RV_UINT64     GuestPa,
	_Out_   VOID**        ppHostAddress
	)
{
	RV_UINT64 GuestPpn;
	VOID*     PageHostAddress;

	//
	// PPN of the PA.
	//
	GuestPpn = ( GuestPa / RV_SV48_PAGESIZE );

	//
	// Translate PPN of the given PA.
	//
	if( RvpMmuPtGuestPpnToHostVa( Vp, GuestPpn, &PageHostAddress ) == RV_FALSE ) {
		return RV_FALSE;
	}

	//
	// Add on the page offset to translated HVA of the PPN.
	//
	*ppHostAddress = ( ( RV_UCHAR* )PageHostAddress + ( GuestPa & ( RV_SV48_PAGESIZE - 1 ) ) );
	return RV_TRUE;
}

//
// Perform SV48 page table lookup.
// Note: this function is currently only used for guest-to-host mapping.
//
RV_MMU_TREE_WALK_RESULT
RvpMmuPtSv48TreeWalk(
	_Inout_ RV_PROCESSOR* Vp,
	_In_    RV_UINT64     TableRootPpn,
	_In_    RV_UINT64     LookupVa,
	_In_    RV_UINT32     AccessFlags
	)
{
	RV_MMU_TREE_WALK_RESULT Result;
	RV_UINT64               RootPpn;
	RV_UINT64               RootPa;
	RV_UINT64               PageSize;
	RV_UINT                 i;
	RV_UINT64               PtePa;
	VOID*                   PteHva;
	RV_UINT64               Pte;
	RV_UINT64               VpnShift;
	RV_UINT64               Vpn;
	RV_UINT64               PpnFull;
	RV_SIZE_T               j;
	RV_UINT64               Pa;
	RV_UINT64               PpnShift;
	RV_UINT64               PpnMask;

	//
	// Physical page number (PPN) of the root page table.
	//
	RootPpn = TableRootPpn;

	//
	// Page size.
	//
	PageSize = RV_SV48_PAGESIZE;

	//
	// Physical address of the root page table (a).
	// 1. Let a be satp.ppn * PAGESIZE, and let i = (LEVELS - 1). (For Sv48, PAGESIZE=4096 and
	// LEVELS=4.) The satp register must be active, i.e., the effective privilege mode must be
	// S-mode or U-mode.
	//
	RootPa = ( RootPpn * PageSize );
	i      = ( RV_SV48_LEVELS - 1 );

	//
	// Set up empty initial results
	//
	Result = ( RV_MMU_TREE_WALK_RESULT ){ 0 };

	//
	// Search all levels for a leaf node.
	//
	while( RV_TRUE ) {
		Result.Level = i;

		//
		// 2. Let pte be the value of the PTE at address a+va.vpn[i]*PTESIZE. (For Sv48, PTESIZE=8.)
		// If accessing pte violates a PMA or PMP check, raise an access-fault exception corresponding
		// to the original access type.
		//
		VpnShift = ( RV_SV48_VA_VPN0_SHIFT + ( i * RV_SV48_VA_VPN_BIT_COUNT ) );
		Vpn      = ( ( LookupVa & ( RV_SV48_VA_VPN0_MASK << VpnShift ) ) >> VpnShift );
		PtePa    = ( RootPa + ( Vpn * RV_SV48_PTESIZE ) );

		//
		// Translate the PTE's PA to a host virtual address.
		//
		if( RvpMmuPtGuestPaToHostVa( Vp, PtePa, &PteHva ) == RV_FALSE ) {
			Result.IsPresent = 0;
			break;
		}

		//
		// Read the value of the PTE.
		//
		Pte = *( volatile RV_UINT64* )PteHva;

		//
		// 3. If pte.v = 0, or if pte.r = 0 and pte.w = 1, or if any bits or encodings that are reserved for
		// future standard use are set within pte, stop and raise a page-fault exception corresponding
		// to the original access type.
		//
		if( ( Pte & RV_SV48_PTE_V_FLAG ) == 0
			|| ( ( Pte & ( RV_SV48_PTE_R_FLAG | RV_SV48_PTE_W_FLAG ) ) == RV_SV48_PTE_W_FLAG )
			|| ( ( Pte & ( RV_SV48_PTE_R_FLAG | RV_SV48_PTE_W_FLAG | RV_SV48_PTE_X_FLAG ) )
				 == ( RV_SV48_PTE_W_FLAG | RV_SV48_PTE_X_FLAG ) ) )
		{
			Result.IsPresent = 0;
			break;
		}

		//
		// Read PPN field of the PTE.
		// If this is a leaf node, then this is the PPN of the memory controlled by the PTE.
		// If this isn't a leaf node, then this is the PPN of another page table level.
		//
		PpnFull = RV_SV48_PTE_FULL_PPN( Pte );

		//
		// 4. Otherwise, the PTE is valid. If pte.r = 1 or pte.x = 1, go to step 5. Otherwise, this PTE is a
		// pointer to the next level of the page table. Let i = i-1. If i < 0, stop and raise a page-fault
		// exception corresponding to the original access type. Otherwise, let a = pte.ppn*PAGESIZE
		// and go to step 2.
		//
		if( ( Pte & ( RV_SV48_PTE_R_FLAG | RV_SV48_PTE_X_FLAG ) ) == 0 ) {
			//
			// This PTE is a pointer to the next level of the page table.
			//
			if( i == 0 ) {
				//
				// If this was the last level, and still wasn't a leaf node, this PTE is invalid,
				// stop and raise a page-fault exception corresponding to the original access type.
				//
				Result.IsPresent = 0;
				break;
			}

			//
			// Move down to the next level of the page table.
			//
			i -= 1;

			//
			// let a = pte.ppn*PAGESIZE and go to step 2.
			//
			RootPa = ( PpnFull * RV_SV48_PAGESIZE );
			continue;
		}

		//
		// 5. A leaf PTE has been found. Determine if the requested memory access is allowed by the
		// pte.r, pte.w, pte.x, and pte.u bits, given the current privilege mode and the value of the
		// SUM and MXR fields of the mstatus register. If not, stop and raise a page-fault exception
		// corresponding to the original access type.
		//
		if( ( Pte & AccessFlags ) != AccessFlags ) {
			Result.IsAccessFault = 1;
			break;
		}

		//
		// 6. If i > 0 and pte.ppn[i - 1 : 0] != 0, this is a misaligned superpage;
		// stop and raise a page-fault exception corresponding to the original access type.
		// 
		if( i > 0 ) {
			for( j = 0; j < i; j++ ) {
				PpnShift = ( RV_SV48_PTE_PPN0_SHIFT + ( 9 * j ) );
				PpnMask  = ( ( i == 3 ) ? RV_SV48_PTE_PPN3_MASK : RV_SV48_PTE_PPN0_MASK );
				if( ( Pte & ( PpnMask << PpnShift ) ) != 0 ) {
					Result.IsPresent = 0;
					break;
				}
			}
		}

		//
		// 7. If pte.a = 0, or if the original memory access is a store and pte.d = 0,
		// either raise a page-fault exception corresponding to the original access type, or:
		//  * If a store to pte would violate a PMA or PMP check, raise an access-fault exception
		//   corresponding to the original access type.
		// 
		//  * Perform the following steps atomically:
		//	  - Compare pte to the value of the PTE at address a + va.vpn[i] * PTESIZE.
		//	  - If the values match, set pte.a to 1 and, if the original memory access is a store, also
		//	  set pte.d to 1.
		//	  - If the comparison fails, return to step 2
		//
		if( ( Pte & RV_SV48_PTE_A_FLAG )
			|| ( ( AccessFlags & RV_SV48_PTE_W_FLAG ) && ( Pte & RV_SV48_PTE_D_FLAG ) == 0 ) )
		{
			/* TODO. */
		}

		//
		// pa.pgoff = va.pgoff.
		//
		Pa = ( ( ( LookupVa & RV_SV48_VA_PGOFF_MASK ) >> RV_SV48_VA_PGOFF_SHIFT ) << RV_SV48_PA_PGOFF_SHIFT );

		//
		// If i > 0, then this is a superpage translation and pa.ppn[i - 1 : 0] = va.vpn[i - 1 : 0].
		//
		if( i > 0 ) {
			for( j = 0; j < i; j++ ) {
				PpnShift = ( RV_SV48_PA_PPN0_SHIFT + ( 9 * j ) );
				PpnMask  = ( ( i == 3 ) ? RV_SV48_PA_PPN3_MASK : RV_SV48_PA_PPN0_MASK );
				VpnShift = ( RV_SV48_VA_VPN0_SHIFT + ( i * RV_SV48_VA_VPN_BIT_COUNT ) );
				Vpn      = ( ( LookupVa & ( RV_SV48_VA_VPN0_MASK << VpnShift ) ) >> VpnShift );
				Pa      |= ( Vpn << PpnShift );
			}
		}

		//
		// pa.ppn[LEVELS - 1 : i] = pte.ppn[LEVELS - 1 : i].
		//
		for( j = i; j < RV_SV48_LEVELS; j++ ) {
			PpnShift = ( RV_SV48_PTE_PPN0_SHIFT + ( 9 * j ) );
			PpnMask  = ( ( i == 3 ) ? RV_SV48_PTE_PPN3_MASK : RV_SV48_PTE_PPN0_MASK );
			Pa      |= ( ( Pte & ( PpnMask << PpnShift ) ) >> PpnShift ) << ( RV_SV48_PA_PPN0_SHIFT + ( 9 * j ) );
		}

		Result.IsPresent       = 1;
		Result.LeafPtePpn      = PpnFull;
		Result.PhysicalAddress = Pa;
		break;
	}

	return Result;
}

//
// Attempts to convert a guest address to host address using flat VA span MMU mode.
//
_Success_( return )
static
RV_BOOLEAN
RvpMmuResolveGuestAddressFlatSpan(
	_Inout_  RV_PROCESSOR* Vp,
	_In_     RV_UINTR      Address,
	_In_     RV_UINT32     AccessType,
	_Outptr_ VOID**        ppHostData,
	_Out_    RV_SIZE_T*    pRemainingSize
	)
{
	RV_UINTR SpanOffset;

	UNREFERENCED_PARAMETER( AccessType );

	SpanOffset = ( Address - Vp->MmuVaSpanGuestBase );
	if( ( Address < Vp->MmuVaSpanGuestBase )
		|| ( SpanOffset >= Vp->MmuVaSpanSize ) )
	{
		return RV_FALSE;
	}

	*ppHostData     = ( ( RV_UCHAR* )Vp->MmuVaSpanHostBase + SpanOffset );
	*pRemainingSize = ( Vp->MmuVaSpanSize - SpanOffset );
	return RV_TRUE;
}

_Success_( return )
static
RV_BOOLEAN
RvpMmuResolveGuestAddress(
	_Inout_  RV_PROCESSOR* Vp,
	_In_     RV_UINTR      Address,
	_In_     RV_UINT32     AccessType,
	_In_     RV_BOOLEAN    PushExceptions,
	_Outptr_ VOID**        ppHostAddress,
	_Out_    RV_SIZE_T*    pRemainingPageSize
	)
{
	RV_MMU_TREE_WALK_RESULT TreeWalk;
	RV_UINT32               PteAccessFlags;
	RV_UINT32               PageFaultException;
	RV_UINT32               AccessFaultException;
	RV_UINT64               PageEndAddress;

	//
	// Try resolving the given address using the flat span.
	//
	if( RvpMmuResolveGuestAddressFlatSpan( Vp, Address, AccessType, ppHostAddress, pRemainingPageSize ) ) {
		return RV_TRUE;
	}

	//
	// Currently all accesses should be done by user privilege level, no others are supported.
	//
	PteAccessFlags = RV_SV48_PTE_U_FLAG;

	//
	// Convert access type to PTE flags and exception types.
	//
	switch( AccessType ) {
	case RV_MMU_ACCESS_TYPE_LOAD:
		PteAccessFlags      |= RV_SV48_PTE_R_FLAG;
		PageFaultException   = RV_EXCEPTION_LOAD_PAGE_FAULT;
		AccessFaultException = RV_EXCEPTION_LOAD_ACCESS_FAULT;
		break;
	case RV_MMU_ACCESS_TYPE_STORE:
		PteAccessFlags      |= RV_SV48_PTE_W_FLAG;
		PageFaultException   = RV_EXCEPTION_STORE_AMO_PAGE_FAULT;
		AccessFaultException = RV_EXCEPTION_STORE_AMO_ACCESS_FAULT;
		break;
	case RV_MMU_ACCESS_TYPE_EXECUTE:
		PteAccessFlags      |= ( RV_SV48_PTE_R_FLAG | RV_SV48_PTE_X_FLAG );
		PageFaultException   = RV_EXCEPTION_INSTRUCTION_PAGE_FAULT;
		AccessFaultException = RV_EXCEPTION_INSTRUCTION_ACCESS_FAULT;
		break;
	default:
		RvpExceptionPush( Vp, RV_EXCEPTION_ILLEGAL_INSTRUCTION ); /* Internal error. */
		return RV_FALSE;
	}

	//
	// If guest to host translation through SV48 page-tables is not enabled,
	// and the flat span doesn't contain the given address, raise a page fault exception and fail.
	//
	if( Vp->MmuUseGuestToHostPt == RV_FALSE ) {
		if( PushExceptions != RV_FALSE ) {
			RvpExceptionPush( Vp, PageFaultException );
		}
		return RV_FALSE;
	}

	//
	// Translation using guest-to-host SV48 page-tables is enabled,
	// then perform a tree walk for the given address.
	//
	TreeWalk = RvpMmuPtSv48TreeWalk( Vp, Vp->MmuGuestToHostPtPpn, Address, PteAccessFlags );

	//
	// If any nodes encountered along the walk conflict with the desired access flags, raise an access fault exception.
	//
	if( TreeWalk.IsAccessFault != 0 ) {
		if( PushExceptions != RV_FALSE ) {
			RvpExceptionPush( Vp, AccessFaultException );
		}
		return RV_FALSE;
	}

	//
	// If no leaf PTE is present for this address, raise a page fault exception.
	//
	if( TreeWalk.IsPresent == 0 ) {
		if( PushExceptions != RV_FALSE ) {
			RvpExceptionPush( Vp, PageFaultException );
		}
		return RV_FALSE;
	}

	//
	// Convert the resolved guest "physical address" to a host virtual address.
	//
	if( RvpMmuPtGuestPaToHostVa( Vp, TreeWalk.PhysicalAddress, ppHostAddress ) == RV_FALSE ) {
		if( PushExceptions != RV_FALSE ) {
			RvpExceptionPush( Vp, PageFaultException );
		}
		return RV_FALSE;
	}
	
	//
	// Calculate how much data is left in the page following the resolved address.
	// TODO: Take into account actual page size of superpages.
	//
	PageEndAddress      = ( ( TreeWalk.PhysicalAddress + RV_SV48_PAGESIZE ) & ~( RV_SV48_PAGESIZE - 1 ) );
	*pRemainingPageSize = ( PageEndAddress - TreeWalk.PhysicalAddress );
	return RV_TRUE;
}

_Success_( return )
static
RV_BOOLEAN
RvpMmuGuestStoreCrossPageBoundary(
	_Inout_                       RV_PROCESSOR* Vp,
	_In_                          RV_UINTR      Address,
	_In_reads_bytes_( StoreSize ) const VOID*   StoreData,
	_In_                          RV_SIZE_T     StoreSize,
	_In_                          RV_SIZE_T     PageSize,
	_In_                          RV_BOOLEAN    PushExceptions
	)
{
	RV_MMU_ACCESS_CHUNK Chunks[ 2 ];
	RV_SIZE_T           ChunkCount;
	RV_SIZE_T           Offset;
	RV_SIZE_T           i;
	RV_UINTR            CurrentAddress;
	RV_SIZE_T           RemainingSize;
	VOID*               HostData;
	RV_SIZE_T           PageAvailableSize;
	RV_SIZE_T           WriteSize;
	RV_SIZE_T           j;

	//
	// Ensure that the store is a supported size.
	// Note: this is essential, ensures that the chunks array is big enough.
	//
	if( StoreSize > PageSize ) {
		RvpExceptionPush( Vp, RV_EXCEPTION_ILLEGAL_INSTRUCTION ); /* Internal error. */
		return RV_FALSE;
	}

	ChunkCount = 0;

	//
	// Attempt to resolve all cross-page chunks for the given span.
	// This pass is done before the actual write, to ensure that we don't
	// write any partial amount of data before raising an exception.
	//
	for( Offset = 0; Offset < StoreSize; Offset += PageAvailableSize ) {
		CurrentAddress = ( Address + Offset );
		RemainingSize  = ( StoreSize - Offset );

		//
		// Attempt to resolve the current guest address to host data address.
		//
		if( RvpMmuResolveGuestAddress( Vp,
									   CurrentAddress,
									   RV_MMU_ACCESS_TYPE_STORE,
									   PushExceptions,
									   &HostData,
									   &PageAvailableSize ) == RV_FALSE )
		{
			return RV_FALSE;
		}

		//
		// Create chunk from data available in this page.
		//
		Chunks[ ChunkCount++ ] = ( RV_MMU_ACCESS_CHUNK ){
			.Data = HostData,
			.Size = PageAvailableSize
		};
	}

	//
	// Split up writes that cross page boundaries.
	//
	for( Offset = 0, i = 0; i < ChunkCount; i++ ) {
		CurrentAddress = ( Address + Offset );
		RemainingSize  = ( StoreSize - Offset );

		//
		// Copy as many bytes as we can to this page.
		//
		HostData = Chunks[ i ].Data;
		PageAvailableSize = Chunks[ i ].Size;
		WriteSize = RV_MIN( RemainingSize, PageAvailableSize );
		for( j = 0; j < WriteSize; j++ ) {
			_Analysis_assume_( ( Offset + j ) < StoreSize );
			( ( RV_UINT8* )HostData )[ j ] = ( ( const RV_UINT8* )StoreData )[ Offset + j ];
		}

		//
		// Move forward by the amount of data that we have copied to this chunk.
		//
		Offset += WriteSize;
	}

	return RV_TRUE;
}

_Success_( return )
static
RV_BOOLEAN
RvpMmuGuestStore(
	_Inout_                       RV_PROCESSOR* Vp,
	_In_                          RV_UINTR      Address,
	_In_reads_bytes_( StoreSize ) const VOID*   StoreData,
	_In_                          RV_SIZE_T     StoreSize,
	_In_                          RV_SIZE_T     PageSize,
	_In_                          RV_BOOLEAN    PushExceptions
	)
{
	VOID*     HostData;
	RV_SIZE_T HostDataSize;
	RV_UINTR  StartPageAddress;
	RV_SIZE_T StartPageOffset;
	RV_SIZE_T StartPageRemaining;

	//
	// PageSize must be a power of 2.
	// StoreSize must be a power of 2 that doesn't exceed the maximum store size, nor the page size.
	//
	if( ( ( PageSize & ( PageSize - 1 ) ) != 0 )
		|| ( ( StoreSize & ( StoreSize - 1 ) ) != 0 )
		|| ( StoreSize > sizeof( RV_UINT64 ) ) 
		|| ( StoreSize > PageSize ) )
	{
		RvpExceptionPush( Vp, RV_EXCEPTION_ILLEGAL_INSTRUCTION ); /* Internal error. */
		return RV_FALSE;
	}

	//
	// If the store is misaligned and passes over the page boundary,
	// then a lookup & store must be performed for both pages.
	// A misaligned load or store instruction may be decomposed
	// into a set of component memory operations of any granularity.
	//
	StartPageAddress = ( Address & ~( PageSize - 1 ) );
	StartPageOffset = ( Address - StartPageAddress );
	StartPageRemaining = ( PageSize - StartPageOffset );
	if( StoreSize > StartPageRemaining ) {
		return RvpMmuGuestStoreCrossPageBoundary( Vp, Address, StoreData, StoreSize, PageSize, PushExceptions );
	}

	//
	// Attempt to resolve the given guest address to host data address.
	//
	if( RvpMmuResolveGuestAddress( Vp,
								   Address,
								   RV_MMU_ACCESS_TYPE_STORE,
								   PushExceptions,
								   &HostData,
								   &HostDataSize ) == RV_FALSE )
	{
		return RV_FALSE;
	}

	//
	// Perform aligned store of given size.
	//
	switch( StoreSize ) {
	case RV_MMU_ACCESS_SIZE_BYTE:
		RV_RELAXED_ATOMIC_STORE8( HostData, *( RV_UINT8* )StoreData );
		break;
	case RV_MMU_ACCESS_SIZE_HALF_WORD:
		RV_RELAXED_ATOMIC_STORE16( HostData, *( RV_UINT16* )StoreData );
		break;
	case RV_MMU_ACCESS_SIZE_WORD:
		RV_RELAXED_ATOMIC_STORE32( HostData, *( RV_UINT32* )StoreData );
		break;
	case RV_MMU_ACCESS_SIZE_DOUBLE_WORD:
		RV_RELAXED_ATOMIC_STORE64( HostData, *( RV_UINT64* )StoreData );
		break;
	default:
		RvpExceptionPush( Vp, RV_EXCEPTION_ILLEGAL_INSTRUCTION ); /* Internal error. */
		return RV_FALSE;
	}

	return RV_TRUE;
}

_Success_( return )
static
RV_BOOLEAN
RvpMmuGuestLoadCrossPageBoundary(
	_Inout_                            RV_PROCESSOR* Vp,
	_In_                               RV_UINTR      Address,
	_Out_writes_bytes_all_( LoadSize ) VOID*         Destination,
	_In_                               RV_SIZE_T     LoadSize,
	_In_                               RV_SIZE_T     PageSize,
	_In_                               RV_BOOLEAN    PushExceptions
	)
{
	RV_MMU_ACCESS_CHUNK Chunks[ 2 ];
	RV_SIZE_T           ChunkCount;
	RV_SIZE_T           Offset;
	RV_SIZE_T           i;
	RV_UINTR            CurrentAddress;
	RV_SIZE_T           RemainingSize;
	VOID*               HostData;
	RV_SIZE_T           PageAvailableSize;
	RV_SIZE_T           ReadSize;
	RV_SIZE_T           j;

	//
	// Ensure that the load is a supported size.
	// Note: this is essential, ensures that the chunks array is big enough.
	//
	if( LoadSize > PageSize ) {
		RvpExceptionPush( Vp, RV_EXCEPTION_ILLEGAL_INSTRUCTION ); /* Internal error. */
		return RV_FALSE;
	}

	ChunkCount = 0;

	//
	// Attempt to resolve all cross-page chunks for the given span.
	// This pass is done before the actual write, to ensure that we don't
	// write any partial amount of data before raising an exception.
	//
	for( Offset = 0; Offset < LoadSize; Offset += PageAvailableSize ) {
		CurrentAddress = ( Address + Offset );
		RemainingSize  = ( LoadSize - Offset );

		//
		// Attempt to resolve the current guest address to host data address.
		//
		if( RvpMmuResolveGuestAddress( Vp,
									   CurrentAddress,
									   RV_MMU_ACCESS_TYPE_LOAD,
									   PushExceptions,
									   &HostData,
									   &PageAvailableSize ) == RV_FALSE )
		{
			return RV_FALSE;
		}

		//
		// Create chunk from data available in this page.
		//
		Chunks[ ChunkCount++ ] = ( RV_MMU_ACCESS_CHUNK ){
			.Data = HostData,
			.Size = PageAvailableSize
		};
	}

	//
	// The found chunks must contain enough data to serve the entire load. Should always happen.
	//
	if( Offset < LoadSize ) {
		RvpExceptionPush( Vp, RV_EXCEPTION_ILLEGAL_INSTRUCTION ); /* Internal error. */
		return RV_FALSE;
	}

	//
	// Split up loads that cross page boundaries.
	//
	for( Offset = 0, i = 0; i < ChunkCount; i++ ) {
		CurrentAddress = ( Address + Offset );
		RemainingSize  = ( LoadSize - Offset );

		//
		// Copy as many bytes as we can from this page.
		//
		HostData = Chunks[ i ].Data;
		PageAvailableSize = Chunks[ i ].Size;
		ReadSize = RV_MIN( RemainingSize, PageAvailableSize );
		for( j = 0; j < ReadSize; j++ ) {
			_Analysis_assume_( ( Offset + j ) < LoadSize );
			( ( RV_UINT8* )Destination )[ Offset + j ] = ( ( const RV_UINT8* )HostData )[ j ];
		}

		//
		// Move forward by the amount of data that we have copied to this chunk.
		//
		Offset += ReadSize;
	}

	_Analysis_assume_( Offset >= LoadSize );

	return RV_TRUE;
}

_Success_( return )
static
RV_BOOLEAN
RvpMmuGuestLoad(
	_Inout_                            RV_PROCESSOR* Vp,
	_In_                               RV_UINTR      Address,
	_Out_writes_bytes_all_( LoadSize ) VOID*         Destination,
	_In_                               RV_SIZE_T     LoadSize,
	_In_                               RV_SIZE_T     PageSize,
	_In_                               RV_BOOLEAN    PushExceptions
	)
{
	VOID*     HostData;
	RV_SIZE_T HostDataSize;
	RV_UINTR  StartPageAddress;
	RV_SIZE_T StartPageOffset;
	RV_SIZE_T StartPageRemaining;

	//
	// PageSize must be a power of 2.
	// LoadSize must be a power of 2 that doesn't exceed the maximum load size, nor the page size.
	//
	if( ( ( PageSize & ( PageSize - 1 ) ) != 0 )
		|| ( ( LoadSize & ( LoadSize - 1 ) ) != 0 )
		|| ( LoadSize > sizeof( RV_UINT64 ) )
		|| ( LoadSize > PageSize ) )
	{
		RvpExceptionPush( Vp, RV_EXCEPTION_ILLEGAL_INSTRUCTION ); /* Internal error. */
		return RV_FALSE;
	}

	//
	// If the load is misaligned and passes over the page boundary,
	// then a lookup & load must be performed for both pages.
	// A misaligned load or store instruction may be decomposed
	// into a set of component memory operations of any granularity.
	//
	StartPageAddress = ( Address & ~( PageSize - 1 ) );
	StartPageOffset = ( Address - StartPageAddress );
	StartPageRemaining = ( PageSize - StartPageOffset );
	if( LoadSize > StartPageRemaining ) {
		return RvpMmuGuestLoadCrossPageBoundary( Vp, Address, Destination, LoadSize, PageSize, PushExceptions );
	}

	//
	// Attempt to resolve the given guest address to host data address.
	//
	if( RvpMmuResolveGuestAddress( Vp,
								   Address,
								   RV_MMU_ACCESS_TYPE_LOAD,
								   PushExceptions,
								   &HostData,
								   &HostDataSize ) == RV_FALSE )
	{
		return RV_FALSE;
	}

	//
	// Perform aligned load of given size.
	//
	switch( LoadSize ) {
	case RV_MMU_ACCESS_SIZE_BYTE:
		*( RV_UINT8* )Destination = RV_RELAXED_ATOMIC_LOAD8( HostData );
		break;
	case RV_MMU_ACCESS_SIZE_HALF_WORD:
		*( RV_UINT16* )Destination = RV_RELAXED_ATOMIC_LOAD16( HostData );
		break;
	case RV_MMU_ACCESS_SIZE_WORD:
		*( RV_UINT32* )Destination = RV_RELAXED_ATOMIC_LOAD32( HostData );
		break;
	case RV_MMU_ACCESS_SIZE_DOUBLE_WORD:
		*( RV_UINT64* )Destination = RV_RELAXED_ATOMIC_LOAD32( HostData );
		break;
	default:
		RvpExceptionPush( Vp, RV_EXCEPTION_ILLEGAL_INSTRUCTION ); /* Internal error. */
		return RV_FALSE;
	}

	return RV_TRUE;
}

//
// Attempt to fetch an instruction word from the given guest address.
// Note: Does not enforce address alignment constraints.
//
_Success_( return )
static
RV_BOOLEAN
RvpFetchInstructionWord(
	_Inout_ RV_PROCESSOR* Vp,
	_In_    RV_UINTR      Address,
	_In_    RV_BOOLEAN    PushExceptions,
	_Out_   RV_UINT32*    pWord
	)
{
	RV_UINT32 RawWord;

	//
	// Attempt to fetch instruction data.
	//
	if( RvpMmuGuestLoad( Vp, Address, &RawWord, sizeof( RawWord ), RV_MMU_MIN_PAGESIZE, PushExceptions ) == RV_FALSE ) {
		return RV_FALSE;
	}

	//
	// Convert from little-endian to host endianness and return fetched word.
	//
	*pWord = RV_LITTLE_ENDIAN_32( RawWord );
	return RV_TRUE;
}


//
// RV32I opcodes.
//


static
VOID
RvpInstructionExecuteOpcodeLoad(
	_Inout_ RV_PROCESSOR* Vp,
	_In_    RV_UINT32     Instruction
	)
{
	RV_UINT32    Funct3;
	RV_UINT32    Rd;
	RV_UINT32    Rs1;
	RV_UINT32    UOffset32;
	RV_UINTR     Address;
	RV_MEM_VALUE Value;

	//
	// Decode I-type instruction fields.
	//
	Funct3    = ( ( Instruction >> RV_INST_I_FUNCT3_SHIFT ) & RV_INST_I_FUNCT3_MASK );
	Rd        = ( ( Instruction >> RV_INST_I_RD_SHIFT ) & RV_INST_I_RD_MASK );
	Rs1       = ( ( Instruction >> RV_INST_I_RS1_SHIFT ) & RV_INST_I_RS1_MASK );
	UOffset32 = ( ( Instruction >> RV_INST_I_IMM_11_0_SHIFT ) & RV_INST_I_IMM_11_0_MASK );
	UOffset32 = RvpSignExtend32( Vp, UOffset32, 11 );

	//
	// Validate register indices.
	//
	if( Rd >= RV_COUNTOF( Vp->Xr ) || Rs1 >= RV_COUNTOF( Vp->Xr ) ) {
		RvpExceptionPush( Vp, RV_EXCEPTION_ILLEGAL_INSTRUCTION );
		return;
	}

	//
	// The effective address is obtained by adding register rs1 to the sign-extended 12-bit offset.
	// Loads copy a value from memory to register rd. Stores copy the value in register rs2 to memory.
	//
	Address = ( Vp->Xr[ Rs1 ] + ( RV_INTR )( RV_INT32 )UOffset32 );

	//
	// Lookup the host data address for the given guest effective address, and perform the load.
	// TODO: Update this to use generic ReadMemory function to handle cross
	// page-boundary accesses once MMU is fleshed out!
	// 
	// RV_OPCODE_LOAD funct3 values (I-type).
	// imm[11:0] rs1 000 rd 0000011 LB
	// imm[11:0] rs1 001 rd 0000011 LH
	// imm[11:0] rs1 010 rd 0000011 LW
	// imm[11:0] rs1 100 rd 0000011 LBU
	// imm[11:0] rs1 101 rd 0000011 LHU
	// imm[11:0] rs1 011 rd 0000011 LD  (RV64I)
	// imm[11:0] rs1 110 rd 0000011 LWU (RV64I)
	//
	switch( Funct3 ) {
#if defined(RV_OPT_RV64I)
	case RV_LOAD_FUNCT3_LD:
		//
		// The LD instruction loads a 64-bit value from memory into register rd for RV64I.
		//
		if( RvpMmuGuestLoad( Vp, Address, &Value.AsU64, sizeof( Value.AsU64 ), RV_MMU_MIN_PAGESIZE, RV_TRUE ) == RV_FALSE ) {
			return;
		}
		Vp->Xr[ Rd ] = Value.AsU64;
		break;
	case RV_LOAD_FUNCT3_LWU:
		//
		// LWU zero-extends the 32-bit value from memory for RV64I.
		//
		if( RvpMmuGuestLoad( Vp, Address, &Value.AsU32, sizeof( Value.AsU32 ), RV_MMU_MIN_PAGESIZE, RV_TRUE ) == RV_FALSE ) {
			return;
		}
		Vp->Xr[ Rd ] = ( RV_UINTR )Value.AsU32;
		break;
#endif
	case RV_LOAD_FUNCT3_LW:
		//
		// LW loads a 32-bit value from memory, then sign-extends to XLEN-bits before storing it in rd.
		//
		if( RvpMmuGuestLoad( Vp, Address, &Value.AsU32, sizeof( Value.AsU32 ), RV_MMU_MIN_PAGESIZE, RV_TRUE ) == RV_FALSE ) {
			return;
		}
		Vp->Xr[ Rd ] = ( RV_INTR )( RV_INT32 )Value.AsU32;
		break;
	case RV_LOAD_FUNCT3_LH:
		//
		// LH loads a 16-bit value from memory, then sign-extends to XLEN-bits before storing in rd.
		//
		if( RvpMmuGuestLoad( Vp, Address, &Value.AsU16, sizeof( Value.AsU16 ), RV_MMU_MIN_PAGESIZE, RV_TRUE ) == RV_FALSE ) {
			return;
		}
		Vp->Xr[ Rd ] = ( RV_INTR )( RV_INT16 )Value.AsU16;
		break;
	case RV_LOAD_FUNCT3_LHU:
		//
		// LHU loads a 16-bit value from memory but then zero extends to XLEN-bits before storing in rd.
		//
		if( RvpMmuGuestLoad( Vp, Address, &Value.AsU16, sizeof( Value.AsU16 ), RV_MMU_MIN_PAGESIZE, RV_TRUE ) == RV_FALSE ) {
			return;
		}
		Vp->Xr[ Rd ] = Value.AsU16;
		break;
	case RV_LOAD_FUNCT3_LB:
		//
		// LB loads an 8-bit value from memory, then sign-extends to XLEN-bits before storing in rd.
		//
		if( RvpMmuGuestLoad( Vp, Address, &Value.AsU8, sizeof( Value.AsU8 ), RV_MMU_MIN_PAGESIZE, RV_TRUE ) == RV_FALSE ) {
			return;
		}
		Vp->Xr[ Rd ] = ( RV_INTR )( RV_INT8 )Value.AsU8;
		break;
	case RV_LOAD_FUNCT3_LBU:
		//
		// LBU loads an 8-bit value from memory but then zero extends to XLEN-bits before storing in rd.
		//
		if( RvpMmuGuestLoad( Vp, Address, &Value.AsU8, sizeof( Value.AsU8 ), RV_MMU_MIN_PAGESIZE, RV_TRUE ) == RV_FALSE ) {
			return;
		}
		Vp->Xr[ Rd ] = Value.AsU8;
		break;
	default:
		RvpExceptionPush( Vp, RV_EXCEPTION_ILLEGAL_INSTRUCTION );
		return;
	}

	Vp->Pc += 4;
}

static
VOID
RvpInstructionExecuteOpcodeStore(
	_Inout_ RV_PROCESSOR* Vp,
	_In_    RV_UINT32     Instruction
	)
{
	RV_UINT32    Funct3;
	RV_UINT32    Offset;
	RV_UINT32    Rs1;
	RV_UINT32    Rs2;
	RV_UINTR     Address;
	RV_MEM_VALUE Value;

	//
	// Decode S-type instruction fields.
	//
	Funct3  = ( ( Instruction >> RV_INST_S_FUNCT3_SHIFT ) & RV_INST_S_FUNCT3_MASK );
	Rs1     = ( ( Instruction >> RV_INST_S_RS1_SHIFT ) & RV_INST_S_RS1_MASK );
	Rs2     = ( ( Instruction >> RV_INST_S_RS2_SHIFT ) & RV_INST_S_RS2_MASK );
	Offset  = ( ( Instruction >> RV_INST_S_IMM_4_0_SHIFT ) & RV_INST_S_IMM_4_0_MASK );
	Offset |= ( ( ( Instruction >> RV_INST_S_IMM_11_5_SHIFT ) & RV_INST_S_IMM_11_5_MASK ) << 5ul );
	Offset  = RvpSignExtend32( Vp, Offset, 11 );

	//
	// Validate register indices.
	//
	if( Rs1 >= RV_COUNTOF( Vp->Xr ) || Rs2 >= RV_COUNTOF( Vp->Xr ) ) {
		RvpExceptionPush( Vp, RV_EXCEPTION_ILLEGAL_INSTRUCTION );
		return;
	}

	//
	// The effective address is obtained by adding register rs1 to the sign-extended 12-bit offset.
	// Loads copy a value from memory to register rd. Stores copy the value in register rs2 to memory.
	//
	Address = ( Vp->Xr[ Rs1 ] + ( RV_INTR )( RV_INT32 )Offset );

	//
	// Lookup the host data address for the given guest effective address, and perform the store.
	// TODO: Update this to use generic WriteMemory function to handle cross
	// page-boundary accesses once MMU is fleshed out!
	// 
	// Base opcode RV_OPCODE_STORE funct3 values (S-type).
	// imm[11:5] rs2 rs1 000 imm[4:0] 0100011 SB
	// imm[11:5] rs2 rs1 001 imm[4:0] 0100011 SH
	// imm[11:5] rs2 rs1 010 imm[4:0] 0100011 SW
	// imm[11:5] rs2 rs1 011 imm[4:0] 0100011 SD (RV64I)
	//
	switch( Funct3 ) {
	case RV_STORE_FUNCT3_SB:
		Value = ( RV_MEM_VALUE ){ .AsU8 = ( RV_UINT8 )Vp->Xr[ Rs2 ] };
		if( RvpMmuGuestStore( Vp, Address, &Value.AsU8, sizeof( Value.AsU8 ), RV_MMU_MIN_PAGESIZE, RV_TRUE ) == RV_FALSE ) {
			return;
		}
		break;
	case RV_STORE_FUNCT3_SH:
		Value = ( RV_MEM_VALUE ){ .AsU16 = ( RV_UINT16 )Vp->Xr[ Rs2 ] };
		if( RvpMmuGuestStore( Vp, Address, &Value.AsU16, sizeof( Value.AsU16 ), RV_MMU_MIN_PAGESIZE, RV_TRUE ) == RV_FALSE ) {
			return;
		}
		break;
	case RV_STORE_FUNCT3_SW:
		Value = ( RV_MEM_VALUE ){ .AsU32 = ( RV_UINT32 )Vp->Xr[ Rs2 ] };
		if( RvpMmuGuestStore( Vp, Address, &Value.AsU32, sizeof( Value.AsU32 ), RV_MMU_MIN_PAGESIZE, RV_TRUE ) == RV_FALSE ) {
			return;
		}
		break;
#if defined(RV_OPT_RV64I)
	case RV_STORE_FUNCT3_SD:
		Value = ( RV_MEM_VALUE ){ .AsU64 = ( RV_UINT64 )Vp->Xr[ Rs2 ] };
		if( RvpMmuGuestStore( Vp, Address, &Value.AsU64, sizeof( Value.AsU64 ), RV_MMU_MIN_PAGESIZE, RV_TRUE ) == RV_FALSE ) {
			return;
		}
		break;
#endif
	default:
		RvpExceptionPush( Vp, RV_EXCEPTION_ILLEGAL_INSTRUCTION );
		return;
	}

	Vp->Pc += 4;
}

static
VOID
RvpInstructionExecuteOpcodeMiscMem(
	_Inout_ RV_PROCESSOR* Vp,
	_In_    RV_UINT32     Instruction
	)
{
	RV_UINT32 Funct3;

	Funct3 = ( ( Instruction >> RV_INST_I_FUNCT3_SHIFT ) & RV_INST_I_FUNCT3_MASK );

	switch( Funct3 ) {
	case RV_MISC_MEM_FUNCT3_FENCE:
		//
		// Unimplemented, TODO!
		//
		break;
	default:
		RvpExceptionPush( Vp, RV_EXCEPTION_ILLEGAL_INSTRUCTION );
		return;
	}

	Vp->Pc += 4;
}

static
VOID
RvpInstructionExecuteOpcodeLui(
	_Inout_ RV_PROCESSOR* Vp,
	_In_    RV_UINT32     Instruction
	)
{
	RV_UINT32 Rd;
	RV_UINT32 Imm_31_12;

	//
	// Decode U-type fields.
	//
	Rd        = ( ( Instruction >> RV_INST_U_RD_SHIFT ) & RV_INST_U_RD_MASK );
	Imm_31_12 = ( Instruction & ( RV_INST_U_IMM_31_12_MASK << RV_INST_U_IMM_31_12_SHIFT ) );

	//
	// Validate register indices.
	//
	if( Rd >= RV_COUNTOF( Vp->Xr ) ) {
		RvpExceptionPush( Vp, RV_EXCEPTION_ILLEGAL_INSTRUCTION );
		return;
	}

	//
	// LUI (load upper immediate) is used to build 32-bit constants and uses the U-type format. LUI
	// places the U-immediate value in the top 20 bits of the destination register rd, filling in the lowest
	// 12 bits with zeros.
	//
	Vp->Xr[ Rd ] = RvpSignExtend32( Vp, Imm_31_12, 31 );
	Vp->Pc += 4;
}

static
VOID
RvpInstructionExecuteOpcodeAuipc(
	_Inout_ RV_PROCESSOR* Vp,
	_In_    RV_UINT32     Instruction
	)
{
	RV_UINT32 Rd;
	RV_UINT32 Imm_31_12;

	//
	// Decode U-type fields.
	//
	Rd        = ( ( Instruction >> RV_INST_U_RD_SHIFT ) & RV_INST_U_RD_MASK );
	Imm_31_12 = ( Instruction & ( RV_INST_U_IMM_31_12_MASK << RV_INST_U_IMM_31_12_SHIFT ) );

	//
	// Validate register indices.
	//
	if( Rd >= RV_COUNTOF( Vp->Xr ) ) {
		RvpExceptionPush( Vp, RV_EXCEPTION_ILLEGAL_INSTRUCTION );
		return;
	}

	//
	// AUIPC (add upper immediate to pc) is used to build pc-relative addresses and uses the U-type
	// format. AUIPC forms a 32-bit offset from the 20-bit U-immediate, filling in the lowest 12 bits with
	// zeros, adds this offset to the address of the AUIPC instruction, then places the result in register rd.
	//
	Vp->Xr[ Rd ] = ( Vp->Pc + ( RV_INTR )( RV_INT32 )RvpSignExtend32( Vp, Imm_31_12, 31 ) );
	Vp->Pc += 4;
}

static
VOID
RvpInstructionExecuteOpcodeOpImmIType(
	_Inout_ RV_PROCESSOR* Vp,
	_In_    RV_UINT32     Instruction,
	_In_    RV_UINT32     Funct3
	)
{
	RV_UINT32 UImm32;
	RV_UINT32 Rd;
	RV_UINT32 Rs1;

	//
	// Decode I-type fields.
	//
	Rd = ( ( Instruction >> RV_INST_I_RD_SHIFT ) & RV_INST_I_RD_MASK );
	Rs1 = ( ( Instruction >> RV_INST_I_RS1_SHIFT ) & RV_INST_I_RS1_MASK );
	UImm32 = RvpSignExtend32(
		Vp,
		( ( ( Instruction ) >> RV_INST_I_IMM_11_0_SHIFT ) & RV_INST_I_IMM_11_0_MASK ),
		11
	);

	//
	// Validate register operands.
	//
	if( Rd >= RV_COUNTOF( Vp->Xr ) || Rs1 >= RV_COUNTOF( Vp->Xr ) ) {
		RvpExceptionPush( Vp, RV_EXCEPTION_ILLEGAL_INSTRUCTION );
		return;
	}

	//
	// Base opcode RV_OPCODE_OP_IMM funct3 values (I-type).
	// imm[11:0] rs1 000 rd 0010011 ADDI
	// imm[11:0] rs1 010 rd 0010011 SLTI
	// imm[11:0] rs1 011 rd 0010011 SLTIU
	// imm[11:0] rs1 100 rd 0010011 XORI
	// imm[11:0] rs1 110 rd 0010011 ORI
	// imm[11:0] rs1 111 rd 0010011 ANDI
	//
	switch( Funct3 ) {
	case RV_OP_IMM_FUNCT3_ADDI:
		Vp->Xr[ Rd ] = ( Vp->Xr[ Rs1 ] + ( RV_INTR )( RV_INT32 )UImm32 );
		break;
	case RV_OP_IMM_FUNCT3_SLTI:
		Vp->Xr[ Rd ] = ( ( ( RV_INTR )Vp->Xr[ Rs1 ] < ( RV_INTR )( RV_INT32 )UImm32 ) ? RV_TRUE : RV_FALSE );
		break;
	case RV_OP_IMM_FUNCT3_SLTIU:
		Vp->Xr[ Rd ] = ( ( Vp->Xr[ Rs1 ] < UImm32 ) ? RV_TRUE : RV_FALSE );
		break;
	case RV_OP_IMM_FUNCT3_XORI:
		Vp->Xr[ Rd ] = ( Vp->Xr[ Rs1 ] ^ UImm32 );
		break;
	case RV_OP_IMM_FUNCT3_ORI:
		Vp->Xr[ Rd ] = ( Vp->Xr[ Rs1 ] | UImm32 );
		break;
	case RV_OP_IMM_FUNCT3_ANDI:
		Vp->Xr[ Rd ] = ( Vp->Xr[ Rs1 ] & UImm32 );
		break;
	default:
		RvpExceptionPush( Vp, RV_EXCEPTION_ILLEGAL_INSTRUCTION );
		return;
	}

	Vp->Pc += 4;
}

static
VOID
RvpInstructionExecuteOpcodeOpImmITypeShamt(
	_Inout_ RV_PROCESSOR* Vp,
	_In_    RV_UINT32     Instruction,
	_In_    RV_UINT32     Funct3
	)
{
	RV_UINT32 Opcode;
	RV_UINT32 Rd;
	RV_UINT32 Rs1;
	RV_UINT32 ImmRaw;
	RV_UINT32 Shamt;
	RV_UINT32 ShiftType;

	//
	// Decode I-type fields.
	//
	Opcode = ( ( Instruction >> RV_INST_I_OPCODE_SHIFT ) & RV_INST_I_OPCODE_MASK );
	Rd     = ( ( Instruction >> RV_INST_I_RD_SHIFT ) & RV_INST_I_RD_MASK );
	Rs1    = ( ( Instruction >> RV_INST_I_RS1_SHIFT ) & RV_INST_I_RS1_MASK );
	ImmRaw = ( ( Instruction >> RV_INST_I_IMM_11_0_SHIFT ) & RV_INST_I_IMM_11_0_MASK );

	//
	// Validate register operands.
	//
	if( Rd >= RV_COUNTOF( Vp->Xr ) || Rs1 >= RV_COUNTOF( Vp->Xr ) ) {
		RvpExceptionPush( Vp, RV_EXCEPTION_ILLEGAL_INSTRUCTION );
		return;
	}

	//
	// In RV64I, the shift amount is encoded in the lower 6 bits of the I-immediate,
	// and the shift type is encoded in the upper 6 bits of the I-immediate field.
	// In RV32I, the shift amount is encoded in the lower 5 bits of the I-immediate,
	// and the shift type is encoded in the upper 7 bits of the I-immediate.
	//
#if defined(RV_OPT_RV64I)
	Shamt     = ( ImmRaw & ( ( 1ul << 6 ) - 1 ) );
	ShiftType = ( ( ImmRaw & ~( ( 1ul << 6 ) - 1 ) ) >> 5ul );
#elif defined(RV_OPT_RV32I)
	Shamt     = ( ImmRaw & ( ( 1ul << 5 ) - 1 ) );
	ShiftType = ( ImmRaw & ~( ( 1ul << 5 ) - 1 ) ) >> 5ul;
#else
	#error "Unsupported mode."
#endif

	//
	// Validate register indices.
	//
	if( Rd >= RV_COUNTOF( Vp->Xr ) || Rs1 >= RV_COUNTOF( Vp->Xr ) ) {
		RvpExceptionPush( Vp, RV_EXCEPTION_ILLEGAL_INSTRUCTION );
		return;
	}

	//
	// In RV64I, shifts by a constant are encoded as a specialization of the I-type format using the same instruction
	// opcode as RV32I. The operand to be shifted is in rs1, and the shift amount is encoded in the lower
	// 6 bits of the I-immediate field for RV64I. The right shift type is encoded in bit 30.
	//

	//
	// Base opcode RV_OPCODE_OP_IMM funct3 values (R-type).
	// 0000000 shamt rs1 001 rd 0010011 SLLI
	// 0000000 shamt rs1 101 rd 0010011 SRLI
	// 0100000 shamt rs1 101 rd 0010011 SRAI
	//
	switch( Funct3 ) {
	case RV_OP_IMM_FUNCT3_SLLI:
		if( ShiftType != RV_OP_IMM_FUNCT7_SLLI ) {
			RvpExceptionPush( Vp, RV_EXCEPTION_ILLEGAL_INSTRUCTION );
			return;
		}
		Vp->Xr[ Rd ] = ( Vp->Xr[ Rs1 ] << Shamt );
		break;
	case RV_OP_IMM_FUNCT3_SRLI_SRAI:
		if( ShiftType == 0 ) {
			Vp->Xr[ Rd ] = ( Vp->Xr[ Rs1 ] >> Shamt );
		} else if( ShiftType == 0x20 ) {
			Vp->Xr[ Rd ] = ( ( RV_INTR )Vp->Xr[ Rs1 ] >> Shamt );
		} else {
			RvpExceptionPush( Vp, RV_EXCEPTION_ILLEGAL_INSTRUCTION );
			return;
		}
		break;
	default:
		RvpExceptionPush( Vp, RV_EXCEPTION_ILLEGAL_INSTRUCTION );
		return;
	}

	Vp->Pc += 4;
}

static
VOID
RvpInstructionExecuteOpcodeOpImm(
	_Inout_ RV_PROCESSOR* Vp,
	_In_    RV_UINT32     Instruction
	)
{
	RV_UINT32 Funct3;

	//
	// Decode funct3 value, same field bits for both I/R-types handled by this function.
	//
	Funct3 = ( ( Instruction >> RV_INST_I_FUNCT3_SHIFT ) & RV_INST_I_FUNCT3_MASK );

	//
	// Dispatch opcode RV_OPCODE_OP_IMM funct3 values to instruction type specific handlers.
	//
	switch( Funct3 ) {
	/* I-Type instructions. */
	case RV_OP_IMM_FUNCT3_ADDI:
	case RV_OP_IMM_FUNCT3_SLTI:
	case RV_OP_IMM_FUNCT3_SLTIU:
	case RV_OP_IMM_FUNCT3_XORI:
	case RV_OP_IMM_FUNCT3_ORI:
	case RV_OP_IMM_FUNCT3_ANDI:
		RvpInstructionExecuteOpcodeOpImmIType( Vp, Instruction, Funct3 );
		break;
	/* I-Type specialized shamt instructions. */
	case RV_OP_IMM_FUNCT3_SLLI:
	case RV_OP_IMM_FUNCT3_SRLI_SRAI:
		RvpInstructionExecuteOpcodeOpImmITypeShamt( Vp, Instruction, Funct3 );
		break;
	default:
		RvpExceptionPush( Vp, RV_EXCEPTION_ILLEGAL_INSTRUCTION );
		break;
	}
}

//
// RV32M instructions.
//
#if defined(RV_OPT_RV32M)

//
// U64xU64 -> upper half of 128bit result.
//
static
RV_UINT64
RvpUMulh64(	
	_Inout_ RV_PROCESSOR* Vp,
	_In_    RV_UINT64     Operand1,
	_In_    RV_UINT64     Operand2
	)
{
	UNREFERENCED_PARAMETER( Vp );

#ifdef RV_OPT_BUILD_MSVC
	return __umulh( Operand1, Operand2 );
#elif defined(RV_OPT_BUILD_INT128_TYPES)
	return ( ( ( uint128_t )Operand1 * ( uint128_t )Operand2 ) >> 64 );
#else
	RV_UINT64 Cp_Lo1_Lo2;
	RV_UINT64 Cp_Lo1_Hi2;
	RV_UINT64 Cp_Hi1_Lo2;
	RV_UINT64 Cp_Hi1_Hi2;
	RV_UINT64 Cross;
	RV_UINT64 Upper;

	//
	// Calculate cross-products of all 32-bit words of Operand1 and Operand2.
	//
	Cp_Lo1_Lo2 = ( ( Operand1 & ( ( 1ull << 32 ) - 1 ) ) * ( Operand2 & ( ( 1ull << 32 ) - 1 ) ) );
	Cp_Lo1_Hi2 = ( ( Operand1 & ( ( 1ull << 32 ) - 1 ) ) * ( Operand2 >> 32ull ) );
	Cp_Hi1_Lo2 = ( ( Operand1 >> 32ull )                 * ( Operand2 & ( ( 1ull << 32 ) - 1 ) ) );
	Cp_Hi1_Hi2 = ( ( Operand1 >> 32ull )                 * ( Operand2 >> 32ull ) );

	//
	// Calculate upper 64-bits of the result.
	//
	Cross = ( ( Cp_Lo1_Lo2 >> 32ull ) + ( Cp_Hi1_Lo2 & ( ( 1ull << 32 ) - 1 ) ) + Cp_Lo1_Hi2 );
	Upper = ( ( Cp_Hi1_Lo2 >> 32ull ) + ( Cross >> 32ull ) + Cp_Hi1_Hi2 );
	return Upper;
#endif
}

//
// S64xS64 -> upper half of 128bit result.
//
static
RV_INT64
RvpSMulh64(
	_Inout_ RV_PROCESSOR* Vp,
	_In_    RV_INT64      Operand1,
	_In_    RV_INT64      Operand2
	)
{
	UNREFERENCED_PARAMETER( Vp );

#ifdef RV_OPT_BUILD_MSVC
	return __mulh( Operand1, Operand2 );
#elif defined(RV_OPT_BUILD_INT128_TYPES)
	return ( ( ( int128_t )Operand1 * ( int128_t )Operand2 ) >> 64 );
#else
	return ( ( ( RV_INT64 )RvpUMulh64( Vp, ( RV_UINT64 )Operand1, ( RV_UINT64 )Operand2 ) )
			- ( ( Operand1 >> 63ull ) & Operand2 )
			- ( ( Operand2 >> 63ull ) & Operand1 ) );
#endif
}


//
// S64xU64 -> upper half of 128bit result.
//
static
RV_INT64
RvpSUMulh64(
	_Inout_ RV_PROCESSOR* Vp,
	_In_    RV_INT64      Operand1,
	_In_    RV_UINT64     Operand2
	)
{
	UNREFERENCED_PARAMETER( Vp );

#if defined(RV_OPT_BUILD_INT128_TYPES)
	return ( ( ( int128_t )Operand1 * ( uint128_t )Operand2 ) >> 64 );
#else
	return ( ( RV_INT64 )RvpUMulh64( Vp, ( RV_UINT64 )Operand1, Operand2 )
			 - ( ( Operand1 >> 63ull ) & ( RV_INT64 )Operand2 ) );
#endif
}

static
VOID
RvpInstructionExecuteOpcodeOpRv32m(
	_Inout_ RV_PROCESSOR* Vp,
	_In_    RV_UINT32     Instruction
	)
{	
	RV_UINT32 Rd;
	RV_UINT32 Rs1;
	RV_UINT32 Rs2;
	RV_UINT32 Funct3;
	RV_UINT32 Funct7;

	//
	// Decode R-type fields.
	//
	Rd     = ( ( Instruction >> RV_INST_R_RD_SHIFT ) & RV_INST_R_RD_MASK );
	Rs1    = ( ( Instruction >> RV_INST_R_RS1_SHIFT ) & RV_INST_R_RS1_MASK );
	Rs2    = ( ( Instruction >> RV_INST_R_RS2_SHIFT ) & RV_INST_R_RS2_MASK );
	Funct3 = ( ( Instruction >> RV_INST_R_FUNCT3_SHIFT ) & RV_INST_R_FUNCT3_MASK );
	Funct7 = ( ( Instruction >> RV_INST_R_FUNCT7_SHIFT ) & RV_INST_R_FUNCT7_MASK );

	//
	// Validate register indices.
	//
	if( Rd >= RV_COUNTOF( Vp->Xr ) || Rs1 >= RV_COUNTOF( Vp->Xr ) || Rs2 >= RV_COUNTOF( Vp->Xr ) ) {
		RvpExceptionPush( Vp, RV_EXCEPTION_ILLEGAL_INSTRUCTION );
		return;
	}

	//
	// Validate funct7, all instructions of RV32M use the same funct7 value.
	//
	if( Funct7 != RV_OP_FUNCT7_RV32M ) {
		RvpExceptionPush( Vp, RV_EXCEPTION_ILLEGAL_INSTRUCTION );
		return;
	}

	//
	// RV32M Standard Extension
	// 0000001 rs2 rs1 000 rd 0110011 MUL
	// 0000001 rs2 rs1 001 rd 0110011 MULH
	// 0000001 rs2 rs1 010 rd 0110011 MULHSU
	// 0000001 rs2 rs1 011 rd 0110011 MULHU
	// 0000001 rs2 rs1 100 rd 0110011 DIV
	// 0000001 rs2 rs1 101 rd 0110011 DIVU
	// 0000001 rs2 rs1 110 rd 0110011 REM
	// 0000001 rs2 rs1 111 rd 0110011 REMU
	//
	switch( Funct3 ) {
	case RV_OP_FUNCT3_RV32M_MUL:
		//
		// MUL performs an XLEN-bit*XLEN-bit multiplication of rs1 by rs2
		// and places the lower XLEN bits in the destination register.
		//
		Vp->Xr[ Rd ] = ( Vp->Xr[ Rs1 ] * Vp->Xr[ Rs2 ] );
		break;
	case RV_OP_FUNCT3_RV32M_MULH:
		//
		// MULH, MULHU, and MULHSU perform the same multiplication but return
		// the upper XLEN bits of the full 2×XLEN-bit product, for signed×signed, unsigned×unsigned,
		// and signed rs1×unsigned rs2 multiplication, respectively.
		//
#if defined(RV_OPT_RV64I)
		Vp->Xr[ Rd ] = ( RV_UINTR )( RV_INTR )RvpSMulh64( Vp, ( RV_INT64 )Vp->Xr[ Rs1 ], ( RV_INT64 )Vp->Xr[ Rs2 ] );
#elif defined(RV_OPT_RV32I)
		Vp->Xr[ Rd ] = ( RV_UINTR )( RV_INTR )( ( ( RV_INT64 )Vp->Xr[ Rs1 ] * ( RV_INT64 )Vp->Xr[ Rs2 ] ) >> 32ull );
#else
		#error "Unsupported XLEN mode."
#endif
		break;
	case RV_OP_FUNCT3_RV32M_MULHU:
#if defined(RV_OPT_RV64I)
		Vp->Xr[ Rd ] = ( RV_UINTR )( RV_INTR )RvpUMulh64( Vp, Vp->Xr[ Rs1 ], Vp->Xr[ Rs2 ] );
#elif defined(RV_OPT_RV32I)
		Vp->Xr[ Rd ] = ( RV_UINTR )( ( ( RV_UINT64 )Vp->Xr[ Rs1 ] * ( RV_UINT64 )Vp->Xr[ Rs2 ] ) >> 32ull );
#else
		#error "Unsupported XLEN mode."
#endif
		break;
	case RV_OP_FUNCT3_RV32M_MULHSU:
#if defined(RV_OPT_RV64I)
	Vp->Xr[ Rd ] = ( RV_UINTR )( RV_INTR )RvpSUMulh64( Vp, Vp->Xr[ Rs1 ], Vp->Xr[ Rs2 ] );
#elif defined(RV_OPT_RV32I)
		Vp->Xr[ Rd ] = ( RV_UINTR )( RV_INTR )( ( ( RV_INT64 )Vp->Xr[ Rs1 ] * ( RV_UINT64 )Vp->Xr[ Rs2 ] ) >> 32ull );
#else
		#error "Unsupported XLEN mode."
#endif
		break;
	case RV_OP_FUNCT3_RV32M_DIV:
		//
		// DIV performs an XLEN bits by XLEN bits signed integer division of rs1 by rs2, rounding towards zero.
		// The quotient of division by zero has all bits set.
		//
		if( Vp->Xr[ Rs2 ] != 0 ) {
			Vp->Xr[ Rd ] = ( RV_UINTR )( ( RV_INTR )Vp->Xr[ Rs1 ] / ( RV_INTR )Vp->Xr[ Rs2 ] );
		} else {
			Vp->Xr[ Rd ] = ~( ( RV_UINTR )0 );
		}
		break;
	case RV_OP_FUNCT3_RV32M_DIVU:
		//
		// DIVU performs an XLEN bits by XLEN bits unsigned integer division of rs1 by rs2, rounding towards zero.
		// The quotient of division by zero has all bits set.
		//
		if( Vp->Xr[ Rs2 ] != 0 ) {
			Vp->Xr[ Rd ] = ( Vp->Xr[ Rs1 ] / Vp->Xr[ Rs2 ] );
		} else {
			Vp->Xr[ Rd ] = ~( ( RV_UINTR )0 );
		}
		break;
	case RV_OP_FUNCT3_RV32M_REM:
		//
		// REM and REMU provide the remainder of the corresponding division
		// operation. For REM, the sign of the result equals the sign of the dividend.
		// The remainder of division by zero equals the dividend.
		//
		if( Vp->Xr[ Rs2 ] != 0 ) {
			Vp->Xr[ Rd ] = ( RV_UINTR )( ( RV_INTR )Vp->Xr[ Rs1 ] % ( RV_INTR )Vp->Xr[ Rs2 ] );
		} else {
			Vp->Xr[ Rd ] = Vp->Xr[ Rs1 ];
		}
		break;
	case RV_OP_FUNCT3_RV32M_REMU:
		if( Vp->Xr[ Rs2 ] != 0 ) {
			Vp->Xr[ Rd ] = ( Vp->Xr[ Rs1 ] % Vp->Xr[ Rs2 ] );
		} else {
			Vp->Xr[ Rd ] = Vp->Xr[ Rs1 ];
		}
		break;
	default:
		RvpExceptionPush( Vp, RV_EXCEPTION_ILLEGAL_INSTRUCTION );
		return;
	}

	Vp->Pc += 4;
}
#endif

static
VOID
RvpInstructionExecuteOpcodeOp(
	_Inout_ RV_PROCESSOR* Vp,
	_In_    RV_UINT32     Instruction
	)
{
	RV_UINT32 Opcode;
	RV_UINT32 Rd;
	RV_UINT32 Rs1;
	RV_UINT32 Rs2;
	RV_UINT32 Funct3;
	RV_UINT32 Funct7;
	RV_UINT32 Class;

	//
	// Decode R-type fields.
	//
	Opcode = ( ( Instruction >> RV_INST_R_OPCODE_SHIFT ) & RV_INST_R_OPCODE_MASK );
	Rd     = ( ( Instruction >> RV_INST_R_RD_SHIFT ) & RV_INST_R_RD_MASK );
	Rs1    = ( ( Instruction >> RV_INST_R_RS1_SHIFT ) & RV_INST_R_RS1_MASK );
	Rs2    = ( ( Instruction >> RV_INST_R_RS2_SHIFT ) & RV_INST_R_RS2_MASK );
	Funct3 = ( ( Instruction >> RV_INST_R_FUNCT3_SHIFT ) & RV_INST_R_FUNCT3_MASK );
	Funct7 = ( ( Instruction >> RV_INST_R_FUNCT7_SHIFT ) & RV_INST_R_FUNCT7_MASK );

	//
	// Validate register indices.
	//
	if( Rd >= RV_COUNTOF( Vp->Xr ) || Rs1 >= RV_COUNTOF( Vp->Xr ) || Rs2 >= RV_COUNTOF( Vp->Xr ) ) {
		RvpExceptionPush( Vp, RV_EXCEPTION_ILLEGAL_INSTRUCTION );
		return;
	}

	//
	// If the RV32M standard extension is enabled, dispatch to the corresponding handler.
	//
#if defined(RV_OPT_RV32M)
	if( Funct7 == RV_OP_FUNCT7_RV32M ) {
		RvpInstructionExecuteOpcodeOpRv32m( Vp, Instruction );
		return;
	}
#endif

	//
	// Clasify instruction by combining opcode, funct3, funct7.
	//
	Class = RV_INST_CLASSIFY_F3F7( Opcode, Funct3, Funct7 );

	//
	// Base opcode RV_OPCODE_OP funct3 values (R-type).
	// 0000000 rs2 rs1 000 rd 0110011 ADD
	// 0100000 rs2 rs1 000 rd 0110011 SUB
	// 0000000 rs2 rs1 001 rd 0110011 SLL
	// 0000000 rs2 rs1 010 rd 0110011 SLT
	// 0000000 rs2 rs1 011 rd 0110011 SLTU
	// 0000000 rs2 rs1 100 rd 0110011 XOR
	// 0000000 rs2 rs1 101 rd 0110011 SRL
	// 0100000 rs2 rs1 101 rd 0110011 SRA
	// 0000000 rs2 rs1 110 rd 0110011 OR
	// 0000000 rs2 rs1 111 rd 0110011 AND
	//
	switch( Class ) {
	case RV_INST_CLASSIFY_F3F7( RV_OPCODE_OP, RV_OP_FUNCT3_ADD_SUB, RV_OP_FUNCT7_ADD ):
		Vp->Xr[ Rd ] = ( Vp->Xr[ Rs1 ] + Vp->Xr[ Rs2 ] );
		break;
	case RV_INST_CLASSIFY_F3F7( RV_OPCODE_OP, RV_OP_FUNCT3_ADD_SUB, RV_OP_FUNCT7_SUB ):
		Vp->Xr[ Rd ] = ( Vp->Xr[ Rs1 ] - Vp->Xr[ Rs2 ] );
		break;
	case RV_INST_CLASSIFY_F3F7( RV_OPCODE_OP, RV_OP_FUNCT3_SLT, RV_OP_FUNCT7_SLT ):
		Vp->Xr[ Rd ] = ( ( ( RV_INTR )Vp->Xr[ Rs1 ] < ( RV_INTR )Vp->Xr[ Rs2 ] ) ? 1 : 0 );
		break;
	case RV_INST_CLASSIFY_F3F7( RV_OPCODE_OP, RV_OP_FUNCT3_SLTU, RV_OP_FUNCT7_SLTU ):
		Vp->Xr[ Rd ] = ( ( Vp->Xr[ Rs1 ] < Vp->Xr[ Rs2 ] ) ? 1 : 0 );
		break;
	case RV_INST_CLASSIFY_F3F7( RV_OPCODE_OP, RV_OP_FUNCT3_XOR, RV_OP_FUNCT7_XOR ):
		Vp->Xr[ Rd ] = ( Vp->Xr[ Rs1 ] ^ Vp->Xr[ Rs2 ] );
		break;
	case RV_INST_CLASSIFY_F3F7( RV_OPCODE_OP, RV_OP_FUNCT3_SLL, RV_OP_FUNCT7_SLL ):
		Vp->Xr[ Rd ] = ( Vp->Xr[ Rs1 ] << ( Vp->Xr[ Rs2 ] & 0x3f ) );
		break;
	case RV_INST_CLASSIFY_F3F7( RV_OPCODE_OP, RV_OP_FUNCT3_SRL_SRA, RV_OP_FUNCT7_SRL ):
		Vp->Xr[ Rd ] = ( Vp->Xr[ Rs1 ] >> ( Vp->Xr[ Rs2 ] & 0x3f ) );
		break;
	case RV_INST_CLASSIFY_F3F7( RV_OPCODE_OP, RV_OP_FUNCT3_SRL_SRA, RV_OP_FUNCT7_SRA ):
		Vp->Xr[ Rd ] = ( ( RV_INTR )Vp->Xr[ Rs1 ] >> ( Vp->Xr[ Rs2 ] & 0x3f ) );
		break;
	case RV_INST_CLASSIFY_F3F7( RV_OPCODE_OP, RV_OP_FUNCT3_OR, RV_OP_FUNCT7_OR ):
		Vp->Xr[ Rd ] = ( Vp->Xr[ Rs1 ] | Vp->Xr[ Rs2 ] );
		break;
	case RV_INST_CLASSIFY_F3F7( RV_OPCODE_OP, RV_OP_FUNCT3_AND, RV_OP_FUNCT7_AND ):
		Vp->Xr[ Rd ] = ( Vp->Xr[ Rs1 ] & Vp->Xr[ Rs2 ] );
		break;
	default:
		RvpExceptionPush( Vp, RV_EXCEPTION_ILLEGAL_INSTRUCTION );
		return;
	}

	Vp->Pc += 4;
}

static
VOID
RvpInstructionExecuteOpcodeBranch(
	_Inout_ RV_PROCESSOR* Vp,
	_In_    RV_UINT32     Instruction
	)
{
	RV_UINT32  Funct3;
	RV_UINT32  Imm_12;
	RV_UINT32  Imm_10_5;
	RV_UINT32  Imm_4_1;
	RV_UINT32  Imm_11;
	RV_UINT32  Rs1;
	RV_UINT32  Rs2;
	RV_UINT32  ImmFull;
	RV_INT32   SDisp32;
	RV_BOOLEAN BranchTaken;
	RV_UINTR   DestAddress;

	//
	// All branch instructions use the B-type instruction format.
	// Decode B-type fields.
	//
	Funct3   = ( ( Instruction >> RV_INST_B_FUNCT3_SHIFT ) & RV_INST_B_FUNCT3_MASK );
	Imm_12   = ( ( Instruction >> RV_INST_B_IMM_12_SHIFT ) & RV_INST_B_IMM_12_MASK );
	Imm_10_5 = ( ( Instruction >> RV_INST_B_IMM_10_5_SHIFT ) & RV_INST_B_IMM_10_5_MASK );
	Imm_4_1  = ( ( Instruction >> RV_INST_B_IMM_4_1_SHIFT ) & RV_INST_B_IMM_4_1_MASK );
	Imm_11   = ( ( Instruction >> RV_INST_B_IMM_11_SHIFT ) & RV_INST_B_IMM_11_MASK );
	Rs1      = ( ( Instruction >> RV_INST_B_RS1_SHIFT ) & RV_INST_B_RS1_MASK );
	Rs2      = ( ( Instruction >> RV_INST_B_RS2_SHIFT ) & RV_INST_B_RS2_MASK );

	//
	// Build full immediate value.
	//
	ImmFull = ( ( Imm_12 << 12ul ) | ( Imm_11 << 11ul ) | ( Imm_10_5 << 5ul ) | ( Imm_4_1 << 1ul ) );

	//
	// Validate register indices.
	//
	if( Rs1 >= RV_COUNTOF( Vp->Xr ) || Rs2 >= RV_COUNTOF( Vp->Xr ) ) {
		RvpExceptionPush( Vp, RV_EXCEPTION_ILLEGAL_INSTRUCTION );
		return;
	}

	//
	// The 12-bit B-immediate encodes signed offsets in multiples of 2 bytes.
	//
	SDisp32 = ( RV_INT32 )RvpSignExtend32( Vp, ImmFull, 12 );

	//
	// Base opcode RV_OPCODE_BRANCH funct3 values (B-type).
	// imm[12|10:5] rs2 rs1 000 imm[4:1|11] 1100011 BEQ
	// imm[12|10:5] rs2 rs1 001 imm[4:1|11] 1100011 BNE
	// imm[12|10:5] rs2 rs1 100 imm[4:1|11] 1100011 BLT
	// imm[12|10:5] rs2 rs1 101 imm[4:1|11] 1100011 BGE
	// imm[12|10:5] rs2 rs1 110 imm[4:1|11] 1100011 BLTU
	// imm[12|10:5] rs2 rs1 111 imm[4:1|11] 1100011 BGEU
	//
	switch( Funct3 ) {
	case RV_BRANCH_FUNCT3_BEQ:
		BranchTaken = ( Vp->Xr[ Rs1 ] == Vp->Xr[ Rs2 ] );
		break;
	case RV_BRANCH_FUNCT3_BNE:
		BranchTaken = ( Vp->Xr[ Rs1 ] != Vp->Xr[ Rs2 ] );
		break;
	case RV_BRANCH_FUNCT3_BLT:
		BranchTaken = ( ( RV_INTR )Vp->Xr[ Rs1 ] < ( RV_INTR )Vp->Xr[ Rs2 ] );
		break;
	case RV_BRANCH_FUNCT3_BLTU:
		BranchTaken = ( Vp->Xr[ Rs1 ] < Vp->Xr[ Rs2 ] );
		break;
	case RV_BRANCH_FUNCT3_BGE:
		BranchTaken = ( ( RV_INTR )Vp->Xr[ Rs1 ] >= ( RV_INTR )Vp->Xr[ Rs2 ] );
		break;
	case RV_BRANCH_FUNCT3_BGEU:
		BranchTaken = ( Vp->Xr[ Rs1 ] >= Vp->Xr[ Rs2 ] );
		break;
	default:
		RvpExceptionPush( Vp, RV_EXCEPTION_ILLEGAL_INSTRUCTION );
		return;
	}

	//
	// The offset is sign-extended and added to the address of the branch
	// instruction to give the target address. The conditional branch range is +-4 KiB.
	//
	DestAddress = ( Vp->Pc + ( RV_INTR )SDisp32 );

	//
	// An instruction-address-misaligned exception is generated on a taken branch or unconditional jump
	// if the target address is not four-byte aligned. This exception is reported on the branch or jump
	// instruction, not on the target instruction. No instruction-address-misaligned exception is generated
	// for a conditional branch that is not taken.
	//
	if( BranchTaken ) {
		if( ( DestAddress & ( 4 - 1 ) ) != 0 ) {
			RvpExceptionPush( Vp, RV_EXCEPTION_INSTRUCTION_ADDRESS_MISALIGNED );
			return;
		}
		Vp->Pc = DestAddress;
	} else {
		Vp->Pc += 4;
	}
}

static
VOID
RvpInstructionExecuteOpcodeJALR(
	_Inout_ RV_PROCESSOR* Vp,
	_In_    RV_UINT32     Instruction
	)
{
	RV_UINT32 Rd;
	RV_UINT32 Rs1;
	RV_UINT32 Imm_11_0;
	RV_INT32  SDisp32;
	RV_UINTR  TargetAddress;

	//
	// Decode I-type fields.
	//
	Rd       = ( ( Instruction >> RV_INST_I_RD_SHIFT ) & RV_INST_I_RD_MASK );
	Rs1      = ( ( Instruction >> RV_INST_I_RS1_SHIFT ) & RV_INST_I_RS1_MASK );
	Imm_11_0 = ( ( Instruction >> RV_INST_I_IMM_11_0_SHIFT ) & RV_INST_I_IMM_11_0_MASK );

	//
	// Validate register indices.
	//
	if( Rd >= RV_COUNTOF( Vp->Xr ) || Rs1 >= RV_COUNTOF( Vp->Xr ) ) {
		RvpExceptionPush( Vp, RV_EXCEPTION_ILLEGAL_INSTRUCTION );
		return;
	}

	//
	// The target address is obtained by adding the sign-extended 12-bit I-immediate to
	// the register rs1, then setting the least-significant bit of the result to zero.
	//
	SDisp32 = ( RV_INT32 )RvpSignExtend32( Vp, Imm_11_0, 11 );
	TargetAddress = ( ( Vp->Xr[ Rs1 ] + ( RV_INTR )SDisp32 ) & ~( ( RV_UINTR )1 ) );

	//
	// The JAL and JALR instructions will generate an instruction-address-misaligned
	// exception if the target address is not aligned to a four-byte boundary.
	//
	if( ( TargetAddress & ( 4 - 1 ) ) != 0 ) {
		RvpExceptionPush( Vp, RV_EXCEPTION_INSTRUCTION_ADDRESS_MISALIGNED );
		return;
	}

	//
	// The address of the instruction following the jump (pc+4) is written to register rd.
	// Register x0 can be used as the destination if the result is not required.
	//
	Vp->Xr[ Rd ] = ( Vp->Pc + 4 );

	//
	// Jump to target address.
	//
	Vp->Pc = TargetAddress;
}

static
VOID
RvpInstructionExecuteOpcodeJAL(
	_Inout_ RV_PROCESSOR* Vp,
	_In_    RV_UINT32     Instruction
	)
{
	RV_UINT32 Rd;
	RV_UINT32 Imm_20;
	RV_UINT32 Imm_10_1;
	RV_UINT32 Imm_11;
	RV_UINT32 Imm_19_12;
	RV_UINT32 ImmFull;
	RV_INT32  SDisp32;
	RV_UINTR  TargetAddress;

	//
	// Decode J-type fields.
	//
	Rd        = ( ( Instruction >> RV_INST_J_RD_SHIFT ) & RV_INST_J_RD_MASK );
	Imm_20    = ( ( Instruction >> RV_INST_J_IMM_20_SHIFT ) & RV_INST_J_IMM_20_MASK );
	Imm_10_1  = ( ( Instruction >> RV_INST_J_IMM_10_1_SHIFT ) & RV_INST_J_IMM_10_1_MASK );
	Imm_11    = ( ( Instruction >> RV_INST_J_IMM_11_SHIFT ) & RV_INST_J_IMM_11_MASK );
	Imm_19_12 = ( ( Instruction >> RV_INST_J_IMM_19_12_SHIFT ) & RV_INST_J_IMM_19_12_MASK );

	//
	// Validate register indices.
	//
	if( Rd >= RV_COUNTOF( Vp->Xr ) ) {
		RvpExceptionPush( Vp, RV_EXCEPTION_ILLEGAL_INSTRUCTION );
		return;
	}

	//
	// Form full immediate value and calculate sign-extended displacement.
	//
	ImmFull = ( ( Imm_20 << 20ul ) | ( Imm_19_12 << 12ul ) | ( Imm_11 << 11ul ) | ( Imm_10_1 << 1ul ) );
	SDisp32 = ( RV_INT32 )RvpSignExtend32( Vp, ImmFull, 20 );

	//
	// The the J-immediate encodes a signed offset in multiples of 2 bytes.
	// The offset is sign-extended and added to the address of the jump instruction
	// to form the jump target address. Jumps can therefore target a +-1 MiB range.
	//
	TargetAddress = ( Vp->Pc + ( RV_INTR )SDisp32 );

	//
	// The JAL and JALR instructions will generate an instruction-address-misaligned
	// exception if the target address is not aligned to a four-byte boundary.
	//
	if( ( TargetAddress & ( 4 - 1 ) ) != 0 ) {
		RvpExceptionPush( Vp, RV_EXCEPTION_INSTRUCTION_ADDRESS_MISALIGNED );
		return;
	}

	//
	// JAL stores the address of the instruction following the jump (pc+4) into register rd.
	//
	Vp->Xr[ Rd ] = ( Vp->Pc + 4 );

	//
	// Jump to target address.
	//
	Vp->Pc = TargetAddress;
}


//
// System opcode and Zicsr implementation.
//


_Success_( return )
static
RV_BOOLEAN
RvpCsrHandleWrite(
	_Inout_ RV_PROCESSOR* Vp,
	_In_    RV_UINT32     Csr,
	_In_    RV_UINTR      NewValue
	)
{
	switch( Csr ) {
#if (defined(RV_OPT_RV32F) || defined(RV_OPT_RV32D))
	case RV_CSR_VALUE_FFLAGS:
		//
		// FFLAGS is a helper CSR that accesses just the FFLAGS field of the actual FCSR.
		//
		Vp->CsrFcsr &= ~( RV_FCSR_FFLAGS_MASK << RV_FCSR_FFLAGS_SHIFT );
		Vp->CsrFcsr |= ( ( NewValue & RV_FCSR_FFLAGS_MASK ) << RV_FCSR_FFLAGS_SHIFT );
		break;
	case RV_CSR_VALUE_FRM:
		//
		// FRM is a helper CSR that accesses just the FRM field of the actual FCSR.
		//
		Vp->CsrFcsr &= ~( RV_FCSR_FRM_MASK << RV_FCSR_FRM_SHIFT );
		Vp->CsrFcsr |= ( ( NewValue & RV_FCSR_FRM_MASK ) << RV_FCSR_FRM_SHIFT );
		break;
	case RV_CSR_VALUE_FCSR:
		//
		// Bits 31-8 of the fcsr are reserved for other standard extensions, including the "L" standard
		// extension for decimal floating-point. If these extensions are not present, implementations shall
		// ignore writes to these bits and supply a zero value when read. Standard software should preserve
		// the contents of these bits.
		//
		Vp->CsrFcsr = ( NewValue & ~( RV_FCSR_RESERVED0_MASK << RV_FCSR_RESERVED0_SHIFT ) );
		break;
#endif
	default:
		return RV_FALSE;
	}

	return RV_TRUE;
}

_Success_( return )
static
RV_BOOLEAN
RvpCsrHandleRead(
	_Inout_ RV_PROCESSOR* Vp,
	_In_    RV_UINT32     Csr,
	_Out_   RV_UINTR*     pValue
	)
{
	switch( Csr ) {
#if (defined(RV_OPT_RV32F) || defined(RV_OPT_RV32D))
	case RV_CSR_VALUE_FFLAGS:
		//
		// FFLAGS is a helper CSR that accesses just the FFLAGS field of the actual FCSR.
		//
		*pValue = ( Vp->CsrFcsr & ( RV_FCSR_FFLAGS_MASK << RV_FCSR_FFLAGS_SHIFT ) );
		break;
	case RV_CSR_VALUE_FRM:
		//
		// FRM is a helper CSR that accesses just the FRM field of the actual FCSR.
		//
		*pValue = ( Vp->CsrFcsr & ( RV_FCSR_FRM_MASK << RV_FCSR_FRM_SHIFT ) );
		break;
	case RV_CSR_VALUE_FCSR:
		//
		// Bits 31-8 of the fcsr are reserved for other standard extensions, including the "L" standard
		// extension for decimal floating-point. If these extensions are not present, implementations shall
		// ignore writes to these bits and supply a zero value when read. Standard software should preserve
		// the contents of these bits.
		//
		*pValue = ( Vp->CsrFcsr & ~( RV_FCSR_RESERVED0_MASK << RV_FCSR_RESERVED0_SHIFT ) );
		break;
#endif
	case RV_CSR_VALUE_CYCLE:
		*pValue = ( RV_UINTR )Vp->CsrCycleCount;
		break;
	case RV_CSR_VALUE_TIME:
		*pValue = ( RV_UINTR )Vp->CsrTime;
		break;
	case RV_CSR_VALUE_INSTRET:
		*pValue = ( RV_UINTR )Vp->CsrInstRetired;
		break;
#if defined (RV_OPT_RV32I) && !defined(RV_OPT_RV64I)
	case RV_CSR_VALUE_CYCLEH:
		*pValue = ( RV_UINT32 )( Vp->CsrCycleCount >> 32ull );
		break;
	case RV_CSR_VALUE_TIMEH:
		*pValue = ( RV_UINT32 )( Vp->CsrTime >> 32ull );
		break;
	case RV_CSR_VALUE_INSTRETH:
		*pValue = ( RV_UINT32 )( Vp->CsrTime >> 32ull );
		break;
#endif
	default:
		return RV_FALSE;
	}

	return RV_TRUE;
}

static
VOID
RvpInstructionExecuteOpcodeSystem(
	_Inout_ RV_PROCESSOR* Vp,
	_In_    RV_UINT32     Instruction
	)
{
	RV_UINT32 Rd;
	RV_UINT32 Rs1;
	RV_UINT32 Funct3;
	RV_UINT32 Funct12;
	RV_UINT32 CsrIndex;
	RV_UINTR  CsrOldValue;

	//
	// Decode I-type fields.
	//
	Rd      = ( ( Instruction >> RV_INST_I_RD_SHIFT ) & RV_INST_I_RD_MASK );
	Rs1     = ( ( Instruction >> RV_INST_I_RS1_SHIFT ) & RV_INST_I_RS1_MASK );
	Funct3  = ( ( Instruction >> RV_INST_I_FUNCT3_SHIFT ) & RV_INST_I_FUNCT3_MASK );
	Funct12 = ( ( Instruction >> RV_INST_I_IMM_11_0_SHIFT ) & RV_INST_I_IMM_11_0_MASK );

	//
	// Validate register indices.
	//
	if( Rd >= RV_COUNTOF( Vp->Xr ) || Rs1 >= RV_COUNTOF( Vp->Xr ) ) {
		RvpExceptionPush( Vp, RV_EXCEPTION_ILLEGAL_INSTRUCTION );
		return;
	}

	//
	// All CSR instructions atomically read-modify-write a single CSR, whose CSR specifier is encoded
	// in the 12-bit csr field of the instruction held in bits 31–20. The immediate forms use a 5-bit
	// zero-extended immediate encoded in the rs1 field.
	//
	CsrIndex = Funct12;

	//
	// SYSTEM instructions are used to access system functionality that might
	// require privileged access and are encoded using the I-type instruction format.
	//
	switch( Funct3 ) {
	case RV_SYSTEM_FUNCT3_ECALL_EBREAK:
		if( Funct12 == RV_SYSTEM_FUNCT12_ECALL ) {
			Vp->ECallPending = 1;
		} else if( Funct12 == RV_SYSTEM_FUNCT12_EBREAK ) {
			Vp->EBreakPending = 1;
		} else {
			RvpExceptionPush( Vp, RV_EXCEPTION_ILLEGAL_INSTRUCTION );
			return;
		}
		break;
	case RV_SYSTEM_FUNCT3_CSRRW:
		//
		// The CSRRW (Atomic Read/Write CSR) instruction atomically swaps values in the CSRs and
		// integer registers. CSRRW reads the old value of the CSR, zero-extends the value to XLEN bits,
		// then writes it to integer register rd. The initial value in rs1 is written to the CSR.
		//
		if( RvpCsrHandleRead( Vp, CsrIndex, &CsrOldValue ) == RV_FALSE ) {
			//
			// If rd=x0, then the instruction shall not read the CSR and shall not cause any of the side
			// effects that might occur on a CSR read.
			//
			if( Rd != 0 ) {
				RvpExceptionPush( Vp, RV_EXCEPTION_ILLEGAL_INSTRUCTION );
				return;
			}
		}

		//
		// Update CSR value, if not writable, throw illegal instruction exception.
		//
		if( RvpCsrHandleWrite( Vp, CsrIndex, Vp->Xr[ Rs1 ] ) == RV_FALSE ) {
			RvpExceptionPush( Vp, RV_EXCEPTION_ILLEGAL_INSTRUCTION );
			return;
		}

		Vp->Xr[ Rd ] = CsrOldValue;
		break;
	case RV_SYSTEM_FUNCT3_CSRRS:
		//
		// The CSRRS (Atomic Read and Set Bits in CSR) instruction reads the value of the CSR, zeroextends
		// the value to XLEN bits, and writes it to integer register rd. The initial value in integer
		// register rs1 is treated as a bit mask that specifies bit positions to be set in the CSR. Any bit that
		// is high in rs1 will cause the corresponding bit to be set in the CSR, if that CSR bit is writable.
		// Other bits in the CSR are unaffected (though CSRs might have side effects when written).
		//
		if( RvpCsrHandleRead( Vp, CsrIndex, &CsrOldValue ) == RV_FALSE ) {
			//
			// Both CSRRS and CSRR will always read the CSR and cause any read side effects regardless of rd and rs1 fields.
			//
			RvpExceptionPush( Vp, RV_EXCEPTION_ILLEGAL_INSTRUCTION );
			return;
		}
			
		//
		// For both CSRRS and CSRRC, if rs1=x0, then the instruction will not write to the CSR at all, and
		// so shall not cause any of the side effects that might otherwise occur on a CSR write, such as raising
		// illegal instruction exceptions on accesses to read-only CSRs.
		//
		if( Rs1 != 0 ) {
			//
			// Update CSR value, if not writable, throw illegal instruction exception.
			//
			if( RvpCsrHandleWrite( Vp, CsrIndex, ( CsrOldValue | Vp->Xr[ Rs1 ] ) ) == RV_FALSE ) {
				RvpExceptionPush( Vp, RV_EXCEPTION_ILLEGAL_INSTRUCTION );
				return;
			}
		}

		Vp->Xr[ Rd ] = CsrOldValue;
		break;
	case RV_SYSTEM_FUNCT3_CSRRC:
		//
		// The CSRRC (Atomic Read and Clear Bits in CSR) instruction reads the value of the CSR, zeroextends
		// the value to XLEN bits, and writes it to integer register rd. The initial value in integer
		// register rs1 is treated as a bit mask that specifies bit positions to be cleared in the CSR. Any bit
		// that is high in rs1 will cause the corresponding bit to be cleared in the CSR, if that CSR bit is
		// writable. Other bits in the CSR are unaffected.
		//
		if( RvpCsrHandleRead( Vp, CsrIndex, &CsrOldValue ) == RV_FALSE ) {
			//
			// Both CSRRS and CSRRC will always read the CSR and cause any read side effects regardless of rd and rs1 fields.
			//
			RvpExceptionPush( Vp, RV_EXCEPTION_ILLEGAL_INSTRUCTION );
			return;
		}
			
		//
		// For both CSRRS and CSRRC, if rs1=x0, then the instruction will not write to the CSR at all, and
		// so shall not cause any of the side effects that might otherwise occur on a CSR write, such as raising
		// illegal instruction exceptions on accesses to read-only CSRs.
		//
		if( Rs1 != 0 ) {
			//
			// Update CSR value, if not writable, throw illegal instruction exception.
			//
			if( RvpCsrHandleWrite( Vp, CsrIndex, ( CsrOldValue & ~Vp->Xr[ Rs1 ] ) ) == RV_FALSE ) {
				RvpExceptionPush( Vp, RV_EXCEPTION_ILLEGAL_INSTRUCTION );
				return;
			}
		}

		Vp->Xr[ Rd ] = CsrOldValue;
		break;
	case RV_SYSTEM_FUNCT3_CSRRWI:
		//
		// The CSRRWI, CSRRSI, and CSRRCI variants are similar to CSRRW, CSRRS, and CSRRC respectively,
		// except they update the CSR using an XLEN-bit value obtained by zero-extending a 5-bit
		// unsigned immediate (uimm[4:0]) field encoded in the rs1 field instead of a value from an integer
		// register.
		//
		if( RvpCsrHandleRead( Vp, CsrIndex, &CsrOldValue ) == RV_FALSE ) {
			//
			// For CSRRWI, if rd=x0, then the instruction shall not read the CSR and shall not cause any
			// of the side effects that might occur on a CSR read.
			//
			if( Rd != 0 ) {
				RvpExceptionPush( Vp, RV_EXCEPTION_ILLEGAL_INSTRUCTION );
				return;
			}
		}

		//
		// Update CSR value, if not writable, throw illegal instruction exception.
		//
		if( RvpCsrHandleWrite( Vp, CsrIndex, ( RV_UINTR )Rs1 ) == RV_FALSE ) {
			RvpExceptionPush( Vp, RV_EXCEPTION_ILLEGAL_INSTRUCTION );
			return;
		}

		Vp->Xr[ Rd ] = CsrOldValue;
		break;
	case RV_SYSTEM_FUNCT3_CSRRSI:
		if( RvpCsrHandleRead( Vp, CsrIndex, &CsrOldValue ) == RV_FALSE ) {
			//
			// Both CSRRSI and CSRRCI will always read the CSR and cause any read side effects regardless of rd and rs1 fields.
			//
			RvpExceptionPush( Vp, RV_EXCEPTION_ILLEGAL_INSTRUCTION );
			return;
		}

		//
		// For CSRRSI and CSRRCI, if the uimm[4:0] field is zero, then these instructions will not
		// write to the CSR, and shall not cause any of the side effects that might otherwise occur on a CSR
		// write.
		//
		if( Rs1 != 0 ) {
			//
			// Update CSR value, if not writable, throw illegal instruction exception.
			//
			if( RvpCsrHandleWrite( Vp, CsrIndex, ( CsrOldValue | ( ( RV_UINTR )Rs1 ) ) ) == RV_FALSE ) {
				RvpExceptionPush( Vp, RV_EXCEPTION_ILLEGAL_INSTRUCTION );
				return;
			}
		}

		Vp->Xr[ Rd ] = CsrOldValue;
		break;
	case RV_SYSTEM_FUNCT3_CSRRCI:
		if( RvpCsrHandleRead( Vp, CsrIndex, &CsrOldValue ) == RV_FALSE ) {
			//
			// Both CSRRSI and CSRRCI will always read the CSR and cause any read side effects regardless of rd and rs1 fields.
			//
			RvpExceptionPush( Vp, RV_EXCEPTION_ILLEGAL_INSTRUCTION );
			return;
		}

		//
		// For CSRRSI and CSRRCI, if the uimm[4:0] field is zero, then these instructions will not
		// write to the CSR, and shall not cause any of the side effects that might otherwise occur on a CSR
		// write.
		//
		if( Rs1 != 0 ) {
			//
			// Update CSR value, if not writable, throw illegal instruction exception.
			//
			if( RvpCsrHandleWrite( Vp, CsrIndex, ( CsrOldValue & ~( ( RV_UINTR )Rs1 ) ) ) == RV_FALSE ) {
				RvpExceptionPush( Vp, RV_EXCEPTION_ILLEGAL_INSTRUCTION );
				return;
			}
		}

		Vp->Xr[ Rd ] = CsrOldValue;
		break;
	default:
		RvpExceptionPush( Vp, RV_EXCEPTION_ILLEGAL_INSTRUCTION );
		return;
	}

	Vp->Pc += 4;
}


//
// RV64I opcodes.
//
#if defined(RV_OPT_RV64I)

//
// RV64M opcodes.
//
#if defined(RV_OPT_RV32M)

static
VOID
RvpInstructionExecuteOpcodeOp32Rv64m(
	_Inout_ RV_PROCESSOR* Vp,
	_In_    RV_UINT32     Instruction
	)
{
	RV_UINT32 Rd;
	RV_UINT32 Rs1;
	RV_UINT32 Rs2;
	RV_UINT32 Funct3;
	RV_UINT32 Funct7;

	//
	// Decode R-type fields.
	//
	Rd     = ( ( Instruction >> RV_INST_R_RD_SHIFT ) & RV_INST_R_RD_MASK );
	Rs1    = ( ( Instruction >> RV_INST_R_RS1_SHIFT ) & RV_INST_R_RS1_MASK );
	Rs2    = ( ( Instruction >> RV_INST_R_RS2_SHIFT ) & RV_INST_R_RS2_MASK );
	Funct3 = ( ( Instruction >> RV_INST_R_FUNCT3_SHIFT ) & RV_INST_R_FUNCT3_MASK );
	Funct7 = ( ( Instruction >> RV_INST_R_FUNCT7_SHIFT ) & RV_INST_R_FUNCT7_MASK );

	//
	// Validate register indices.
	//
	if( Rd >= RV_COUNTOF( Vp->Xr ) || Rs1 >= RV_COUNTOF( Vp->Xr ) || Rs2 >= RV_COUNTOF( Vp->Xr ) ) {
		RvpExceptionPush( Vp, RV_EXCEPTION_ILLEGAL_INSTRUCTION );
		return;
	}

	//
	// Validate funct7, all instructions of RV64M use the same funct7 value.
	//
	if( Funct7 != RV_OP_32_FUNCT7_RV64M ) {
		RvpExceptionPush( Vp, RV_EXCEPTION_ILLEGAL_INSTRUCTION );
		return;
	}

	//
	// RV64M Standard Extension
	// 0000001 rs2 rs1 000 rd 0111011 MULW
	// 0000001 rs2 rs1 100 rd 0111011 DIVW
	// 0000001 rs2 rs1 101 rd 0111011 DIVUW
	// 0000001 rs2 rs1 110 rd 0111011 REMW
	// 0000001 rs2 rs1 111 rd 0111011 REMUW
	//
	switch( Funct3 ) {
	case RV_OP_32_FUNCT3_RV64M_MULW:
		//
		// MULW is an RV64 instruction that multiplies the lower 32 bits of the source registers, placing the
		// sign-extension of the lower 32 bits of the result into the destination register.
		//
		Vp->Xr[ Rd ] = ( RV_INTR )( RV_INT32 )( ( RV_UINT32 )Vp->Xr[ Rs1 ] * ( RV_UINT32 )Vp->Xr[ Rs2 ] );
		break;
	case RV_OP_32_FUNCT3_RV64M_DIVW:
		//
		// DIVW and DIVUW are RV64 instructions that divide the lower 32 bits of rs1 by the lower 32
		// bits of rs2, treating them as signed and unsigned integers respectively, placing the 32-bit quotient
		// in rd, sign-extended to 64 bits.
		//
		if( Vp->Xr[ Rs2 ] != 0 ) {
			Vp->Xr[ Rd ] = ( RV_INTR )( ( RV_INT32 )Vp->Xr[ Rs1 ] / ( RV_INT32 )Vp->Xr[ Rs2 ] );
		} else {
			Vp->Xr[ Rd ] = ~( ( RV_UINTR )0 );
		}
		break;
	case RV_OP_32_FUNCT3_RV64M_DIVUW:
		//
		// DIVU performs an XLEN bits by XLEN bits unsigned integer division of rs1 by rs2, rounding towards zero.
		// The quotient of division by zero has all bits set.
		//
		if( Vp->Xr[ Rs2 ] != 0 ) {
			Vp->Xr[ Rd ] = ( RV_INTR )( RV_INT32 )( ( RV_UINT32 )Vp->Xr[ Rs1 ] / ( RV_UINT32 )Vp->Xr[ Rs2 ] );
		} else {
			Vp->Xr[ Rd ] = ~( ( RV_UINTR )0 );
		}
		break;
	case RV_OP_32_FUNCT3_RV64M_REMW:
		//
		// REMW and REMUW are RV64 instructions that provide the
		// corresponding signed and unsigned remainder operations respectively. Both REMW and REMUW
		// always sign-extend the 32-bit result to 64 bits, including on a divide by zero.
		//
		if( Vp->Xr[ Rs2 ] != 0 ) {
			Vp->Xr[ Rd ] = ( RV_INTR )( ( RV_INT32 )Vp->Xr[ Rs1 ] % ( RV_INT32 )Vp->Xr[ Rs2 ] );
		} else {
			Vp->Xr[ Rd ] = ( RV_INTR )( RV_INT32 )Vp->Xr[ Rs1 ];
		}
		break;
	case RV_OP_32_FUNCT3_RV64M_REMUW:
		if( Vp->Xr[ Rs2 ] != 0 ) {
			Vp->Xr[ Rd ] = ( RV_INTR )( RV_INT32 )( ( RV_UINT32 )Vp->Xr[ Rs1 ] % ( RV_UINT32 )Vp->Xr[ Rs2 ] );
		} else {
			Vp->Xr[ Rd ] = ( RV_INTR )( RV_INT32 )Vp->Xr[ Rs1 ];
		}
		break;
	default:
		RvpExceptionPush( Vp, RV_EXCEPTION_ILLEGAL_INSTRUCTION );
		return;
	}

	Vp->Pc += 4;
}

#endif

//
// RV64I opcodes.
//

static
VOID
RvpInstructionExecuteOpcodeOp32(
	_Inout_ RV_PROCESSOR* Vp,
	_In_    RV_UINT32     Instruction
	)
{
	RV_UINT32 Rd;
	RV_UINT32 Rs1;
	RV_UINT32 Rs2;
	RV_UINT32 Funct3;
	RV_UINT32 Funct7;
	RV_UINT32 Class;

	//
	// Decode R-type fields.
	//
	Rd     = ( ( Instruction >> RV_INST_R_RD_SHIFT ) & RV_INST_R_RD_MASK );
	Rs1    = ( ( Instruction >> RV_INST_R_RS1_SHIFT ) & RV_INST_R_RS1_MASK );
	Rs2    = ( ( Instruction >> RV_INST_R_RS2_SHIFT ) & RV_INST_R_RS2_MASK );
	Funct3 = ( ( Instruction >> RV_INST_R_FUNCT3_SHIFT ) & RV_INST_R_FUNCT3_MASK );
	Funct7 = ( ( Instruction >> RV_INST_R_FUNCT7_SHIFT ) & RV_INST_R_FUNCT7_MASK );

	//
	// Validate register indices.
	//
	if( Rd >= RV_COUNTOF( Vp->Xr ) || Rs1 >= RV_COUNTOF( Vp->Xr ) || Rs2 >= RV_COUNTOF( Vp->Xr ) ) {
		RvpExceptionPush( Vp, RV_EXCEPTION_ILLEGAL_INSTRUCTION );
		return;
	}

	//
	// Handle RV64M instructions.
	//
#if defined(RV_OPT_RV32M)
	if( Funct7 == RV_OP_32_FUNCT7_RV64M ) {
		RvpInstructionExecuteOpcodeOp32Rv64m( Vp, Instruction );
		return;
	}
#endif

	//
	// Build classification value from opcode+funct3+funct7.
	//
	Class = RV_INST_CLASSIFY_F3F7( RV_OPCODE_OP_32, Funct3, Funct7 );

	//
	// Handle OP-32 instructions.
	//
	switch( Class ) {
	case RV_INST_CLASSIFY_F3F7( RV_OPCODE_OP_32, RVP_OP_32_FUNCT3_ADDW, RV_OP_32_FUNCT7_ADDW ):
		//
		// ADDW and SUBW are RV64I-only instructions that are defined analogously to ADD and SUB
		// but operate on 32-bit values and produce signed 32-bit results. Overflows are ignored, and the low
		// 32-bits of the result is sign-extended to 64-bits and written to the destination register.
		//
		Vp->Xr[ Rd ] = ( RV_INTR )( RV_INT32 )( ( RV_UINT32 )Vp->Xr[ Rs1 ] + ( RV_UINT32 )Vp->Xr[ Rs2 ] );
		break;
	case RV_INST_CLASSIFY_F3F7( RV_OPCODE_OP_32, RVP_OP_32_FUNCT3_SUBW, RV_OP_32_FUNCT7_SUBW ):
		Vp->Xr[ Rd ] = ( RV_INTR )( RV_INT32 )( ( RV_UINT32 )Vp->Xr[ Rs1 ] - ( RV_UINT32 )Vp->Xr[ Rs2 ] );
		break;
	case RV_INST_CLASSIFY_F3F7( RV_OPCODE_OP_32, RVP_OP_32_FUNCT3_SLLW, RV_OP_32_FUNCT7_SLLW ):
		//
		// SLLW, SRLW, and SRAW are RV64I-only instructions that are analogously defined but operate
		// on 32-bit values and produce signed 32-bit results. The shift amount is given by rs2[4:0].
		//
		Vp->Xr[ Rd ] = ( RV_INTR )( RV_INT32 )( ( RV_UINT32 )Vp->Xr[ Rs1 ] << ( ( RV_UINT32 )Vp->Xr[ Rs2 ] & 0x1F ) );
		break;
	case RV_INST_CLASSIFY_F3F7( RV_OPCODE_OP_32, RVP_OP_32_FUNCT3_SRLW, RV_OP_32_FUNCT7_SRLW ):
		Vp->Xr[ Rd ] = ( RV_INTR )( RV_INT32 )( ( RV_UINT32 )Vp->Xr[ Rs1 ] >> ( ( RV_UINT32 )Vp->Xr[ Rs2 ] & 0x1F ) );
		break;
	case RV_INST_CLASSIFY_F3F7( RV_OPCODE_OP_32, RVP_OP_32_FUNCT3_SRAW, RV_OP_32_FUNCT7_SRAW ):
		Vp->Xr[ Rd ] = ( RV_INTR )( RV_INT32 )( ( RV_INT32 )Vp->Xr[ Rs1 ] >> ( ( RV_UINT32 )Vp->Xr[ Rs2 ] & 0x1F ) );
		break;
	default:
		RvpExceptionPush( Vp, RV_EXCEPTION_ILLEGAL_INSTRUCTION );
		return;
	}

	Vp->Pc += 4;
}

static
VOID
RvpInstructionExecuteOpcodeOpImm32(
	_Inout_ RV_PROCESSOR* Vp,
	_In_    RV_UINT32     Instruction
	)
{
	RV_UINT32 Opcode;
	RV_UINT32 Funct3;
	RV_UINT32 Rd;
	RV_UINT32 Rs1;
	RV_UINT32 ImmRaw;
	RV_UINT32 UImm32;
	RV_UINT32 Shamt;
	RV_UINT32 ShiftType;

	//
	// Decode I-type fields.
	//
	Opcode = ( ( Instruction >> RV_INST_I_OPCODE_SHIFT ) & RV_INST_I_OPCODE_MASK );
	Funct3 = ( ( Instruction >> RV_INST_I_FUNCT3_SHIFT ) & RV_INST_I_FUNCT3_MASK );
	Rd     = ( ( Instruction >> RV_INST_I_RD_SHIFT ) & RV_INST_I_RD_MASK );
	Rs1    = ( ( Instruction >> RV_INST_I_RS1_SHIFT ) & RV_INST_I_RS1_MASK );
	ImmRaw = ( ( Instruction >> RV_INST_I_IMM_11_0_SHIFT ) & RV_INST_I_IMM_11_0_MASK );
	UImm32 = RvpSignExtend32( Vp, ImmRaw, 11 );

	//
	// Validate register operands.
	//
	if( Rd >= RV_COUNTOF( Vp->Xr ) || Rs1 >= RV_COUNTOF( Vp->Xr ) ) {
		RvpExceptionPush( Vp, RV_EXCEPTION_ILLEGAL_INSTRUCTION );
		return;
	}

	//
	// The shift amount is encoded in the lower 6 bits of the I-immediate field for RV64I.
	// The shift type is encoded in the upper 6 bits of the I-immediate field for RV64I.
	//
	Shamt     = ( ImmRaw & ( ( 1ul << 6 ) - 1 ) );
	ShiftType = ( ( ImmRaw & ~( ( 1ul << 6 ) - 1 ) ) >> 6ul );

	//
	// Previously, SLLIW, SRLIW, and SRAIW with imm[5] != 0 were defined to cause illegal instruction
	// exceptions, whereas now they are marked as reserved. This is a backwards-compatible change.
	//
	Shamt &= ~( 1ul << 5 );

	//
	// Handle funct3 variants.
	//
	switch( Funct3 ) {
	case RV_OP_IMM_32_FUNCT3_ADDIW:
		//
		// ADDIW is an RV64I instruction that adds the sign-extended 12-bit immediate to register rs1
		// and produces the proper sign-extension of a 32-bit result in rd. Overflows are ignored and the
		// result is the low 32 bits of the result sign-extended to 64 bits. Note, ADDIW rd, rs1, 0 writes
		// the sign-extension of the lower 32 bits of register rs1 into register rd (assembler pseudoinstruction SEXT.W).
		//
		Vp->Xr[ Rd ] = ( RV_INTR )( ( RV_UINT32 )Vp->Xr[ Rs1 ] + ( RV_INT32 )UImm32 );
		break;
	case RV_OP_IMM_32_FUNCT3_SLLIW:
		//
		// SLLIW, SRLIW, and SRAIW are RV64I-only instructions that are analogously defined but operate
		// on 32-bit values and produce signed 32-bit results.
		//
		if( ShiftType != 0 ) {
			RvpExceptionPush( Vp, RV_EXCEPTION_ILLEGAL_INSTRUCTION );
			return;
		}
		Vp->Xr[ Rd ] = ( RV_INTR )( RV_INT32 )( ( RV_UINT32 )Vp->Xr[ Rs1 ] << Shamt );
		break;
	case RV_OP_IMM_32_FUNCT3_SRLIW_SRAIW:
		if( ShiftType == 0 ) {
			Vp->Xr[ Rd ] = ( RV_INTR )( RV_INT32 )( ( RV_UINT32 )Vp->Xr[ Rs1 ] >> Shamt );
		} else if( ShiftType == 0x10 ) {
			Vp->Xr[ Rd ] = ( RV_INTR )( RV_INT32 )( ( RV_INT32 )Vp->Xr[ Rs1 ] >> Shamt );
		} else {
			RvpExceptionPush( Vp, RV_EXCEPTION_ILLEGAL_INSTRUCTION );
			return;
		}
		break;
	default:
		RvpExceptionPush( Vp, RV_EXCEPTION_ILLEGAL_INSTRUCTION );
		return;
	}

	Vp->Pc += 4;
}
#endif

//
// Atomic memory operation instructions.
//

static
VOID
RvpInstructionExecuteOpcodeAmo(
	_Inout_ RV_PROCESSOR* Vp,
	_In_    RV_UINT32     Instruction
	)
{
	RvpExceptionPush( Vp, RV_EXCEPTION_ILLEGAL_INSTRUCTION );
	return;
}

//
// RV32F implementation.
// TODO: Convert some of these functions to use SSE if supported,
// instead of raw access to binary encoded IEEE754 floats.
//

#if (defined(RV_OPT_RV32F) || defined(RV_OPT_RV32D))
#if 0
//
// Not very performant when called for every FP operation,
// should be set once at the beginning of guest execution,
// and then only updated again once the guest changes to a new rounding mode!
//
static
VOID
RvpFpuHostSetRoundingMode(
	_Inout_ RV_PROCESSOR* Vp,
	_In_    RV_UINT32     RvRoundingMode
	)
{
	RV_UINT32 HostFpuCsr;

#if defined(RV_OPT_BUILD_SSE)
	HostFpuCsr = _mm_getcsr();
	switch( RvRoundingMode ) {
	case RV_ROUNDING_MODE_RNE:
	case RV_ROUNDING_MODE_RMM: /* ? */
		HostFpuCsr = ( ( HostFpuCsr & ~_MM_ROUND_MASK ) | _MM_ROUND_NEAREST );
		break;
	case RV_ROUNDING_MODE_RTZ:
		HostFpuCsr = ( ( HostFpuCsr & ~_MM_ROUND_MASK ) | _MM_ROUND_TOWARD_ZERO );
		break;
	case RV_ROUNDING_MODE_RDN:
		HostFpuCsr = ( ( HostFpuCsr & ~_MM_ROUND_MASK ) | _MM_ROUND_DOWN );
		break;
	case RV_ROUNDING_MODE_RUP:
		HostFpuCsr = ( ( HostFpuCsr & ~_MM_ROUND_MASK ) | _MM_ROUND_UP );
		break;
	default:
		break;
	}
	_mm_setcsr( HostFpuCsr )
#endif
}
#endif

//
// Single-precision floating-point SQRT approximation.
//
static
RV_FLOAT
RvpFpuSqrtF32(
	_Inout_ RV_PROCESSOR* Vp,
	_In_    RV_FLOAT      Value
	)
{
#if defined(RV_OPT_SQRT_F32)
	return RV_OPT_SQRT_F32( Value );
#elif defined(RV_OPT_BUILD_SSE)
	return _mm_cvtss_f32( _mm_sqrt_ss( _mm_set_ss( Value ) ) );
#elif defined(RV_OPT_BUILD_LIBC)
	return sqrtf( Value );
#elif defined(RV_OPT_BUILD_IEEE_754)
	RV_FLOAT32_RAW RawValue;

	//
	// ASQRT with two newton-raphson iterations from hacker's delight by Henry Warren.
	// Relative error ranges from 0.f to 0.00000023f.
	//
	RawValue     = ( RV_FLOAT32_RAW ){ .F32 = Value };
	RawValue.U32 = ( ( RawValue.U32 >> 1 ) + 0x1fbb3f80ul );               /* Initial guess. */
	RawValue.F32 = ( 0.5f * ( RawValue.F32 + ( Value / RawValue.F32 ) ) ); /* First N-R step. */
	RawValue.F32 = ( 0.5f * ( RawValue.F32 + ( Value / RawValue.F32 ) ) ); /* Second N-R step. */
	return RawValue.F32;
#else
	#error "Unsupported."
#endif
}

//
// The canonical NaN has a positive sign and all significand bits clear
// except the MSB, a.k.a. the quiet bit. For single-precision floating-point,
// this corresponds to the pattern 0x7fc00000.
//
RV_FORCEINLINE
static
RV_FLOAT
RvpFpuCanonicalNanF32(
	VOID
	)
{
	return ( RV_FLOAT32_RAW ){ .U32 = 0x7fc00000ul }.F32;
}

//
// Check if the given single-precision floating-point value is any kind of NaN value.
//
RV_FORCEINLINE
static
RV_BOOLEAN
RvpFpuIsNanF32(
	_In_ RV_FLOAT Value
	)
{
#if defined(RV_OPT_BUILD_IEEE_754)
	RV_FLOAT32_RAW RawValue;
	RV_UINT32      RawExponent;
	RV_UINT32      RawFraction;

	//
	// IEEE-754 single-precision floating-point values are encoded in binary
	// as 1 sign bit, 8 exponent bits, and 23 fraction bits, ordered respectively.
	//
	RawValue    = ( RV_FLOAT32_RAW ){ .F32 = Value };
	RawExponent = ( ( RawValue.U32 >> 23ul ) & ( ( 1ul << 8 ) - 1 ) );
	RawFraction = ( RawValue.U32 & ( ( 1ul << 23 ) - 1 ) );

	//
	// NaNs have all exponent bits set to 1, but with a fraction of anything other than all bits set to 0.
	//
	return ( ( RawExponent == ( ( 1ul << 8 ) - 1 ) ) && ( RawFraction != 0 ) );
#elif defined(RV_OPT_BUILD_LIBC)
	return isnan( Value );
#else
	//
	// Warning, may cause exception if value is a signaling NaN.
	//
	return ( Value != Value );
#endif
}

//
// Check if the given single-precision floating-point value is a signaling NaN value.
//
RV_FORCEINLINE
static
RV_BOOLEAN
RvpFpuIsSignalingNanF32(
	_In_ RV_FLOAT Value
	)
{
#if defined(RV_OPT_BUILD_IEEE_754)
	RV_FLOAT32_RAW RawValue;
	RV_UINT32      RawExponent;
	RV_UINT32      RawFraction;

	//
	// IEEE-754 single-precision floating-point values are encoded in binary
	// as 1 sign bit, 8 exponent bits, and 23 fraction bits, ordered respectively.
	//
	RawValue    = ( RV_FLOAT32_RAW ){ .F32 = Value };
	RawExponent = ( ( RawValue.U32 >> 23ul ) & ( ( 1ul << 8 ) - 1 ) );
	RawFraction = ( RawValue.U32 & ( ( 1ul << 23 ) - 1 ) );

	//
	// NaNs have all exponent bits set to 1, but with a fraction of anything other than all bits set to 0.
	// On the majority of modern processors, the first free bit of the fraction is used as the quiet bit, if this bit is set,
	// then the NaN is quiet/non-signaling. Note: a very small handful of ancient processors used unset to mean quiet.
	//
	if( ( RawExponent == ( ( 1ul << 8 ) - 1 ) ) && ( RawFraction != 0 ) ) {
		return ( ( RawFraction & ( 1ul << 22 ) ) == 0 );
	}

	return RV_FALSE;
#else
	#error "Unsupported."
#endif
}

//
// Check if the given single-precision floating-point value is an INF value.
//
RV_FORCEINLINE
static
RV_BOOLEAN
RvpFpuIsInfinityF32(
	_In_ RV_FLOAT Value
	)
{
#if defined(RV_OPT_BUILD_IEEE_754)
	RV_FLOAT32_RAW RawValue;
	RV_UINT32      RawExponent;
	RV_UINT32      RawFraction;

	//
	// IEEE-754 single-precision floating-point values are encoded in binary
	// as 1 sign bit, 8 exponent bits, and 23 fraction bits, ordered respectively.
	//
	RawValue    = ( RV_FLOAT32_RAW ){ .F32 = Value };
	RawExponent = ( ( RawValue.U32 >> 23ul ) & ( ( 1ul << 8 ) - 1 ) );
	RawFraction = ( RawValue.U32 & ( ( 1ul << 23 ) - 1 ) );

	//
	// Infinity is encoded as having all exponent bits set to 1, and all fraction bits set to 0,
	//
	if( ( RawExponent == ( ( 1ul << 8 ) - 1 ) ) && ( RawFraction == 0 ) ) {
		return RV_TRUE;
	}

	return RV_FALSE;
#else
	#error "Unsupported."
#endif
}


//
// If the input value is any kind of NaN, this function returns the canonical NaN value.
// If input value is not NaN, the input value is returned.
//
RV_FORCEINLINE
static
RV_FLOATR
RvpFpuCanonicalizeNan(
	_In_ RV_FLOATR Value
	)
{
#if defined(RV_OPT_RV32D)
	// TODO!
	// if( RvpFpuIsNanF64( Value ) ) {
	// 	return RvpFpuCanonicalNanF64();
	//}
	#error "Unsupported."
#elif defined(RV_OPT_RV32F)
	if( RvpFpuIsNanF32( Value ) ) {
		return RvpFpuCanonicalNanF32();
	}
#else
	#error "Unsupported."
#endif

	return Value;
}

//
// Check if the sign of a single-precision floating-point value is set.
//
RV_FORCEINLINE
static
RV_BOOLEAN
RvpFpuIsSignSetF32(
	_In_ RV_FLOAT Value
	)
{
#if defined(RV_OPT_BUILD_IEEE_754)
	return ( ( ( ( RV_FLOAT32_RAW ){ .F32 = Value } ).U32 & ( 1ull << 31 ) ) != 0 );
#elif defined(RV_OPT_BUILD_LIBC)
	return ( signbit( Value ) != 0 );
#else
	#error "Unsupported."
#endif
}

//
// Copy the sign of a single-precision floating-point value.
//
RV_FORCEINLINE
static
RV_FLOAT
RvpFpuCopySignF32(
	_In_ RV_FLOAT Value,
	_In_ RV_FLOAT Sign
	)
{
#if defined(RV_OPT_BUILD_IEEE_754)
	RV_FLOAT32_RAW RawValue;
	RV_FLOAT32_RAW RawSign;

	//
	// Get raw U32 representation of IEEE-754 input values.
	//
	RawValue = ( RV_FLOAT32_RAW ){ .F32 = Value };
	RawSign  = ( RV_FLOAT32_RAW ){ .F32 = Sign };

	//
	// Strip the original sign-bit of the destination input,
	// then copy the sign-bit of the source input.
	//
	RawValue.U32 &= ~( 1ull << 31 );
	RawValue.U32 |= ( RawSign.U32 & ( 1ull << 31 ) );
	return RawValue.F32;
#elif defined(RV_OPT_BUILD_LIBC)
	return copysignf( Value, Sign );
#else
	#error "Unsupported."
#endif
}

//
// Floating-point minimum-number and maximum-number instructions FMIN.S and FMAX.S write,
// respectively, the smaller or larger of rs1 and rs2 to rd. For the purposes of these instructions only,
// the value -0.0 is considered to be less than the value +0.0. If both inputs are NaNs, the result is
// the canonical NaN. If only one operand is a NaN, the result is the non-NaN operand. Signaling
// NaN inputs set the invalid operation exception flag, even when the result is not NaN.
//

static
RV_FLOAT
RvpFpuMinS(
	_Inout_ RV_PROCESSOR* Vp,
	_In_    RV_FLOAT      Lhs,
	_In_    RV_FLOAT      Rhs
	)
{
	//
	// If both inputs are NaNs, the result is the canonical NaN. 
	//
	if( RvpFpuIsNanF32( Lhs ) && RvpFpuIsNanF32( Rhs ) ) {
		if( RvpFpuIsSignalingNanF32( Lhs ) || RvpFpuIsSignalingNanF32( Rhs ) ) {
			Vp->CsrFcsr |= RV_FCSR_NV_FLAG;
		}
		return RvpFpuCanonicalNanF32();
	}

	//
	// If only one operand is a NaN, the result is the non-NaN operand.
	// Signaling NaN inputs set the invalid operation exception flag, even when the result is not NaN.
	//
	if( RvpFpuIsNanF32( Lhs ) ) {
		Vp->CsrFcsr |= ( RvpFpuIsSignalingNanF32( Lhs ) ? RV_FCSR_NV_FLAG : 0 );
		return Rhs;
	} else if( RvpFpuIsNanF32( Rhs ) ) {
		Vp->CsrFcsr |= ( RvpFpuIsSignalingNanF32( Rhs ) ? RV_FCSR_NV_FLAG : 0 );
		return Lhs;
	}

	return RV_MIN( Lhs, Rhs );
}

static
RV_FLOAT
RvpFpuMaxS(
	_Inout_ RV_PROCESSOR* Vp,
	_In_    RV_FLOAT      Lhs,
	_In_    RV_FLOAT      Rhs
	)
{
	//
	// If both inputs are NaNs, the result is the canonical NaN. 
	//
	if( RvpFpuIsNanF32( Lhs ) && RvpFpuIsNanF32( Rhs ) ) {
		Vp->CsrFcsr |= ( ( RvpFpuIsSignalingNanF32( Lhs ) || RvpFpuIsSignalingNanF32( Rhs ) )
						 ? RV_FCSR_NV_FLAG : 0 );
		return RvpFpuCanonicalNanF32();
	}

	//
	// If only one operand is a NaN, the result is the non-NaN operand.
	// Signaling NaN inputs set the invalid operation exception flag, even when the result is not NaN.
	//
	if( RvpFpuIsNanF32( Lhs ) ) {
		Vp->CsrFcsr |= ( RvpFpuIsSignalingNanF32( Lhs ) ? RV_FCSR_NV_FLAG : 0 );
		return Rhs;
	} else if( RvpFpuIsNanF32( Rhs ) ) {
		Vp->CsrFcsr |= ( RvpFpuIsSignalingNanF32( Rhs ) ? RV_FCSR_NV_FLAG : 0 );
		return Lhs;
	}

	return RV_MAX( Lhs, Rhs );
}

RV_FORCEINLINE
static
RV_FLOATR
RvpFpuApplyRounding(
	_Inout_ RV_PROCESSOR* Vp,
	_In_    RV_FLOATR     Value
	)
{
	//
	// TODO!
	//
	return Value;
}

RV_FORCEINLINE
static
RV_FLOATR
RvpFpuNormalizeHostResultF32(
	_Inout_ RV_PROCESSOR* Vp,
	_In_    RV_FLOAT      Input
	)
{
	RV_FLOATR Output;

	//
	// Convert F32 to register-sized float.
	//
	Output = ( RV_FLOATR )Input;

	//
	// Canonicalize NaN values.
	//
	Output = RvpFpuCanonicalizeNan( Input );

	//
	// Apply rounding.
	//
	// Output = RvpFpuApplyRounding( Vp, Output );

	return Output;
}

//
// Note that exactly one bit in rd will be set.
//  0: rs1 is -inf.
//  1: rs1 is a negative normal number.
//  2: rs1 is a negative subnormal number.
//  3: rs1 is -0.
//  4: rs1 is +0.
//  5: rs1 is a positive subnormal number.
//  6: rs1 is a positive normal number.
//  7: rs1 is +inf.
//  8: rs1 is a signaling NaN.
//  9: rs1 is a quiet NaN.
//
static
RV_UINTR
RvpFpuClassifyF32(
	_In_ RV_FLOAT Value
	)
{
#if defined(RV_OPT_BUILD_IEEE_754)
	RV_FLOAT32_RAW RawValue;
	RV_UINT32      RawExponent;
	RV_UINT32      RawSign;
	RV_UINT32      RawFraction;

	//
	// IEEE-754 single-precision floating-point values are encoded in binary
	// as 1 sign bit, 8 exponent bits, and 23 fraction bits, ordered respectively.
	//
	RawValue    = ( RV_FLOAT32_RAW ){ .F32 = Value };
	RawSign     = ( ( RawValue.U32 >> 31ul ) & 1 );
	RawExponent = ( ( RawValue.U32 >> 23ul ) & ( ( 1ul << 8 ) - 1 ) );
	RawFraction = ( RawValue.U32 & ( ( 1ul << 23 ) - 1 ) );

	//
	// Check for signaling NaN and quiet NaN.
	//
	if( RvpFpuIsSignalingNanF32( Value ) ) {
		return RV_FCLASS_RD_SIGNALING_NAN;
	} else if( RvpFpuIsNanF32( Value ) ) {
		return RV_FCLASS_RD_QUIET_NAN;
	}

	//
	// A number is denormalized if the exponent contains all 0s and the mantissa/fraction does not contain all 0s.
	// Zero is encoded as having an exponent of 0 and a fraction of 0.
	//
	if( RawExponent == 0 ) {
		if( RawFraction != 0 ) {
			return ( RawSign ? RV_FCLASS_RD_NEGATIVE_SUBNORMAL : RV_FCLASS_RD_POSITIVE_SUBNORMAL );
		}
		return ( RawSign ? RV_FCLASS_RD_NEGATIVE_ZERO : RV_FCLASS_RD_POSITIVE_ZERO );
	}

	//
	// Infinity is encoded as having all exponent bits set to 1, and all fraction bits set to 0,
	// whereas NaNs have all exponent bits set to 1, but with a fraction of anything other than all bits set to 0.
	// On the majority of modern processors, the first bit of the fraction is used as the quiet bit, if this bit is set,
	// then the NaN is quiet/non-signaling. Note: a very small handful of ancient processors used unset to mean quiet.
	//
	if( RawExponent == ( ( 1ul << 8 ) - 1 ) ) {
		if( RawFraction == 0 ) {
			return ( RawSign ? RV_FCLASS_RD_NEGATIVE_INF : RV_FCLASS_RD_POSITIVE_INF );
		} else {
			return ( ( RawExponent >> 22ul ) ? RV_FCLASS_RD_QUIET_NAN : RV_FCLASS_RD_SIGNALING_NAN );
		}
	}

	//
	// If none of the above checks of have passed, this must be a normal number.
	//
	return ( RawSign ? RV_FCLASS_RD_NEGATIVE_NORMAL : RV_FCLASS_RD_POSITIVE_NORMAL );
#elif defined(RV_OPT_BUILD_LIBC)
	switch( fpclassify( Value ) ) {
	case FP_NAN:
		return ( RvpFpuIsSignalingNanF32( Value ) ? RV_FCLASS_RD_SIGNALING_NAN : RV_FCLASS_RD_QUIET_NAN );
	case FP_INFINITE:
		return ( RvpFpuIsSignSetF32( Value ) ? RV_FCLASS_RD_NEGATIVE_INF : RV_FCLASS_RD_POSITIVE_INF );
	case FP_ZERO:
		return ( RvpFpuIsSignSetF32( Value ) ? RV_FCLASS_RD_NEGATIVE_ZERO : RV_FCLASS_RD_POSITIVE_ZERO );
	case FP_SUBNORMAL:
		return ( RvpFpuIsSignSetF32( Value ) ? RV_FCLASS_RD_NEGATIVE_SUBNORMAL : RV_FCLASS_RD_POSITIVE_SUBNORMAL );
	case FP_NORMAL:
		return ( RvpFpuIsSignSetF32( Value ) ? RV_FCLASS_RD_NEGATIVE_NORMAL : RV_FCLASS_RD_POSITIVE_NORMAL );
	}
	return ( RvpFpuIsSignSetF32( Value ) ? RV_FCLASS_RD_NEGATIVE_NORMAL : RV_FCLASS_RD_POSITIVE_NORMAL );
#else
	#error "Unsupported."
#endif
}

//
// TODO:
// A value of 111 in the instruction's rm field selects the dynamic rounding mode held in frm.
// If frm is set to an invalid value (101-111), any subsequent attempt to execute a floating-point
// operation with a dynamic rounding mode will raise an illegal instruction exception.
//

//
// RV32F FMA function implementations.
// Floating-point fused multiply-add instructions require a new standard instruction format. R4-type
// instructions specify three source registers (rs1, rs2, and rs3) and a destination register (rd). This
// format is only used by the floating-point fused multiply-add instructions.
// The R4-type format is the same as the R-type format, but funct5 becomes rs3.
// Note:
// The FNMSUB and FNMADD instructions are counterintuitively named,
// owing to the naming of the corresponding instructions in MIPS-IV.
//

static
VOID
RvpInstructionExecuteOpcodeMAdd(
	_Inout_ RV_PROCESSOR* Vp,
	_In_    RV_UINT32     Instruction
	)
{
	RV_UINT32 Opcode;
	RV_UINT32 Rm;
	RV_UINT32 Rd;
	RV_UINT32 Rs1;
	RV_UINT32 Rs2;
	RV_UINT32 Funct7;
	RV_UINT32 Fmt;
	RV_UINT32 Rs3;

	//
	// Decode R4-type fields.
	//
	Opcode = ( ( Instruction >> RV_INST_R_OPCODE_SHIFT ) & RV_INST_R_OPCODE_MASK );
	Rm     = ( ( Instruction >> RV_INST_R_FUNCT3_SHIFT ) & RV_INST_R_FUNCT3_MASK );
	Rd     = ( ( Instruction >> RV_INST_R_RD_SHIFT ) & RV_INST_R_RD_MASK );
	Rs1    = ( ( Instruction >> RV_INST_R_RS1_SHIFT ) & RV_INST_R_RS1_MASK );
	Rs2    = ( ( Instruction >> RV_INST_R_RS2_SHIFT ) & RV_INST_R_RS2_MASK );
	Funct7 = ( ( Instruction >> RV_INST_R_FUNCT7_SHIFT ) & RV_INST_R_FUNCT7_MASK );

	//
	// Floating point instructions split funct7 into funct5/rs3 and fmt.
	// The upper 5 bits being funct5/rs3, and the lower 2 bits being fmt.
	// fmt is a 2-bit field indicating operation size. Always 0b00 for single-precision.
	//
	Fmt = ( Funct7 & ( ( 1ul << 2 ) - 1 ) );
	Rs3 = ( ( Funct7 >> 2ul ) & ( ( 1ul << 5 ) - 1 ) );

	//
	// Only single-precision (RV32F) is currently supported.
	//
	if( Fmt != RV_FP_FMT_S ) {
		RvpExceptionPush( Vp, RV_EXCEPTION_ILLEGAL_INSTRUCTION );
		return;
	}

	//
	// Validate register indices.
	//
	if( Rd >= RV_COUNTOF( Vp->Fr ) || Rs1 >= RV_COUNTOF( Vp->Fr )
		|| Rs2 >= RV_COUNTOF( Vp->Fr ) || Rs3 >= RV_COUNTOF( Vp->Fr ) )
	{
		RvpExceptionPush( Vp, RV_EXCEPTION_ILLEGAL_INSTRUCTION );
		return;
	}

	//
	// The fused multiply-add instructions must set the invalid operation exception flag when the
	// multiplicands are inf and zero, even when the addend is a quiet NaN.
	// IEEE 754 specifies the result of (0 * INF) as NaN.
	//
	if( ( RvpFpuIsInfinityF32( Vp->Fr[ Rs1 ] ) && Vp->Fr[ Rs2 ] == 0.f )
		|| ( RvpFpuIsInfinityF32( Vp->Fr[ Rs2 ] ) && Vp->Fr[ Rs1 ] == 0.f ) )
	{
		Vp->CsrFcsr |= RV_FCSR_NV_FLAG;
	}

	//
	// FMADD.S multiplies the values in rs1 and rs2, adds the value in rs3
	// and writes the final result to rd. FMADD.S computes (rs1*rs2)+rs3.
	// TODO: Explicitly cast operands to single-precision RV_FLOAT?
	//
	Vp->Fr[ Rd ] = RvpFpuNormalizeHostResultF32( Vp, ( RV_FLOAT )( Vp->Fr[ Rs3 ] + ( Vp->Fr[ Rs1 ] * Vp->Fr[ Rs2 ] ) ) );
	Vp->Pc += 4;
}

static
VOID
RvpInstructionExecuteOpcodeMSub(
	_Inout_ RV_PROCESSOR* Vp,
	_In_    RV_UINT32     Instruction
	)
{
	RV_UINT32 Opcode;
	RV_UINT32 Rm;
	RV_UINT32 Rd;
	RV_UINT32 Rs1;
	RV_UINT32 Rs2;
	RV_UINT32 Funct7;
	RV_UINT32 Fmt;
	RV_UINT32 Rs3;

	//
	// Decode R4-type fields.
	//
	Opcode = ( ( Instruction >> RV_INST_R_OPCODE_SHIFT ) & RV_INST_R_OPCODE_MASK );
	Rm     = ( ( Instruction >> RV_INST_R_FUNCT3_SHIFT ) & RV_INST_R_FUNCT3_MASK );
	Rd     = ( ( Instruction >> RV_INST_R_RD_SHIFT ) & RV_INST_R_RD_MASK );
	Rs1    = ( ( Instruction >> RV_INST_R_RS1_SHIFT ) & RV_INST_R_RS1_MASK );
	Rs2    = ( ( Instruction >> RV_INST_R_RS2_SHIFT ) & RV_INST_R_RS2_MASK );
	Funct7 = ( ( Instruction >> RV_INST_R_FUNCT7_SHIFT ) & RV_INST_R_FUNCT7_MASK );

	//
	// Floating point instructions split funct7 into funct5/rs3 and fmt.
	// The upper 5 bits being funct5/rs3, and the lower 2 bits being fmt.
	// fmt is a 2-bit field indicating operation size. Always 0b00 for single-precision.
	//
	Fmt = ( Funct7 & ( ( 1ul << 2 ) - 1 ) );
	Rs3 = ( ( Funct7 >> 2ul ) & ( ( 1ul << 5 ) - 1 ) );

	//
	// Only single-precision (RV32F) is currently supported.
	//
	if( Fmt != RV_FP_FMT_S ) {
		RvpExceptionPush( Vp, RV_EXCEPTION_ILLEGAL_INSTRUCTION );
		return;
	}

	//
	// Validate register indices.
	//
	if( Rd >= RV_COUNTOF( Vp->Fr ) || Rs1 >= RV_COUNTOF( Vp->Fr )
		|| Rs2 >= RV_COUNTOF( Vp->Fr ) || Rs3 >= RV_COUNTOF( Vp->Fr ) )
	{
		RvpExceptionPush( Vp, RV_EXCEPTION_ILLEGAL_INSTRUCTION );
		return;
	}

	//
	// The fused multiply-add instructions must set the invalid operation exception flag when the
	// multiplicands are inf and zero, even when the addend is a quiet NaN.
	// IEEE 754 specifies the result of (0 * INF) as NaN.
	//
	if( ( RvpFpuIsInfinityF32( Vp->Fr[ Rs1 ] ) && Vp->Fr[ Rs2 ] == 0.f )
		|| ( RvpFpuIsInfinityF32( Vp->Fr[ Rs2 ] ) && Vp->Fr[ Rs1 ] == 0.f ) )
	{
		Vp->CsrFcsr |= RV_FCSR_NV_FLAG;
	}

	//
	// FMSUB.S multiplies the values in rs1 and rs2, subtracts the value in rs3,
	// and writes the final result to rd. FMSUB.S computes (rs1*rs2)-rs3.
	// TODO: Explicitly cast operands to single-precision RV_FLOAT?
	//
	Vp->Fr[ Rd ] = RvpFpuNormalizeHostResultF32( Vp, ( RV_FLOAT )( ( Vp->Fr[ Rs1 ] * Vp->Fr[ Rs2 ] ) - Vp->Fr[ Rs3 ] ) );
	Vp->Pc += 4;
}

static
VOID
RvpInstructionExecuteOpcodeNMSub(
	_Inout_ RV_PROCESSOR* Vp,
	_In_    RV_UINT32     Instruction
	)
{
	RV_UINT32 Opcode;
	RV_UINT32 Rm;
	RV_UINT32 Rd;
	RV_UINT32 Rs1;
	RV_UINT32 Rs2;
	RV_UINT32 Funct7;
	RV_UINT32 Fmt;
	RV_UINT32 Rs3;

	//
	// Decode R4-type fields.
	//
	Opcode = ( ( Instruction >> RV_INST_R_OPCODE_SHIFT ) & RV_INST_R_OPCODE_MASK );
	Rm     = ( ( Instruction >> RV_INST_R_FUNCT3_SHIFT ) & RV_INST_R_FUNCT3_MASK );
	Rd     = ( ( Instruction >> RV_INST_R_RD_SHIFT ) & RV_INST_R_RD_MASK );
	Rs1    = ( ( Instruction >> RV_INST_R_RS1_SHIFT ) & RV_INST_R_RS1_MASK );
	Rs2    = ( ( Instruction >> RV_INST_R_RS2_SHIFT ) & RV_INST_R_RS2_MASK );
	Funct7 = ( ( Instruction >> RV_INST_R_FUNCT7_SHIFT ) & RV_INST_R_FUNCT7_MASK );

	//
	// Floating point instructions split funct7 into funct5/rs3 and fmt.
	// The upper 5 bits being funct5/rs3, and the lower 2 bits being fmt.
	// fmt is a 2-bit field indicating operation size. Always 0b00 for single-precision.
	//
	Fmt = ( Funct7 & ( ( 1ul << 2 ) - 1 ) );
	Rs3 = ( ( Funct7 >> 2ul ) & ( ( 1ul << 5 ) - 1 ) );

	//
	// Only single-precision (RV32F) is currently supported.
	//
	if( Fmt != RV_FP_FMT_S ) {
		RvpExceptionPush( Vp, RV_EXCEPTION_ILLEGAL_INSTRUCTION );
		return;
	}

	//
	// Validate register indices.
	//
	if( Rd >= RV_COUNTOF( Vp->Fr ) || Rs1 >= RV_COUNTOF( Vp->Fr )
		|| Rs2 >= RV_COUNTOF( Vp->Fr ) || Rs3 >= RV_COUNTOF( Vp->Fr ) )
	{
		RvpExceptionPush( Vp, RV_EXCEPTION_ILLEGAL_INSTRUCTION );
		return;
	}

	//
	// The fused multiply-add instructions must set the invalid operation exception flag when the
	// multiplicands are inf and zero, even when the addend is a quiet NaN.
	// IEEE 754 specifies the result of (0 * INF) as NaN.
	//
	if( ( RvpFpuIsInfinityF32( Vp->Fr[ Rs1 ] ) && Vp->Fr[ Rs2 ] == 0.f )
		|| ( RvpFpuIsInfinityF32( Vp->Fr[ Rs2 ] ) && Vp->Fr[ Rs1 ] == 0.f ) )
	{
		Vp->CsrFcsr |= RV_FCSR_NV_FLAG;
	}

	//
	// FNMSUB.S multiplies the values in rs1 and rs2, negates the product,
	// adds the value in rs3, and writes the final result to rd.
	// FNMSUB.S computes -(rs1(rs2)+rs3.
	// TODO: Explicitly cast operands to single-precision RV_FLOAT?
	//
	Vp->Fr[ Rd ] = RvpFpuNormalizeHostResultF32( Vp, ( RV_FLOAT )( -( Vp->Fr[ Rs1 ] * Vp->Fr[ Rs2 ] ) + Vp->Fr[ Rs3 ] ) );
	Vp->Pc += 4;
}

static
VOID
RvpInstructionExecuteOpcodeNMAdd(
	_Inout_ RV_PROCESSOR* Vp,
	_In_    RV_UINT32     Instruction
	)
{
	RV_UINT32 Opcode;
	RV_UINT32 Rm;
	RV_UINT32 Rd;
	RV_UINT32 Rs1;
	RV_UINT32 Rs2;
	RV_UINT32 Funct7;
	RV_UINT32 Fmt;
	RV_UINT32 Rs3;

	//
	// Decode R4-type fields.
	//
	Opcode = ( ( Instruction >> RV_INST_R_OPCODE_SHIFT ) & RV_INST_R_OPCODE_MASK );
	Rm     = ( ( Instruction >> RV_INST_R_FUNCT3_SHIFT ) & RV_INST_R_FUNCT3_MASK );
	Rd     = ( ( Instruction >> RV_INST_R_RD_SHIFT ) & RV_INST_R_RD_MASK );
	Rs1    = ( ( Instruction >> RV_INST_R_RS1_SHIFT ) & RV_INST_R_RS1_MASK );
	Rs2    = ( ( Instruction >> RV_INST_R_RS2_SHIFT ) & RV_INST_R_RS2_MASK );
	Funct7 = ( ( Instruction >> RV_INST_R_FUNCT7_SHIFT ) & RV_INST_R_FUNCT7_MASK );

	//
	// Floating point instructions split funct7 into funct5/rs3 and fmt.
	// The upper 5 bits being funct5/rs3, and the lower 2 bits being fmt.
	// fmt is a 2-bit field indicating operation size. Always 0b00 for single-precision.
	//
	Fmt = ( Funct7 & ( ( 1ul << 2 ) - 1 ) );
	Rs3 = ( ( Funct7 >> 2ul ) & ( ( 1ul << 5 ) - 1 ) );

	//
	// Only single-precision (RV32F) is currently supported.
	//
	if( Fmt != RV_FP_FMT_S ) {
		RvpExceptionPush( Vp, RV_EXCEPTION_ILLEGAL_INSTRUCTION );
		return;
	}

	//
	// Validate register indices.
	//
	if( Rd >= RV_COUNTOF( Vp->Fr ) || Rs1 >= RV_COUNTOF( Vp->Fr )
		|| Rs2 >= RV_COUNTOF( Vp->Fr ) || Rs3 >= RV_COUNTOF( Vp->Fr ) )
	{
		RvpExceptionPush( Vp, RV_EXCEPTION_ILLEGAL_INSTRUCTION );
		return;
	}

	//
	// The fused multiply-add instructions must set the invalid operation exception flag when the
	// multiplicands are inf and zero, even when the addend is a quiet NaN.
	// IEEE 754 specifies the result of (0 * INF) as NaN.
	//
	if( ( RvpFpuIsInfinityF32( Vp->Fr[ Rs1 ] ) && Vp->Fr[ Rs2 ] == 0.f )
		|| ( RvpFpuIsInfinityF32( Vp->Fr[ Rs2 ] ) && Vp->Fr[ Rs1 ] == 0.f ) )
	{
		Vp->CsrFcsr |= RV_FCSR_NV_FLAG;
	}

	//
	// FNMADD.S multiplies the values in rs1 and rs2, negates the product,
	// subtracts the value in rs3, and writes the final result to rd
	// FNMADD.S computes -(rs1*rs2)-rs3.
	// TODO: Explicitly cast operands to single-precision RV_FLOAT?
	//
	Vp->Fr[ Rd ] = RvpFpuNormalizeHostResultF32( Vp, ( RV_FLOAT )( -( Vp->Fr[ Rs1 ] * Vp->Fr[ Rs2 ] ) - Vp->Fr[ Rs3 ] ) );
	Vp->Pc += 4;
}

//
// RV32F OP-FP/LOAD/STORE implementations.
// TODO: Improve classification for FP instructions, and clean
// up/simplify dispatch, this function has grown a bit too messy.
// 

static
VOID
RvpInstructionExecuteOpcodeOpFp(
	_Inout_ RV_PROCESSOR* Vp,
	_In_    RV_UINT32     Instruction
	)
{
	RV_UINT32  Opcode;
	RV_UINT32  Rd;
	RV_UINT32  Rs1;
	RV_UINT32  Rs2;
	RV_UINT32  Funct3;
	RV_UINT32  Rm;
	RV_UINT32  Funct7;
	RV_UINT32  Class;
	RV_BOOLEAN ShouldSignalNV;

	//
	// Decode R-type fields.
	//
	Opcode = ( ( Instruction >> RV_INST_R_OPCODE_SHIFT ) & RV_INST_R_OPCODE_MASK );
	Rd     = ( ( Instruction >> RV_INST_R_RD_SHIFT ) & RV_INST_R_RD_MASK );
	Rs1    = ( ( Instruction >> RV_INST_R_RS1_SHIFT ) & RV_INST_R_RS1_MASK );
	Rs2    = ( ( Instruction >> RV_INST_R_RS2_SHIFT ) & RV_INST_R_RS2_MASK );
	Funct3 = ( ( Instruction >> RV_INST_R_FUNCT3_SHIFT ) & RV_INST_R_FUNCT3_MASK );
	Funct7 = ( ( Instruction >> RV_INST_R_FUNCT7_SHIFT ) & RV_INST_R_FUNCT7_MASK );

	//
	// Some instructions use funct3 as the rounding-mode field.
	//
	Rm = Funct3;

	//
	// Classify Floating-point instruction
	//
	Class = RV_INST_CLASSIFY_F3F7( Opcode, 0, Funct7 );

	//
	// Validate register indices.
	// Note: Not all instructions use rs2, but use the rs2 field for their own purpose
	// in this case, the value of rs2 is always <= 1, so this check will still pass for them.
	// Some instructions also use the registers fields for X registers, not just F registers,
	// so for now, we just ensure that there are the same amount of both register types,
	// so that the register index check holds valid for both.
	//
	_Static_assert( RV_COUNTOF( Vp->Xr ) == RV_COUNTOF( Vp->Fr ), "Register count mismatch." );
	if( Rd >= RV_COUNTOF( Vp->Fr ) || Rs1 >= RV_COUNTOF( Vp->Fr ) || Rs2 >= RV_COUNTOF( Vp->Fr ) ) {
		RvpExceptionPush( Vp, RV_EXCEPTION_ILLEGAL_INSTRUCTION );
		return;
	}

	//
	// Floating-point arithmetic instructions with one or two source operands use the R-type format with the OP-FP major opcode.
	// 0000000 rs2   rs1 rm  rd 1010011 FADD.S
	// 0000100 rs2   rs1 rm  rd 1010011 FSUB.S
	// 0001000 rs2   rs1 rm  rd 1010011 FMUL.S
	// 0001100 rs2   rs1 rm  rd 1010011 FDIV.S
	// 0101100 00000 rs1 rm  rd 1010011 FSQRT.S
	// 0010000 rs2   rs1 000 rd 1010011 FSGNJ.S
	// 0010000 rs2   rs1 001 rd 1010011 FSGNJN.S
	// 0010000 rs2   rs1 010 rd 1010011 FSGNJX.S
	// 0010100 rs2   rs1 000 rd 1010011 FMIN.S
	// 0010100 rs2   rs1 001 rd 1010011 FMAX.S
	// 1100000 00000 rs1 rm  rd 1010011 FCVT.W.S
	// 1100000 00001 rs1 rm  rd 1010011 FCVT.WU.S
	// 1100000 00011 rs1 rm  rd 1010011 FCVT.LU.S
	// 1110000 00000 rs1 000 rd 1010011 FMV.X.W
	// 1010000 rs2   rs1 010 rd 1010011 FEQ.S
	// 1010000 rs2   rs1 001 rd 1010011 FLT.S
	// 1010000 rs2   rs1 000 rd 1010011 FLE.S
	// 1110000 00000 rs1 001 rd 1010011 FCLASS.S
	// 1101000 00000 rs1 rm  rd 1010011 FCVT.S.W
	// 1101000 00001 rs1 rm  rd 1010011 FCVT.S.WU
	// 1111000 00000 rs1 000 rd 1010011 FMV.W.X
	// 1101000 00010 rs1 rm  rd 1010011 FCVT.S.L
	// 1101000 00011 rs1 rm  rd 1010011 FCVT.S.LU
	//
	switch( Class ) {
	case RV_INST_CLASSIFY_F3F7( RV_OPCODE_OP_FP, 0, RV_OP_FP_FUNCT7_FADD_S ):
		Vp->Fr[ Rd ] = RvpFpuNormalizeHostResultF32( Vp, ( ( RV_FLOAT )Vp->Fr[ Rs1 ] + ( RV_FLOAT )Vp->Fr[ Rs2 ] ) );
		break;
	case RV_INST_CLASSIFY_F3F7( RV_OPCODE_OP_FP, 0, RV_OP_FP_FUNCT7_FSUB_S ):
		Vp->Fr[ Rd ] = RvpFpuNormalizeHostResultF32( Vp, ( ( RV_FLOAT )Vp->Fr[ Rs1 ] - ( RV_FLOAT )Vp->Fr[ Rs2 ] ) );
		break;
	case RV_INST_CLASSIFY_F3F7( RV_OPCODE_OP_FP, 0, RV_OP_FP_FUNCT7_FMUL_S ):
		Vp->Fr[ Rd ] = RvpFpuNormalizeHostResultF32( Vp, ( ( RV_FLOAT )Vp->Fr[ Rs1 ] * ( RV_FLOAT )Vp->Fr[ Rs2 ] ) );
		break;
	case RV_INST_CLASSIFY_F3F7( RV_OPCODE_OP_FP, 0, RV_OP_FP_FUNCT7_FDIV_S ):
		Vp->Fr[ Rd ] = RvpFpuNormalizeHostResultF32( Vp, ( ( RV_FLOAT )Vp->Fr[ Rs1 ] / ( RV_FLOAT )Vp->Fr[ Rs2 ] ) );
		break;
	case RV_INST_CLASSIFY_F3F7( RV_OPCODE_OP_FP, 0, RV_OP_FP_FUNCT7_FSQRT_S ):
		if( Rs2 != RV_OP_FP_RS2_FSQRT_S ) {
			RvpExceptionPush( Vp, RV_EXCEPTION_ILLEGAL_INSTRUCTION );
			return;
		}
		Vp->Fr[ Rd ] = RvpFpuNormalizeHostResultF32( Vp, RvpFpuSqrtF32( Vp, ( RV_FLOAT )Vp->Fr[ Rs1 ] ) );
		break;
	case RV_INST_CLASSIFY_F3F7( RV_OPCODE_OP_FP, 0, RV_OP_FP_FUNCT7_FMIN_FMAX_S ):
		//
		// Floating-point minimum-number and maximum-number instruction FMIN.S and FMAX.S write,
		// respectively, the smaller or larger of rs1 and rs2 to rd. For the purposes of these instructions only,
		// the value -0.0 is considered to be less than the value +0.0. If both inputs are NaNs, the result is
		// the canonical NaN. If only one operand is a NaN, the result is the non-NaN operand. Signaling
		// NaN inputs set the invalid operation exception flag, even when the result is not NaN.
		// The funct3 field indicates if this is a min or max instruction variant.
		//
		if( Funct3 == RV_OP_FP_FUNCT3_FMIN_S ) {
			Vp->Fr[ Rd ] = ( RV_FLOATR )RvpFpuMinS( Vp, ( RV_FLOAT )Vp->Fr[ Rs1 ], ( RV_FLOAT )Vp->Fr[ Rs2 ] );
		} else if( Funct3 == RV_OP_FP_FUNCT3_FMAX_S ) {
			Vp->Fr[ Rd ] = ( RV_FLOATR )RvpFpuMaxS( Vp, ( RV_FLOAT )Vp->Fr[ Rs1 ], ( RV_FLOAT )Vp->Fr[ Rs2 ] );
		} else {
			RvpExceptionPush( Vp, RV_EXCEPTION_ILLEGAL_INSTRUCTION );
			return;
		}
		break;
	case RV_INST_CLASSIFY_F3F7( RV_OPCODE_OP_FP, 0, RV_OP_FP_FUNCT7_FCVT_W_S_WU_S ):
		//
		// (S -> W/L).
		// FCVT.W.S converts register rs1 to a signed 32-bit integer in integer register rd.
		// FCVT.WU.S converts to unsigned integer 32-bit integer.
		// For XLEN> 32, FCVT.W[U].S sign-extends the 32-bit result to the destination register width.
		// 
		// TODO:
		// If the rounded result is not representable in the destination format, it
		// is clipped to the nearest value and the invalid flag is set. Table 11.4 gives the range of valid inputs
		// for FCVT.int.S and the behavior for invalid inputs.
		// All floating-point to integer and integer to floating-point conversion instructions round according
		// to the rm field.
		//
		switch( Rs2 ) {
		case RV_OP_FP_RS2_FCVT_W_S:
			Vp->Xr[ Rd ] = ( RV_INTR )( RV_INT32 )RvpFpuApplyRounding( Vp, Vp->Fr[ Rs1 ] );
			break;
		case RV_OP_FP_RS2_FCVT_WU_S:
			Vp->Xr[ Rd ] = ( RV_INTR )( RV_INT32 )( RV_UINT32 )RvpFpuApplyRounding( Vp, Vp->Fr[ Rs1 ] );
			break;
#if defined(RV_OPT_RV64I)
		case RV_OP_FP_RS2_FCVT_L_S:
			Vp->Xr[ Rd ] = ( RV_INTR )RvpFpuApplyRounding( Vp, Vp->Fr[ Rs1 ] );
			break;
		case RV_OP_FP_RS2_FCVT_LU_S:
			Vp->Xr[ Rd ] = ( RV_UINTR )RvpFpuApplyRounding( Vp, Vp->Fr[ Rs1 ] );
			break;
#endif
		default:
			RvpExceptionPush( Vp, RV_EXCEPTION_ILLEGAL_INSTRUCTION );
			return;
		}
		break;
	case RV_INST_CLASSIFY_F3F7( RV_OPCODE_OP_FP, 0, RV_OP_FP_FUNCT7_FCVT_S_W_S_WU ):
		//
		// (W/L -> S).
		// FCVT.S.W or FCVT.S.L converts a 32-bit or 64-bit signed integer, respectively,
		// in integer register rs1 into a floating-point number in floating-point register rd.
		//
		switch( Rs2 ) {
		case RV_OP_FP_RS2_FCVT_S_W:
			Vp->Fr[ Rd ] = RvpFpuApplyRounding( Vp, ( RV_FLOATR )( RV_INT32 )Vp->Xr[ Rs1 ] );
			break;
		case RV_OP_FP_RS2_FCVT_S_WU:
			Vp->Fr[ Rd ] = RvpFpuApplyRounding( Vp, ( RV_FLOATR )( RV_UINT32 )Vp->Xr[ Rs1 ] );
			break;
#if defined(RV_OPT_RV64I)
		case RV_OP_FP_RS2_FCVT_S_L:
			Vp->Fr[ Rd ] = RvpFpuApplyRounding( Vp, ( RV_FLOATR )( RV_INT64 )Vp->Xr[ Rs1 ] );
			break;
		case RV_OP_FP_RS2_FCVT_S_LU:
			Vp->Fr[ Rd ] = RvpFpuApplyRounding( Vp, ( RV_FLOATR )Vp->Xr[ Rs1 ] );
			break;
#endif
		default:
			RvpExceptionPush( Vp, RV_EXCEPTION_ILLEGAL_INSTRUCTION );
			return;
		}
		break;
	case RV_INST_CLASSIFY_F3F7( RV_OPCODE_OP_FP, 0, RV_OP_FP_FUNCT7_FSGNJ_S ):
		//
		// Floating-point to floating-point sign-injection instructions.
		// FSGNJ.S, FSGNJN.S, and FSGNJX.S produce a result that takes all bits except the sign bit from rs1.
		// Sign-injection instructions do not set floating-point exception flags, nor do they canonicalize NaNs.
		//
		switch( Funct3 ) {
		case RV_OP_FP_FUNCT3_FSGNJ_S:
			//
			// For FSGNJ, the result's sign bit is rs2's sign bit.
			//
			Vp->Fr[ Rd ] = ( RV_FLOATR )RvpFpuCopySignF32( Vp->Fr[ Rs1 ], Vp->Fr[ Rs2 ] );
			break;
		case RV_OP_FP_FUNCT3_FSGNJN_S:
			//
			// For FSGNJN, the result's sign bit is the opposite of rs2's sign bit.
			//
			Vp->Fr[ Rd ] = ( RV_FLOATR )RvpFpuCopySignF32(
				Vp->Fr[ Rs1 ],
				( RvpFpuIsSignSetF32( Vp->Fr[ Rs2 ] ? 1.f : -1.f ) )
			);
			break;
		case RV_OP_FP_FUNCT3_FSGNJX_S:
			//
			// For FSGNJX, the sign bit is the XOR of the sign bits of rs1 and rs2.
			//
			Vp->Fr[ Rd ] = ( RV_FLOATR )RvpFpuCopySignF32(
				Vp->Fr[ Rs1 ],
				( ( RvpFpuIsSignSetF32( Vp->Fr[ Rs1 ] ) ^ RvpFpuIsSignSetF32( Vp->Fr[ Rs2 ] ) ) ? -1.f : 1.f )
			);
			break;
		default:
			RvpExceptionPush( Vp, RV_EXCEPTION_ILLEGAL_INSTRUCTION );
			return;
		}
		break;
	case RV_INST_CLASSIFY_F3F7( RV_OPCODE_OP_FP, 0, RV_OP_FP_FUNCT7_FMV_X_W_FCLASS_S ):
		if( Funct3 == RV_OP_FP_FUNCT3_FMV_X_W ) {
			//
			// Instructions are provided to move bit patterns between the floating-point and integer registers.
			// FMV.X.W moves the single-precision value in floating-point register rs1 represented in IEEE 754-
			// 2008 encoding to the lower 32 bits of integer register rd. The bits are not modified in the transfer,
			// and in particular, the payloads of non-canonical NaNs are preserved. For RV64, the higher 32 bits
			// of the destination register are filled with copies of the floating-point number's sign bit.
			//
			Vp->Xr[ Rd ] = 0;
#if defined(RV_OPT_RV64I)
			Vp->Xr[ Rd ] |= ( RvpFpuIsSignSetF32( Vp->Fr[ Rs1 ] ) ? ( 0xFFFFFFFFull << 32ull ) : 0 );
#endif
			Vp->Xr[ Rd ] |= ( RV_FLOAT32_RAW ) { .F32 = ( RV_FLOAT )Vp->Fr[ Rs1 ] }.U32;
		} else if( Funct3 == RV_OP_FP_FUNCT3_FCLASS_S ) {
			//
			// The FCLASS.S instruction examines the value in floating-point register rs1 and writes to
			// integer register rd a 10-bit mask that indicates the class of the floating-point number.
			// The corresponding bit in rd will be set if the property is true and clear otherwise.
			// All other bits in rd are cleared. Note that exactly one bit in rd will be set.
			// FCLASS.S does not set the floating-point exception flags.
			//
			Vp->Xr[ Rd ] = RvpFpuClassifyF32( ( RV_FLOAT )Vp->Fr[ Rs1 ] );
		} else {
			RvpExceptionPush( Vp, RV_EXCEPTION_ILLEGAL_INSTRUCTION );
			return;
		}
		break;
	case RV_INST_CLASSIFY_F3F7( RV_OPCODE_OP_FP, 0, RV_OP_FP_FUNCT7_FMV_W_X ):
		//
		// FMV.W.X moves the single-precision value encoded in IEEE 754-2008 standard encoding from the
		// lower 32 bits of integer register rs1 to the floating-point register rd. The bits are not modified in
		// the transfer, and in particular, the payloads of non-canonical NaNs are preserved.
		//
		Vp->Fr[ Rd ] = ( RV_FLOATR )( ( RV_FLOAT32_RAW ){ .U32 = ( RV_UINT32 )Vp->Xr[ Rs1 ] }.F32 );
		break;
	case RV_INST_CLASSIFY_F3F7( RV_OPCODE_OP_FP, 0, RV_OP_FP_FUNCT7_FEQ_S_FLT_S_FLE_S ):
		//
		// Floating-point compare instructions (FEQ.S, FLT.S, FLE.S) perform the specified comparison between
		// floating-point registers (rs1 = rs2, rs1 < rs2, rs1 <= rs2) writing 1 to the integer register rd
		// if the condition holds, and 0 otherwise.
		//
		switch( Funct3 ) {
		case RV_OP_FP_FUNCT3_FEQ_S:
			//
			// FEQ.S performs a quiet comparison: it only sets the invalid operation
			// exception flag if either input is a signaling NaN.
			//
			ShouldSignalNV = (
				RvpFpuIsSignalingNanF32( ( RV_FLOAT )Vp->Fr[ Rs1 ] )
				|| RvpFpuIsSignalingNanF32( ( RV_FLOAT )Vp->Fr[ Rs2 ] )
			);
			Vp->CsrFcsr |= ( ShouldSignalNV ? RV_FCSR_NV_FLAG : 0 );

			//
			// For all three instructions, the result is 0 if either operand is NaN.
			//
			if( RvpFpuIsNanF32( ( RV_FLOAT )Vp->Fr[ Rs1 ] ) || RvpFpuIsNanF32( ( RV_FLOAT )Vp->Fr[ Rs2 ] ) ) {
				Vp->Xr[ Rd ] = 0;
			} else {
				Vp->Xr[ Rd ] = ( ( ( RV_FLOAT )Vp->Fr[ Rs1 ] == ( RV_FLOAT )Vp->Fr[ Rs2 ] ) ? 1 : 0 );
			}
			break;
		case RV_OP_FP_FUNCT3_FLT_S:
			//
			// FLT.S and FLE.S perform what the IEEE 754-2008 standard refers to as signaling comparisons:
			// that is, they set the invalid operation exception flag if either input is NaN.
			//
			ShouldSignalNV = (
				RvpFpuIsNanF32( ( RV_FLOAT )Vp->Fr[ Rs1 ] )
				|| RvpFpuIsNanF32( ( RV_FLOAT )Vp->Fr[ Rs2 ] )
			);
			Vp->CsrFcsr |= ( ShouldSignalNV ? RV_FCSR_NV_FLAG : 0 );

			//
			// For all three instructions, the result is 0 if either operand is NaN.
			//
			if( ShouldSignalNV ) {
				Vp->Xr[ Rd ] = 0;
			} else {
				Vp->Xr[ Rd ] = ( ( ( RV_FLOAT )Vp->Fr[ Rs1 ] < ( RV_FLOAT )Vp->Fr[ Rs2 ] ) ? 1 : 0 );
			}
			break;
		case RV_OP_FP_FUNCT3_FLE_S:
			//
			// FLT.S and FLE.S perform what the IEEE 754-2008 standard refers to as signaling comparisons:
			// that is, they set the invalid operation exception flag if either input is NaN.
			//
			ShouldSignalNV = (
				RvpFpuIsNanF32( ( RV_FLOAT )Vp->Fr[ Rs1 ] )
				|| RvpFpuIsNanF32( ( RV_FLOAT )Vp->Fr[ Rs2 ] )
			);
			Vp->CsrFcsr |= ( ShouldSignalNV ? RV_FCSR_NV_FLAG : 0 );

			//
			// For all three instructions, the result is 0 if either operand is NaN.
			//
			if( ShouldSignalNV ) {
				Vp->Xr[ Rd ] = 0;
			} else {
				Vp->Xr[ Rd ] = ( ( ( RV_FLOAT )Vp->Fr[ Rs1 ] <= ( RV_FLOAT )Vp->Fr[ Rs2 ] ) ? 1 : 0 );
			}
			break;
		default:
			RvpExceptionPush( Vp, RV_EXCEPTION_ILLEGAL_INSTRUCTION );
			return;
		}
		break;
	default:
		RvpExceptionPush( Vp, RV_EXCEPTION_ILLEGAL_INSTRUCTION );
		return;
	}

	Vp->Pc += 4;
}

static
VOID
RvpInstructionExecuteOpcodeStoreFp(
	_Inout_ RV_PROCESSOR* Vp,
	_In_    RV_UINT32     Instruction
	)
{
	RV_UINT32    Width;
	RV_UINT32    UOffset32;
	RV_UINT32    BaseXr;
	RV_UINT32    SrcFr;
	RV_UINTR     Address;
	RV_MEM_VALUE Value;

	//
	// Decode S-type instruction fields.
	//
	Width      = ( ( Instruction >> RV_INST_S_FUNCT3_SHIFT ) & RV_INST_S_FUNCT3_MASK );
	BaseXr     = ( ( Instruction >> RV_INST_S_RS1_SHIFT ) & RV_INST_S_RS1_MASK );
	SrcFr      = ( ( Instruction >> RV_INST_S_RS2_SHIFT ) & RV_INST_S_RS2_MASK );
	UOffset32  = ( ( Instruction >> RV_INST_S_IMM_4_0_SHIFT ) & RV_INST_S_IMM_4_0_MASK );
	UOffset32 |= ( ( ( Instruction >> RV_INST_S_IMM_11_5_SHIFT ) & RV_INST_S_IMM_11_5_MASK ) << 5ul );
	UOffset32  = RvpSignExtend32( Vp, UOffset32, 11 );

	//
	// Validate register indices.
	//
	if( BaseXr >= RV_COUNTOF( Vp->Xr ) || SrcFr >= RV_COUNTOF( Vp->Fr ) ) {
		RvpExceptionPush( Vp, RV_EXCEPTION_ILLEGAL_INSTRUCTION );
		return;
	}

	//
	// Floating-point loads and stores use the same base+offset addressing mode as the
	// integer base ISA, with a base address in register rs1 and a 12-bit signed byte offset.
	// FSW stores a single-precision value from Floating-point register rs2 to memory.
	//
	Address = ( Vp->Xr[ BaseXr ] + ( RV_INTR )( RV_INT32 )UOffset32 );

	//
	// Store Floating-point value of given width to memory.
	//
	switch( Width ) {
	case RV_STORE_FUNCT3_SW:
		//
		// FSW stores a single-precision value from Floating-point register rs2 to memory.
		//
		Value = ( RV_MEM_VALUE ){ .AsFloat = ( RV_FLOAT )Vp->Fr[ SrcFr ] };
		if( RvpMmuGuestStore( Vp, Address, &Value.AsU32, sizeof( Value.AsU32 ), RV_MMU_MIN_PAGESIZE, RV_TRUE ) == RV_FALSE ) {
			return;
		}
		break;
#if defined(RV_OPT_RV32D)
	case RV_STORE_FUNCT3_SD:
		//
		// FSD stores a double-precision value from the Floating-point registers to memory.
		//
		Value = ( RV_MEM_VALUE ){ .AsDouble = ( RV_DOUBLE )Vp->Fr[ SrcFr ] };
		if( RvpMmuGuestStore( Vp, Address, &Value.AsU64, sizeof( Value.AsU64 ), RV_MMU_MIN_PAGESIZE, RV_TRUE ) == RV_FALSE ) {
			return;
		}
		break;
#endif
	default:
		RvpExceptionPush( Vp, RV_EXCEPTION_ILLEGAL_INSTRUCTION );
		return;
	}

	Vp->Pc += 4;
}

static
VOID
RvpInstructionExecuteOpcodeLoadFp(
	_Inout_ RV_PROCESSOR* Vp,
	_In_    RV_UINT32     Instruction
	)
{
	RV_UINT32    Width;
	RV_UINT32    RdFr;
	RV_UINT32    BaseXr;
	RV_UINT32    UOffset32;
	RV_UINTR     Address;
	RV_MEM_VALUE Value;

	//
	// Decode I-type instruction fields.
	//
	Width     = ( ( Instruction >> RV_INST_I_FUNCT3_SHIFT ) & RV_INST_I_FUNCT3_MASK );
	RdFr      = ( ( Instruction >> RV_INST_I_RD_SHIFT ) & RV_INST_I_RD_MASK );
	BaseXr    = ( ( Instruction >> RV_INST_I_RS1_SHIFT ) & RV_INST_I_RS1_MASK );
	UOffset32 = ( ( Instruction >> RV_INST_I_IMM_11_0_SHIFT ) & RV_INST_I_IMM_11_0_MASK );
	UOffset32 = RvpSignExtend32( Vp, UOffset32, 11 );

	//
	// Validate register indices.
	//
	if( RdFr >= RV_COUNTOF( Vp->Xr ) || BaseXr >= RV_COUNTOF( Vp->Fr ) ) {
		RvpExceptionPush( Vp, RV_EXCEPTION_ILLEGAL_INSTRUCTION );
		return;
	}

	//
	// The effective address is obtained by adding register rs1 to the sign-extended 12-bit offset.
	//
	Address = ( Vp->Xr[ BaseXr ] + ( RV_INTR )( RV_INT32 )UOffset32 );

	//
	// Load Floating-point value of given width from memory.
	//
	switch( Width ) {
	case RV_STORE_FUNCT3_SW:
		//
		// The FLW instruction loads a single-precision Floating-point value from memory into Floating-point register rd.
		//
		Value = ( RV_MEM_VALUE ){ 0 };
		if( RvpMmuGuestLoad( Vp, Address, &Value.AsU32, sizeof( Value.AsU32 ), RV_MMU_MIN_PAGESIZE, RV_TRUE ) ) {
			return;
		}
		Vp->Fr[ RdFr ] = ( RV_FLOATR )Value.AsFloat;
		break;
#if defined(RV_OPT_RV32D)
	case RV_STORE_FUNCT3_SD:
		//
		// The FLD instruction loads a double-precision Floating-point value from memory into Floating-point register rd.
		//
		Value = ( RV_MEM_VALUE ){ 0 };
		if( RvpMmuGuestLoad( Vp, Address, &Value.AsU64, sizeof( Value.AsU64 ), RV_MMU_MIN_PAGESIZE, RV_TRUE ) ) {
			return;
		}
		Vp->Fr[ RdFr ] = ( RV_FLOATR )Value.AsDouble;
		break;
#endif
	default:
		RvpExceptionPush( Vp, RV_EXCEPTION_ILLEGAL_INSTRUCTION );
		return;
	}

	Vp->Pc += 4;
}
#endif

//
// Base instruction execution dispatcher.
//

static
VOID
RvpInstructionExecute(
	_Inout_ RV_PROCESSOR* Vp,
	_In_    RV_UINT32     Instruction
	)
{
	RV_UINT8 Opcode;

	//
	// Attempt to read common opcode field, and dispatch accordingly.
	//
	Opcode = ( ( Instruction >> RV_INST_R_OPCODE_SHIFT ) & RV_INST_R_OPCODE_MASK );
	switch( Opcode ) {
	case RV_OPCODE_LOAD:
		RvpInstructionExecuteOpcodeLoad( Vp, Instruction );
		break;
	case RV_OPCODE_STORE:
		RvpInstructionExecuteOpcodeStore( Vp, Instruction );
		break;
	case RV_OPCODE_MISC_MEM:
		RvpInstructionExecuteOpcodeMiscMem( Vp, Instruction );
		break;
	case RV_OPCODE_OP:
		RvpInstructionExecuteOpcodeOp( Vp, Instruction );
		break;
	case RV_OPCODE_OP_IMM:
		RvpInstructionExecuteOpcodeOpImm( Vp, Instruction );
		break;
	case RV_OPCODE_LUI:
		RvpInstructionExecuteOpcodeLui( Vp, Instruction );
		break;
	case RV_OPCODE_AUIPC:
		RvpInstructionExecuteOpcodeAuipc( Vp, Instruction );
		break;
	case RV_OPCODE_JALR:
		RvpInstructionExecuteOpcodeJALR( Vp, Instruction );
		break;
	case RV_OPCODE_JAL:
		RvpInstructionExecuteOpcodeJAL( Vp, Instruction );
		break;
	case RV_OPCODE_BRANCH:
		RvpInstructionExecuteOpcodeBranch( Vp, Instruction );
		break;
	case RV_OPCODE_SYSTEM:
		RvpInstructionExecuteOpcodeSystem( Vp, Instruction );
		break;
	case RV_OPCODE_AMO:
		RvpInstructionExecuteOpcodeAmo( Vp, Instruction );
		break;
#if defined(RV_OPT_RV64I)
	case RV_OPCODE_OP_IMM32:
		RvpInstructionExecuteOpcodeOpImm32( Vp, Instruction );
		break;
	case RV_OPCODE_OP_32:
		RvpInstructionExecuteOpcodeOp32( Vp, Instruction );
		break;
#endif
#if (defined(RV_OPT_RV32F) || defined(RV_OPT_RV32D))
	case RV_OPCODE_MADD:
		RvpInstructionExecuteOpcodeMAdd( Vp, Instruction );
		break;
	case RV_OPCODE_MSUB:
		RvpInstructionExecuteOpcodeMSub( Vp, Instruction );
		break;
	case RV_OPCODE_NMSUB:
		RvpInstructionExecuteOpcodeNMSub( Vp, Instruction );
		break;
	case RV_OPCODE_NMADD:
		RvpInstructionExecuteOpcodeNMAdd( Vp, Instruction );
		break;
	case RV_OPCODE_OP_FP:
		RvpInstructionExecuteOpcodeOpFp( Vp, Instruction );
		break;
	case RV_OPCODE_LOAD_FP:
		RvpInstructionExecuteOpcodeLoadFp( Vp, Instruction );
		break;
	case RV_OPCODE_STORE_FP:
		RvpInstructionExecuteOpcodeStoreFp( Vp, Instruction );
		break;
#endif
	default:
		RvpExceptionPush( Vp, RV_EXCEPTION_ILLEGAL_INSTRUCTION );
		break;
	}
}

//
// General processor tick execution functions (update values/timers, fetch and execute a instruction, etc).
//

VOID
RvpTickExecute(
	_Inout_ RV_PROCESSOR* Vp
	)
{
	RV_UINT32 Instruction;

	//
	// Reset per-tick fields.
	//
	Vp->ExceptionPending = 0;
	Vp->ECallPending     = 0;
	Vp->EBreakPending    = 0;
	Vp->ExceptionIndex   = 0;

	//
	// Instructions addresses must be 32-bit aligned.
	// Typically this should only be done on branch instructions, but this will be left here just in case.
	//
	if( ( Vp->Pc & ( 4 - 1 ) ) != 0 ) {
		RvpExceptionPush( Vp, RV_EXCEPTION_INSTRUCTION_ADDRESS_MISALIGNED );
		return;
	}

	//
	// Fetch instruction word from PC.
	//
	if( RvpFetchInstructionWord( Vp, Vp->Pc, RV_TRUE, &Instruction ) == RV_FALSE ) {
		return;
	}

	//
	// Execute fetched instruction, this function will handle advancing the PC.
	//
	RvpInstructionExecute( Vp, Instruction );

	//
	// Reset zero register.
	//
	Vp->Xr[ 0 ] = 0;

	//
	// Increase counters.
	//
	Vp->CsrTime        += 1; /* TODO. */
	Vp->CsrCycleCount  += 1;
	Vp->CsrInstRetired += 1;
}