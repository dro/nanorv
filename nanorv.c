#include "nanorv.h"

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
#define RV_OPCODE_LOAD       (0b00'000'11)
#define RV_OPCODE_LOAD_FP    (0b00'001'11)
#define RV_OPCODE_CUSTOM0    (0b00'010'11)
#define RV_OPCODE_MISC_MEM   (0b00'011'11)
#define RV_OPCODE_OP_IMM     (0b00'100'11)
#define RV_OPCODE_AUIPC      (0b00'101'11)
#define RV_OPCODE_OP_IMM32   (0b00'110'11) 

#define RV_OPCODE_STORE      (0b01'000'11)
#define RV_OPCODE_STORE_FP   (0b01'001'11)
#define RV_OPCODE_CUSTOM1    (0b01'010'11)
#define RV_OPCODE_AMO        (0b01'011'11)
#define RV_OPCODE_OP         (0b01'100'11)
#define RV_OPCODE_LUI        (0b01'101'11)
#define RV_OPCODE_OP_32      (0b01'110'11)

#define RV_OPCODE_MADD       (0b10'000'11)
#define RV_OPCODE_MSUB       (0b10'001'11)
#define RV_OPCODE_NMSUB      (0b10'010'11)
#define RV_OPCODE_NMADD      (0b10'011'11)
#define RV_OPCODE_OP_FP      (0b10'100'11)
#define RV_OPCODE_RESERVED0  (0b10'101'11)
#define RV_OPCODE_CUSTOM2    (0b10'110'11)

#define RV_OPCODE_BRANCH     (0b11'000'11)
#define RV_OPCODE_JALR       (0b11'001'11)
#define RV_OPCODE_RESERVED1  (0b11'010'11)
#define RV_OPCODE_JAL        (0b11'011'11)
#define RV_OPCODE_SYSTEM     (0b11'100'11)
#define RV_OPCODE_RESERVED2  (0b11'101'11)
#define RV_OPCODE_CUSTOM3    (0b11'110'11)

//
// Instruction funct3 values.
//

//
// Base opcode RV_OPCODE_JALR funct3 values (I-type).
// imm[11:0] rs1 000 rd 1100111 JALR
//
#define RV_JALR_FUNCT3_BASE (0b000)

//
// Base opcode RV_OPCODE_BRANCH funct3 values (B-type).
//
#define RV_BRANCH_FUNCT3_BEQ  (0b000)
#define RV_BRANCH_FUNCT3_BNE  (0b001)
#define RV_BRANCH_FUNCT3_BLT  (0b100)
#define RV_BRANCH_FUNCT3_BGE  (0b101)
#define RV_BRANCH_FUNCT3_BLTU (0b110)
#define RV_BRANCH_FUNCT3_BGEU (0b111)

//
// Base opcode RV_OPCODE_LOAD funct3 values (I-type).
//
#define RV_LOAD_FUNCT3_LB  (0b000)
#define RV_LOAD_FUNCT3_LH  (0b001)
#define RV_LOAD_FUNCT3_LW  (0b010)
#define RV_LOAD_FUNCT3_LBU (0b100)
#define RV_LOAD_FUNCT3_LHU (0b101)

//
// Base opcode RV_OPCODE_STORE funct3 values (S-type).
//
#define RV_STORE_FUNCT3_SB (0b000)
#define RV_STORE_FUNCT3_SH (0b001)
#define RV_STORE_FUNCT3_SW (0b010)
#define RV_STORE_FUNCT3_SD (0b011)

//
// Base opcode RV_OPCODE_OP_IMM funct3 values (I-type).
//
#define RV_OP_IMM_FUNCT3_ADDI  (0b000)
#define RV_OP_IMM_FUNCT3_SLTI  (0b010)
#define RV_OP_IMM_FUNCT3_SLTIU (0b011)
#define RV_OP_IMM_FUNCT3_XORI  (0b100)
#define RV_OP_IMM_FUNCT3_ORI   (0b110)
#define RV_OP_IMM_FUNCT3_ANDI  (0b111)

//
// Base opcode RV_OPCODE_OP_IMM funct3 values (R-type).
//
#define RV_OP_IMM_FUNCT3_SLLI      (0b001)
#define RV_OP_IMM_FUNCT3_SRLI_SRAI (0b101)

//
// Base opcode RV_OPCODE_OP funct3 values (R-type).
//
#define RV_OP_FUNCT3_ADD_SUB (0b000)
#define RV_OP_FUNCT3_SLL     (0b001)
#define RV_OP_FUNCT3_SLT     (0b010)
#define RV_OP_FUNCT3_SLTU    (0b011)
#define RV_OP_FUNCT3_XOR     (0b100)
#define RV_OP_FUNCT3_SRL_SRA (0b101)
#define RV_OP_FUNCT3_OR      (0b110)
#define RV_OP_FUNCT3_AND     (0b111)

//
// Base opcode RV_OPCODE_MISC_MEM funct3 values (I-type?)
//
#define RV_MISC_MEM_FUNCT3_FENCE (0b000)

//
// Base opcode RV_OPCODE_SYSTEM funct3 values.
//
#define RV_SYSTEM_FUNCT3_ECALL_EBREAK  (0b000)

//
// RV32/RV64 Zicsr Standard Extension RV_OPCODE_SYSTEM funct3 values.
//
#define RV_SYSTEM_FUNCT3_CSRRW  (0b001)
#define RV_SYSTEM_FUNCT3_CSRRS  (0b010)
#define RV_SYSTEM_FUNCT3_CSRRC  (0b011)
#define RV_SYSTEM_FUNCT3_CSRRWI (0b101)
#define RV_SYSTEM_FUNCT3_CSRRSI (0b110)
#define RV_SYSTEM_FUNCT3_CSRRCI (0b111)

//
// RV32M RV_OPCODE_OP funct3 values.
//
#define RV_OP_FUNCT3_RV32M_MUL    (0b000)
#define RV_OP_FUNCT3_RV32M_MULH   (0b001)
#define RV_OP_FUNCT3_RV32M_MULHSU (0b010)
#define RV_OP_FUNCT3_RV32M_MULHU  (0b011)
#define RV_OP_FUNCT3_RV32M_DIV    (0b100)
#define RV_OP_FUNCT3_RV32M_DIVU   (0b101)
#define RV_OP_FUNCT3_RV32M_REM    (0b110)
#define RV_OP_FUNCT3_RV32M_REMU   (0b111)

//
// RV32F RV_OPCODE_OP_FP funct3 values.
//
#define	RV_OP_FP_FUNCT3_FSGNJ_S	 (0b000)
#define	RV_OP_FP_FUNCT3_FSGNJN_S (0b001)
#define	RV_OP_FP_FUNCT3_FSGNJX_S (0b010)
#define	RV_OP_FP_FUNCT3_FMIN_S	 (0b000)
#define	RV_OP_FP_FUNCT3_FMAX_S	 (0b001)
#define	RV_OP_FP_FUNCT3_FMV_X_W	 (0b000)
#define	RV_OP_FP_FUNCT3_FEQ_S	 (0b010)
#define	RV_OP_FP_FUNCT3_FLT_S	 (0b001)
#define	RV_OP_FP_FUNCT3_FLE_S	 (0b000)
#define	RV_OP_FP_FUNCT3_FCLASS_S (0b001)
#define	RV_OP_FP_FUNCT3_FMV_W_X	 (0b000)

//
// RV64I RV_OPCODE_OP_IMM_32 funct3 values.
//
#define RV_OP_IMM_32_FUNCT3_ADDIW       (0b000)
#define RV_OP_IMM_32_FUNCT3_SLLIW       (0b001)
#define RV_OP_IMM_32_FUNCT3_SRLIW_SRAIW (0b101)

//
// RV64I RV_OPCODE_OP_32 funct3 values.
//
#define RVP_OP_32_FUNCT3_ADDW (0b000)
#define RVP_OP_32_FUNCT3_SUBW (0b000)
#define RVP_OP_32_FUNCT3_SLLW (0b001)
#define RVP_OP_32_FUNCT3_SRLW (0b101)
#define RVP_OP_32_FUNCT3_SRAW (0b101)

//
// RV64M RV_OPCODE_OP_32 funct3 values.
//
#define RV_OP_32_FUNCT3_RV64M_MULW  (0b000)
#define RV_OP_32_FUNCT3_RV64M_DIVW  (0b100)
#define RV_OP_32_FUNCT3_RV64M_DIVUW (0b101)
#define RV_OP_32_FUNCT3_RV64M_REMW  (0b110)
#define RV_OP_32_FUNCT3_RV64M_REMUW (0b111)

//
// Instruction funct7 values.
//

//
// Base opcode RV_OPCODE_OP_IMM funct7 values (R-type).
//
#define RV_OP_IMM_FUNCT7_SLLI (0b0000000)
#define RV_OP_IMM_FUNCT7_SRLI (0b0000000)
#define RV_OP_IMM_FUNCT7_SRAI (0b0100000)

//
// Base opcode RV_OPCODE_OP funct7 values (R-type).
//
#define RV_OP_FUNCT7_ADD   (0b0000000)
#define RV_OP_FUNCT7_SUB   (0b0100000)
#define RV_OP_FUNCT7_SLL   (0b0000000)
#define RV_OP_FUNCT7_SLT   (0b0000000)
#define RV_OP_FUNCT7_SLTU  (0b0000000)
#define RV_OP_FUNCT7_XOR   (0b0000000)
#define RV_OP_FUNCT7_SRL   (0b0000000)
#define RV_OP_FUNCT7_SRA   (0b0100000)
#define RV_OP_FUNCT7_OR	   (0b0000000)
#define RV_OP_FUNCT7_AND   (0b0000000)
#define RV_OP_FUNCT7_RV32M (0b0000001)

//
// RV_OPCODE_OP_IMM_32 funct7 values.
//
#define RV_OP_IMM_32_FUNCT7_SLLIW (0b0000000)
#define RV_OP_IMM_32_FUNCT7_SRLIW (0b0000000)
#define RV_OP_IMM_32_FUNCT7_SRAIW (0b0100000)

//
// RV_OPCODE_OP_32 funct7 values.
//
#define RV_OP_32_FUNCT7_ADDW  (0b0000000)
#define RV_OP_32_FUNCT7_SUBW  (0b0100000)
#define RV_OP_32_FUNCT7_SLLW  (0b0000000)
#define RV_OP_32_FUNCT7_SRLW  (0b0000000)
#define RV_OP_32_FUNCT7_SRAW  (0b0100000)
#define RV_OP_32_FUNCT7_RV64M (0b0000001)

//
// RV32F RV_OPCODE_OP_FP funct7 values.
//
#define RV_OP_FP_FUNCT7_FADD_S	   (0b0000000)
#define RV_OP_FP_FUNCT7_FSUB_S	   (0b0000100)
#define RV_OP_FP_FUNCT7_FMUL_S	   (0b0001000)
#define RV_OP_FP_FUNCT7_FDIV_S	   (0b0001100)
#define RV_OP_FP_FUNCT7_FSQRT_S	   (0b0101100)
#define RV_OP_FP_FUNCT7_FSGNJ_S	   (0b0010000)
#define RV_OP_FP_FUNCT7_FSGNJN_S   (0b0010000)
#define RV_OP_FP_FUNCT7_FSGNJX_S   (0b0010000)
#define RV_OP_FP_FUNCT7_FMIN_S	   (0b0010100)
#define RV_OP_FP_FUNCT7_FMAX_S	   (0b0010100)
#define RV_OP_FP_FUNCT7_FCVT_W_S   (0b1100000)
#define RV_OP_FP_FUNCT7_FCVT_WU_S  (0b1100000)
#define RV_OP_FP_FUNCT7_FMV_X_W	   (0b1110000)
#define RV_OP_FP_FUNCT7_FEQ_S	   (0b1010000)
#define RV_OP_FP_FUNCT7_FLT_S	   (0b1010000)
#define RV_OP_FP_FUNCT7_FLE_S	   (0b1010000)
#define RV_OP_FP_FUNCT7_FCLASS_S   (0b1110000)
#define RV_OP_FP_FUNCT7_FCVT_S_W   (0b1101000)
#define RV_OP_FP_FUNCT7_FCVT_S_WU  (0b1101000)
#define RV_OP_FP_FUNCT7_FMV_W_X	   (0b1111000)

//
// Special RV32F rs2 values.
//
#define RV_OP_FP_RS2_FSQRT_S   (0b00000)
#define RV_OP_FP_RS2_FCVT_W_S  (0b00000)
#define RV_OP_FP_RS2_FCVT_WU_S (0b00001)
#define RV_OP_FP_RS2_FMV_X_W   (0b00000)
#define RV_OP_FP_RS2_FCLASS_S  (0b00000)
#define RV_OP_FP_RS2_FCVT_S_W  (0b00000)
#define RV_OP_FP_RS2_FCVT_S_WU (0b00001)
#define RV_OP_FP_RS2_FMV_W_X   (0b00000)

//
// Instruction funct12 values.
//

//
// Base opcode RV_OPCODE_SYSTEM funct12 values.
//
#define RV_SYSTEM_FUNCT12_ECALL  (0b000000000000)
#define RV_SYSTEM_FUNCT12_EBREAK (0b000000000001)

//
// Floating-point-specific values.
//

//
// RV32F 2-bit Floating-point format field fmt values.
//
#define RV_FP_FMT_S (0b00) /* 32-bit single-precision. */
#define RV_FP_FMT_D (0b01) /* 64-bit double-precision. */
#define RV_FP_FMT_H (0b10) /* 16-bit half-precision. */
#define RV_FP_FMT_Q (0b11) /* 128-bit quad-precision. */

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
#define RV_FCSR_NV_SHIFT        (4ul) /* Divide by Zero. */
#define RV_FCSR_NV_MASK         (1ul)
#define RV_FCSR_NV_FLAG         (RV_FCSR_NV_MASK << RV_FCSR_NV_SHIFT)
#define RV_FCSR_FRM_SHIFT       (5ul) /* Rounding Mode. */
#define RV_FCSR_FRM_MASK        ((1ul << (7 - 5 + 1)) - 1)
#define RV_FCSR_RESERVED0_SHIFT (8ul) /* Reserved. */
#define RV_FCSR_RESERVED0_MASK  ((1ul << (31 - 8 + 1)) - 1)

//
// RV32F rounding modes.
//
#define RV_ROUNDING_MODE_RNE        (0b000) /* Round to Nearest, ties to Even						 */
#define RV_ROUNDING_MODE_RTZ        (0b001) /* Round towards Zero									 */
#define RV_ROUNDING_MODE_RDN        (0b010) /* Round Down( towards -inf )							 */
#define RV_ROUNDING_MODE_RUP        (0b011) /* Round Up( towards +inf )								 */
#define RV_ROUNDING_MODE_RMM        (0b100) /* Round to Nearest, ties to Max Magnitude				 */
#define RV_ROUNDING_MODE_RESERVED0  (0b101) /* Reserved for future use.								 */
#define RV_ROUNDING_MODE_RESERVED1  (0b110) /* Reserved for future use.								 */
#define RV_ROUNDING_MODE_DYN        (0b111) /* Only valid if specified in an instruction's rm field. */

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
// MMU-related values.
//

//
// Page permission flags (temporary).
//
#define RV_PAGE_FLAG_R (1ul << 1)
#define RV_PAGE_FLAG_W (1ul << 2)
#define RV_PAGE_FLAG_X (1ul << 3)

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
// Attempts to convert a guest address to host address using flat VA span MMU mode.
//
_Success_( return )
static
RV_BOOLEAN
RvpMmuResolveGuestAddressFlat(
	_Inout_  RV_PROCESSOR* Vp,
	_In_     RV_UINTR      Address,
	_In_     RV_SIZE_T     Size,
	_In_     RV_UINT32     AccessFlags,
	_Outptr_ VOID**        ppHostData
	)
{
	UNREFERENCED_PARAMETER( AccessFlags );

	//
	// Currently only allow a flat space of contiguous memory to be allocated to the guest.
	//
	if( Address < Vp->VaSpanGuestBase
		|| Size > Vp->VaSpanSize
		|| ( Address + Size ) > ( Vp->VaSpanGuestBase + Vp->VaSpanSize ) )
	{
		return RV_FALSE;
	}

	*ppHostData = ( ( RV_UINT8* )Vp->VaSpanHostBase + ( Address - Vp->VaSpanGuestBase ) );
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
	_Out_   RV_UINT32*    pWord
	)
{
	VOID* HostAddress;

	//
	// Currently only allow a flat space of contiguous memory to be allocated to the guest.
	//
	if( RvpMmuResolveGuestAddressFlat( Vp, Address, sizeof( *pWord ),
									   ( RV_PAGE_FLAG_R | RV_PAGE_FLAG_X ),
									   &HostAddress ) == RV_FALSE )
	{
		return RV_FALSE;
	}

	//
	// Convert from big-endian to host endianness and return fetched word.
	//
	*pWord = RV_BIG_ENDIAN( *( RV_UINT32* )HostAddress );
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
	RV_UINT32 Funct3;
	RV_UINT32 Rd;
	RV_UINT32 Rs1;
	RV_UINT32 Offset;
	RV_UINTR  Address;
	VOID*     HostData;

	//
	// Decode I-type instruction fields.
	//
	Funct3 = ( ( Instruction >> RV_INST_I_FUNCT3_SHIFT ) & RV_INST_I_FUNCT3_MASK );
	Rd     = ( ( Instruction >> RV_INST_I_RD_SHIFT ) & RV_INST_I_RD_MASK );
	Rs1    = ( ( Instruction >> RV_INST_I_RS1_SHIFT ) & RV_INST_I_RS1_MASK );
	Offset = ( ( Instruction >> RV_INST_I_IMM_11_0_SHIFT ) & RV_INST_I_IMM_11_0_MASK );
	Offset = RvpSignExtend32( Vp, Offset, 11 );

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
	Address = ( Vp->Xr[ Rs1 ] + ( RV_INTR )( RV_INT32 )Offset );

	//
	// Lookup the host data address for the given guest effective address, and perform the load.
	// TODO: Update this to use generic ReadMemory function to handle cross
	// page-boundary accesses once MMU is fleshed out!
	// 
	// Base opcode RV_OPCODE_LOAD funct3 values (I-type).
	// imm[11:0] rs1 000 rd 0000011 LB
	// imm[11:0] rs1 001 rd 0000011 LH
	// imm[11:0] rs1 010 rd 0000011 LW
	// imm[11:0] rs1 100 rd 0000011 LBU
	// imm[11:0] rs1 101 rd 0000011 LHU
	//
	switch( Funct3 ) {
	case RV_LOAD_FUNCT3_LW:
		//
		// The LW instruction loads a 32-bit value from memory into rd.
		//
		if( RvpMmuResolveGuestAddressFlat( Vp, Address, sizeof( RV_UINT32 ), RV_PAGE_FLAG_R, &HostData ) == RV_FALSE ) {
			RvpExceptionPush( Vp, RV_EXCEPTION_STORE_AMO_PAGE_FAULT );
			return;
		}
		Vp->Xr[ Rd ] = *( RV_UINT32* )HostData;
		break;
	case RV_LOAD_FUNCT3_LH:
		//
		// LH loads a 16-bit value from memory, then sign-extends to 32-bits before storing in rd.
		//
		if( RvpMmuResolveGuestAddressFlat( Vp, Address, sizeof( RV_UINT16 ), RV_PAGE_FLAG_R, &HostData ) == RV_FALSE ) {
			RvpExceptionPush( Vp, RV_EXCEPTION_STORE_AMO_PAGE_FAULT );
			return;
		}
		Vp->Xr[ Rd ] = RvpSignExtend32( Vp, *( RV_UINT16* )HostData, 15 );
		break;
	case RV_LOAD_FUNCT3_LHU:
		//
		// LHU loads a 16-bit value from memory but then zero extends to 32-bits before storing in rd.
		//
		if( RvpMmuResolveGuestAddressFlat( Vp, Address, sizeof( RV_UINT16 ), RV_PAGE_FLAG_R, &HostData ) == RV_FALSE ) {
			RvpExceptionPush( Vp, RV_EXCEPTION_STORE_AMO_PAGE_FAULT );
			return;
		}
		Vp->Xr[ Rd ] = *( RV_UINT16* )HostData;
		break;
	case RV_LOAD_FUNCT3_LB:
		//
		// LB loads an 8-bit value from memory, then sign-extends to 32-bits before storing in rd.
		//
		if( RvpMmuResolveGuestAddressFlat( Vp, Address, sizeof( RV_UINT8 ), RV_PAGE_FLAG_R, &HostData ) == RV_FALSE ) {
			RvpExceptionPush( Vp, RV_EXCEPTION_STORE_AMO_PAGE_FAULT );
			return;
		}
		Vp->Xr[ Rd ] = RvpSignExtend32( Vp, *( RV_UINT8* )HostData, 7 );
		break;
	case RV_LOAD_FUNCT3_LBU:
		//
		// LBU loads an 8-bit value from memory but then zero extends to 32-bits before storing in rd.
		//
		if( RvpMmuResolveGuestAddressFlat( Vp, Address, sizeof( RV_UINT8 ), RV_PAGE_FLAG_R, &HostData ) == RV_FALSE ) {
			RvpExceptionPush( Vp, RV_EXCEPTION_STORE_AMO_PAGE_FAULT );
			return;
		}
		Vp->Xr[ Rd ] = *( RV_UINT8* )HostData;
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
	RV_UINT32 Funct3;
	RV_UINT32 Offset;
	RV_UINT32 Rs1;
	RV_UINT32 Rs2;
	RV_UINTR  Address;
	VOID*     HostData;

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
	//
	switch( Funct3 ) {
	case RV_STORE_FUNCT3_SB:
		if( RvpMmuResolveGuestAddressFlat( Vp, Address, sizeof( RV_UINT8 ), RV_PAGE_FLAG_W, &HostData ) == RV_FALSE ) {
			RvpExceptionPush( Vp, RV_EXCEPTION_STORE_AMO_PAGE_FAULT );
			return;
		}
		*( RV_UINT8* )HostData = ( RV_UINT8 )Vp->Xr[ Rs2 ];
		break;
	case RV_STORE_FUNCT3_SH:
		if( RvpMmuResolveGuestAddressFlat( Vp, Address, sizeof( RV_UINT16 ), RV_PAGE_FLAG_W, &HostData ) == RV_FALSE ) {
			RvpExceptionPush( Vp, RV_EXCEPTION_STORE_AMO_PAGE_FAULT );
			return;
		}
		*( RV_UINT16* )HostData = ( RV_UINT16 )Vp->Xr[ Rs2 ];
		break;
	case RV_STORE_FUNCT3_SW:
		if( RvpMmuResolveGuestAddressFlat( Vp, Address, sizeof( RV_UINT32 ), RV_PAGE_FLAG_W, &HostData ) == RV_FALSE ) {
			RvpExceptionPush( Vp, RV_EXCEPTION_STORE_AMO_PAGE_FAULT );
			return;
		}
		*( RV_UINT32* )HostData = ( RV_UINT32 )Vp->Xr[ Rs2 ];
		break;
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
		// Unimplemented.
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

//
// TODO: Fix this for rv64i, decoding of shamt requires an extra bit!
//
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
		if( ShiftType == 0b000000 ) {
			Vp->Xr[ Rd ] = ( Vp->Xr[ Rs1 ] >> Shamt );
		} else if( ShiftType == 0b100000 ) {
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
// S64xS64 -> upper half of 128bit result.
//
static
RV_INT64
RvpMulh64(
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
	#error "Unsupported."
#endif
}

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
	#error "Unsupported."
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

#ifdef RV_OPT_BUILD_MSVC
	return ( ( RV_INT64 )RvpUMulh64( Vp, ( RV_UINT64 )Operand1, Operand2 ) - ( ( Operand1 >> 63ull ) & ( RV_INT64 )Operand2 ) );
#elif defined(RV_OPT_BUILD_INT128_TYPES)
	return ( ( ( int128_t )Operand1 * ( uint128_t )Operand2 ) >> 64 );
#else
	#error "Unsupported."
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
#if defined(RV_OPT_RV32I)
		Vp->Xr[ Rd ] = ( RV_UINTR )( RV_INTR )( ( ( RV_INT64 )Vp->Xr[ Rs1 ] * ( RV_INT64 )Vp->Xr[ Rs2 ] ) >> 32ull );
#elif defined(RV_OPT_RV64I)
		Vp->Xr[ Rd ] = ( RV_UINTR )( RV_INTR )RvpMulh64( Vp, ( RV_INT64 )Vp->Xr[ Rs1 ], ( RV_INT64 )Vp->Xr[ Rs2 ] );
#else
		#error "Unsupported XLEN mode."
#endif
		break;
	case RV_OP_FUNCT3_RV32M_MULHU:
#if defined(RV_OPT_RV32I)
		Vp->Xr[ Rd ] = ( RV_UINTR )( ( ( RV_UINT64 )Vp->Xr[ Rs1 ] * ( RV_UINT64 )Vp->Xr[ Rs2 ] ) >> 32ull );
#elif defined(RV_OPT_RV64I)
		Vp->Xr[ Rd ] = ( RV_UINTR )( RV_INTR )RvpUMulh64( Vp, Vp->Xr[ Rs1 ], Vp->Xr[ Rs2 ] );
#else
		#error "Unsupported XLEN mode."
#endif
		break;
	case RV_OP_FUNCT3_RV32M_MULHSU:
#if defined(RV_OPT_RV32I)
		Vp->Xr[ Rd ] = ( RV_UINTR )( RV_INTR )( ( ( RV_INT64 )Vp->Xr[ Rs1 ] * ( RV_UINT64 )Vp->Xr[ Rs2 ] ) >> 32ull );
#elif defined(RV_OPT_RV64I)
		Vp->Xr[ Rd ] = ( RV_UINTR )( RV_INTR )RvpSUMulh64( Vp, Vp->Xr[ Rs1 ], Vp->Xr[ Rs2 ] );
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

//
// TODO: Improve the dispatch logic of this function, handling of funct7+funct3 combinations has become messy.
//
static
VOID
RvpInstructionExecuteOpcodeOp(
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
	// If the RV32M standard extension is enabled, dispatch to the corresponding handler.
	//
#if defined(RV_OPT_RV32M)
	if( Funct7 == RV_OP_FUNCT7_RV32M ) {
		RvpInstructionExecuteOpcodeOpRv32m( Vp, Instruction );
		return;
	}
#endif

	//
	// Base opcode RV_OPCODE_OP funct3 values (R-type).
	// 0000000 rs2   rs1 000 rd 0110011 ADD
	// 0100000 rs2   rs1 000 rd 0110011 SUB
	// 0000000 rs2   rs1 001 rd 0110011 SLL
	// 0000000 rs2   rs1 010 rd 0110011 SLT
	// 0000000 rs2   rs1 011 rd 0110011 SLTU
	// 0000000 rs2   rs1 100 rd 0110011 XOR
	// 0000000 rs2   rs1 101 rd 0110011 SRL
	// 0100000 rs2   rs1 101 rd 0110011 SRA
	// 0000000 rs2   rs1 110 rd 0110011 OR
	// 0000000 rs2   rs1 111 rd 0110011 AND
	//
	switch( Funct3 ) {
	case RV_OP_FUNCT3_ADD_SUB:
		if( Funct7 == RV_OP_FUNCT7_ADD ) {
			Vp->Xr[ Rd ] = ( Vp->Xr[ Rs1 ] + Vp->Xr[ Rs2 ] );
		} else if( Funct7 == RV_OP_FUNCT7_SUB ) {
			Vp->Xr[ Rd ] = ( Vp->Xr[ Rs1 ] - Vp->Xr[ Rs2 ] );
		} else {
			RvpExceptionPush( Vp, RV_EXCEPTION_ILLEGAL_INSTRUCTION );
			return;
		}
		break;
	case RV_OP_FUNCT3_SLT:
		if( Funct7 != RV_OP_FUNCT7_SLT ) {
			RvpExceptionPush( Vp, RV_EXCEPTION_ILLEGAL_INSTRUCTION );
			return;
		}
		Vp->Xr[ Rd ] = ( ( ( RV_INTR )Vp->Xr[ Rs1 ] < ( RV_INTR )Vp->Xr[ Rs2 ] ) ? 1 : 0 );
		break;
	case RV_OP_FUNCT3_SLTU:
		if( Funct7 != RV_OP_FUNCT7_SLTU ) {
			RvpExceptionPush( Vp, RV_EXCEPTION_ILLEGAL_INSTRUCTION );
			return;
		}
		Vp->Xr[ Rd ] = ( ( Vp->Xr[ Rs1 ] < Vp->Xr[ Rs2 ] ) ? 1 : 0 );
		break;
	case RV_OP_FUNCT3_XOR:
		if( Funct7 != RV_OP_FUNCT7_XOR ) {
			RvpExceptionPush( Vp, RV_EXCEPTION_ILLEGAL_INSTRUCTION );
			return;
		}
		Vp->Xr[ Rd ] = ( Vp->Xr[ Rs1 ] ^ Vp->Xr[ Rs2 ] );
		break;
	case RV_OP_FUNCT3_SLL:
		if( Funct7 != RV_OP_FUNCT7_SLL ) {
			RvpExceptionPush( Vp, RV_EXCEPTION_ILLEGAL_INSTRUCTION );
			return;
		}
		Vp->Xr[ Rd ] = ( Vp->Xr[ Rs1 ] << ( Vp->Xr[ Rs2 ] & 0x3f ) );
		break;
	case RV_OP_FUNCT3_SRL_SRA:
		if( Funct7 == RV_OP_FUNCT7_SRL ) {
			Vp->Xr[ Rd ] = ( Vp->Xr[ Rs1 ] >> ( Vp->Xr[ Rs2 ] & 0x3f ) );
		} else if( Funct7 == RV_OP_FUNCT7_SRA ) {
			Vp->Xr[ Rd ] = ( ( RV_INTR )Vp->Xr[ Rs1 ] >> ( Vp->Xr[ Rs2 ] & 0x3f ) );
		} else {
			RvpExceptionPush( Vp, RV_EXCEPTION_ILLEGAL_INSTRUCTION );
			return;
		}
		break;
	case RV_OP_FUNCT3_OR:
		if( Funct7 != RV_OP_FUNCT7_OR ) {
			RvpExceptionPush( Vp, RV_EXCEPTION_ILLEGAL_INSTRUCTION );
			return;
		}
		Vp->Xr[ Rd ] = ( Vp->Xr[ Rs1 ] | Vp->Xr[ Rs2 ] );
		break;
	case RV_OP_FUNCT3_AND:
		if( Funct7 != RV_OP_FUNCT7_AND ) {
			RvpExceptionPush( Vp, RV_EXCEPTION_ILLEGAL_INSTRUCTION );
			return;
		}
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
	case RV_CSR_VALUE_FFLAGS:
		Vp->CsrFFlags = NewValue;
		break;
	case RV_CSR_VALUE_FRM:
		Vp->CsrFrm = NewValue;
		break;
	case RV_CSR_VALUE_FCSR:
		Vp->CsrFcsr = NewValue;
		break;
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
	case RV_CSR_VALUE_FFLAGS:
		*pValue = Vp->CsrFFlags;
		break;
	case RV_CSR_VALUE_FRM:
		*pValue = Vp->CsrFrm;
		break;
	case RV_CSR_VALUE_FCSR:
		*pValue = Vp->CsrFcsr;
		break;
	case RV_CSR_VALUE_CYCLE:
		*pValue = ( RV_UINTR )Vp->CsrCycleCount;
		break;
	case RV_CSR_VALUE_TIME:
		*pValue = ( RV_UINTR )Vp->CsrTime;
		break;
	case RV_CSR_VALUE_INSTRET:
		*pValue = ( RV_UINTR )Vp->CsrInstRetired;
		break;
#if defined (RV_OPT_RV32I) //&& !defined(RV_OPT_RV64I)
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
RV_UINTR
RvpCsrAtomicSwap(
	_Inout_ RV_PROCESSOR* Vp,
	_Inout_ RV_UINTR*     pCsrRegister,
	_In_    RV_UINTR      NewValue
	)
{
	RV_UINTR OldValue;

	OldValue      = *pCsrRegister;
	*pCsrRegister = NewValue;

	return OldValue;
}

static
RV_UINTR
RvpCsrAtomicReadAndSetBits(
	_Inout_ RV_PROCESSOR* Vp,
	_Inout_ RV_UINTR*     pCsrRegister,
	_In_    RV_UINTR      SetBitMask
	)
{
	RV_UINTR OldValue;

	OldValue      = *pCsrRegister;
	*pCsrRegister = ( OldValue | SetBitMask );

	return OldValue;
}

static
RV_UINTR
RvpCsrAtomicReadAndClearBits(
	_Inout_ RV_PROCESSOR* Vp,
	_Inout_ RV_UINTR*     pCsrRegister,
	_In_    RV_UINTR      ClearBitMask
	)
{
	RV_UINTR OldValue;

	OldValue      = *pCsrRegister;
	*pCsrRegister = ( OldValue & ~ClearBitMask );

	return OldValue;
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

#if defined(RV_OPT_RV64I)
#if defined(RV_OPT_RV32M)

//
// RV64M opcodes.
//

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
		if( ShiftType != 0b000000 ) {
			RvpExceptionPush( Vp, RV_EXCEPTION_ILLEGAL_INSTRUCTION );
			return;
		}
		Vp->Xr[ Rd ] = ( RV_INTR )( RV_INT32 )( ( RV_UINT32 )Vp->Xr[ Rs1 ] << Shamt );
		break;
	case RV_OP_IMM_32_FUNCT3_SRLIW_SRAIW:
		if( ShiftType == 0b000000 ) {
			Vp->Xr[ Rd ] = ( RV_INTR )( RV_INT32 )( ( RV_UINT32 )Vp->Xr[ Rs1 ] >> Shamt );
		} else if( ShiftType == 0b010000 ) {
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
// Atomic memory operation opcodes.
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
// RV32F instructions, WIP.
//

#if 0
static
VOID
RvpInstructionExecuteOpcodeMAdd(
	_Inout_ RV_PROCESSOR* Vp,
	_In_    RV_UINT32     Instruction
	)
{
	RvpExceptionPush( Vp, RV_EXCEPTION_ILLEGAL_INSTRUCTION );
	return;
}

static
VOID
RvpInstructionExecuteOpcodeMSub(
	_Inout_ RV_PROCESSOR* Vp,
	_In_    RV_UINT32     Instruction
	)
{
	RvpExceptionPush( Vp, RV_EXCEPTION_ILLEGAL_INSTRUCTION );
	return;
}

static
VOID
RvpInstructionExecuteOpcodeNMSub(
	_Inout_ RV_PROCESSOR* Vp,
	_In_    RV_UINT32     Instruction
	)
{
	RvpExceptionPush( Vp, RV_EXCEPTION_ILLEGAL_INSTRUCTION );
	return;
}

static
VOID
RvpInstructionExecuteOpcodeNMAdd(
	_Inout_ RV_PROCESSOR* Vp,
	_In_    RV_UINT32     Instruction
	)
{
	RvpExceptionPush( Vp, RV_EXCEPTION_ILLEGAL_INSTRUCTION );
	return;
}

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
	Vp->HostFpuCsr = HostFpuCsr;
	_mm_setcsr( HostFpuCsr );
#else
	Vp->HostFpuCsr = 0;
#endif
}

static
RV_FLOAT
RvpFpuHostSqrtF32(
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
#else
	#error "Unsupported."
#endif
}

static
VOID
RvpInstructionExecuteOpcodeOpFp(
	_Inout_ RV_PROCESSOR* Vp,
	_In_    RV_UINT32     Instruction
	)
{
	RV_UINT32 Opcode;
	RV_UINT32 RdFr;
	RV_UINT32 Rs1Fr;
	RV_UINT32 Rs2Fr;
	RV_UINT32 Funct3;
	RV_UINT32 Rm;
	RV_UINT32 Funct7;
	RV_UINT32 Class;

	//
	// Decode R-type fields.
	//
	Opcode = ( ( Instruction >> RV_INST_R_OPCODE_SHIFT ) & RV_INST_R_OPCODE_MASK );
	RdFr   = ( ( Instruction >> RV_INST_R_RD_SHIFT ) & RV_INST_R_RD_MASK );
	Rs1Fr  = ( ( Instruction >> RV_INST_R_RS1_SHIFT ) & RV_INST_R_RS1_MASK );
	Rs2Fr  = ( ( Instruction >> RV_INST_R_RS2_SHIFT ) & RV_INST_R_RS2_MASK );
	Funct3 = ( ( Instruction >> RV_INST_R_FUNCT3_SHIFT ) & RV_INST_R_FUNCT3_MASK );
	Rm     = Funct3;
	Funct7 = ( ( Instruction >> RV_INST_R_FUNCT7_SHIFT ) & RV_INST_R_FUNCT7_MASK );

	//
	// Classify Floating-point instruction
	//
	Class = RV_INST_CLASSIFY_F3F7( Opcode, 0, Funct7 );

	//
	// Validate register indices.
	// Note: Not all instructions use rs2, but use the rs2 field for their own purpose
	// in this case, the value of rs2 is always <= 1, so this check will still pass for them.
	//
	if( RdFr >= RV_COUNTOF( Vp->Fr ) || Rs1Fr >= RV_COUNTOF( Vp->Fr ) || Rs2Fr >= RV_COUNTOF( Vp->Fr ) ) {
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
	// 1110000 00000 rs1 000 rd 1010011 FMV.X.W
	// 1010000 rs2   rs1 010 rd 1010011 FEQ.S
	// 1010000 rs2   rs1 001 rd 1010011 FLT.S
	// 1010000 rs2   rs1 000 rd 1010011 FLE.S
	// 1110000 00000 rs1 001 rd 1010011 FCLASS.S
	// 1101000 00000 rs1 rm  rd 1010011 FCVT.S.W
	// 1101000 00001 rs1 rm  rd 1010011 FCVT.S.WU
	// 1111000 00000 rs1 000 rd 1010011 FMV.W.X
	//
	switch( Class ) {
	case RV_INST_CLASSIFY_F3F7( RV_OPCODE_OP_FP, 0, RV_OP_FP_FUNCT7_FADD_S ):
		RvpFpuHostSetRoundingMode( Vp, Rm );
		Vp->Fr[ RdFr ] = ( RV_FLOATR )( ( RV_FLOAT )Vp->Fr[ Rs1Fr ] + ( RV_FLOAT )Vp->Fr[ Rs2Fr ] );
		break;
	case RV_INST_CLASSIFY_F3F7( RV_OPCODE_OP_FP, 0, RV_OP_FP_FUNCT7_FSUB_S ):
		RvpFpuHostSetRoundingMode( Vp, Rm );
		Vp->Fr[ RdFr ] = ( RV_FLOATR )( ( RV_FLOAT )Vp->Fr[ Rs1Fr ] - ( RV_FLOAT )Vp->Fr[ Rs2Fr ] );
		break;
	case RV_INST_CLASSIFY_F3F7( RV_OPCODE_OP_FP, 0, RV_OP_FP_FUNCT7_FMUL_S ):
		RvpFpuHostSetRoundingMode( Vp, Rm );
		Vp->Fr[ RdFr ] = ( RV_FLOATR )( ( RV_FLOAT )Vp->Fr[ Rs1Fr ] * ( RV_FLOAT )Vp->Fr[ Rs2Fr ] );
		break;
	case RV_INST_CLASSIFY_F3F7( RV_OPCODE_OP_FP, 0, RV_OP_FP_FUNCT7_FDIV_S ):
		RvpFpuHostSetRoundingMode( Vp, Rm );
		Vp->Fr[ RdFr ] = ( RV_FLOATR )( ( RV_FLOAT )Vp->Fr[ Rs1Fr ] / ( RV_FLOAT )Vp->Fr[ Rs2Fr ] );
		break;
	case RV_INST_CLASSIFY_F3F7( RV_OPCODE_OP_FP, 0, RV_OP_FP_FUNCT7_FSQRT_S ):
		RvpFpuHostSetRoundingMode( Vp, Rm );
		Vp->Fr[ RdFr ] = ( RV_FLOATR )RvpFpuHostSqrtF32( Vp, ( RV_FLOAT )Vp->Fr[ Rs1Fr ] );
		break;
	default:
		RvpExceptionPush( Vp, RV_EXCEPTION_ILLEGAL_INSTRUCTION );
		return;
	}

	//
	// Implementation unfinished.
	//
	RvpExceptionPush( Vp, RV_EXCEPTION_ILLEGAL_INSTRUCTION );
	return;
}

static
VOID
RvpInstructionExecuteOpcodeStoreFp(
	_Inout_ RV_PROCESSOR* Vp,
	_In_    RV_UINT32     Instruction
	)
{
	RV_UINT32 Width;
	RV_UINT32 UOffset32;
	RV_UINT32 BaseXr;
	RV_UINT32 SrcFr;
	RV_UINTR  Address;
	VOID*     HostData;

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
	// TODO: Change this to use generic WriteMemory, and handle writes that cross page boundaries.
	//
	switch( Width ) {
	case RV_STORE_FUNCT3_SW:
		//
		// FSW stores a single-precision value from Floating-point register rs2 to memory.
		//
		if( RvpMmuResolveGuestAddressFlat( Vp, Address, sizeof( RV_FLOAT ), RV_PAGE_FLAG_W, &HostData ) == RV_FALSE ) {
			RvpExceptionPush( Vp, RV_EXCEPTION_STORE_AMO_PAGE_FAULT );
			return;
		}
		*( RV_FLOAT* )HostData = ( RV_FLOAT )Vp->Fr[ SrcFr ];
		break;
#if defined(RV_OPT_RV32D)
	case RV_STORE_FUNCT3_SD:
		//
		// FSD stores a double-precision value from the Floating-point registers to memory.
		//
		if( RvpMmuResolveGuestAddressFlat( Vp, Address, sizeof( RV_DOUBLE ), RV_PAGE_FLAG_W, &HostData ) == RV_FALSE ) {
			RvpExceptionPush( Vp, RV_EXCEPTION_STORE_AMO_PAGE_FAULT );
			return;
		}
		*( RV_DOUBLE* )HostData = ( RV_DOUBLE )Vp->Fr[ SrcFr ];
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
	RV_UINT32 Width;
	RV_UINT32 RdFr;
	RV_UINT32 BaseXr;
	RV_UINT32 UOffset32;
	RV_UINTR  Address;
	VOID*     HostData;

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
	// TODO: Change this to use generic ReadMemory, and handle writes that cross page boundaries.
	//
	switch( Width ) {
	case RV_STORE_FUNCT3_SW:
		//
		// The FLW instruction loads a single-precision Floating-point value from memory into Floating-point register rd.
		//
		if( RvpMmuResolveGuestAddressFlat( Vp, Address, sizeof( RV_FLOAT ), RV_PAGE_FLAG_R, &HostData ) == RV_FALSE ) {
			RvpExceptionPush( Vp, RV_EXCEPTION_STORE_AMO_PAGE_FAULT );
			return;
		}
		Vp->Fr[ RdFr ] = ( RV_FLOATR )( *( RV_FLOAT* )HostData );
		break;
#if defined(RV_OPT_RV32D)
	case RV_STORE_FUNCT3_SD:
		//
		// The FLD instruction loads a double-precision Floating-point value from memory into Floating-point register rd.
		//
		if( RvpMmuResolveGuestAddressFlat( Vp, Address, sizeof( RV_DOUBLE ), RV_PAGE_FLAG_R, &HostData ) == RV_FALSE ) {
			RvpExceptionPush( Vp, RV_EXCEPTION_STORE_AMO_PAGE_FAULT );
			return;
		}
		Vp->Fr[ RdFr ] = ( RV_FLOATR )( *( RV_DOUBLE* )HostData );
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
#if 0 /*defined(RV_OPT_RV32F) || defined(RV_OPT_RV32D)*/
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
	if( RvpFetchInstructionWord( Vp, Vp->Pc, &Instruction ) == RV_FALSE ) {
		RvpExceptionPush( Vp, RV_EXCEPTION_INSTRUCTION_PAGE_FAULT );
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
	// Increase timers.
	//
	Vp->CsrTime        += 1; /* TODO. */
	Vp->CsrCycleCount  += 1;
	Vp->CsrInstRetired += 1;
}