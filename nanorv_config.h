#pragma once

//
// Example nanorv user config.
//

//
// Enable base RV32I instruction set.
//
#define RV_OPT_RV32I

//
// Enable RV32M multiplication extension.
//
#define RV_OPT_RV32M

//
// Enable RV32F single precision floating-point extension (and RV64F if 
//
#define RV_OPT_RV32F

//
// Enable base RV64I instruction set (and RV64M/RV64F if enabled).
//
#define RV_OPT_RV64I

//
// Allow the use of MSVC-specific extensions and intrinsics.
//
// #define RV_OPT_BUILD_MSVC

//
// Allow the use of GCC-specific extensions and intrinsics.
//
// #define RV_OPT_BUILD_GCC

//
// Allow the use of Clang-specific extensions and intrinsics.
//
// #define RV_OPT_BUILD_CLANG

//
// Use actual SAL definitions.
//
// #define RV_OPT_BUILD_SAL

//
// Allow the use of SSE intrinsics (used by FP code).
//
// #define RV_OPT_BUILD_SSE

//
// Allow the use of CRT/LibC functions (used by FP code).
//
// #define RV_OPT_BUILD_LIBC

//
// Allow the use of GCC/clang-specific int128 types (used by RV32M code).
//
// #define RV_OPT_BUILD_INT128_TYPES

//
// Build for big-endian host environment.
//
// #define RV_OPT_HOST_BIG_ENDIAN
