# NanoRV
A minimalistic C11 RISC-V (RV32/64[I|M|F]) userspace emulator designed for embedded scripting.

Originally written as the backend for the Nano game engine's scripting system.

No external dependencies, and no implicit depedency on any runtime/LibC (unless specified through build options).

## Bases/extensions that are currently supported.
* RV32I
* RV64I
* RV32M/RV64M
* Zicsr
* FP/counter CSRs

## Bases/extensions that are a work in progress.
* RV32F/RV64F
* RV32D/RV64D
* RV32A/AMO

# Building
NanoRV is designed as a single pair of header and source files, intended to be directly embedded in an existing project.

Build configuration is specified through a set of `RV_` prefixed preprocessor macros. For a full listing of supported build options, view nanorv_config.h.

Use the `RV_OPT_INCLUDE_CONFIG` preprocessor definition to have NanoRV include a config file (nanorv_config.h) containing your build configuration,
or specify all your build configuration options project-wide through the compiler's preprocessor definition options.

# Example
```c
#include "nanorv.h"
#include <stdio.h>

VOID
RvTest(
  VOID
  )
{
  RV_SIZE_T    i;
  RV_PROCESSOR Vp;
  RV_UINT32    VpMemory[ 1024 ] =
  {
                // 0000000000000000 <_start>:
    0xb7100000, //    0:  lui ra,0x1
    0x13010010, //    4:  li  sp,256
    0x63441100, //    8:  blt sp,ra,10 
    0x6f00c000, //    c:  j   18 
    
                // 0000000000000010 :
    0x93012000, //   10:  li gp,2
    0x6f008000, //   14:  j  1c 
    
                // 0000000000000018:
    0x93011000, //   18:  li gp,1
    
                // 000000000000001c :
    0x33c23000, //   1c:  xor tp,ra,gp
    0x73001000, //   20:  ebreak
  };
  
  //
  // Set up initial processor context.
  //
  Vp = ( RV_PROCESSOR ) {
    /* Flat span of host memory to pass to the guest. */
    .VaSpanHostBase  = VpMemory,
    .VaSpanSize      = sizeof( VpMemory ),
    /* Begin the flat span of memory at guest effective address 0. */
    .VaSpanGuestBase = 0, 
    /* Begin executing at guest effective address 0. */
    .Pc              = 0
  };
  
  //
  // Execute code until an exception is triggered, or an EBREAK instruction is hit.
  //
  while( Vp.EBreakPending == 0 ) {
    //
    // Execute a tick; Fetches, decodes, and executes an instruction.
    //
    RvpTickExecute( &Vp );
    
    //
    // Print exception information.
    //
    if( Vp.ExceptionPending ) {
      printf( "RV exception %i @ PC=0x%llx\n", ( RV_INT )Vp.ExceptionIndex, ( RV_UINT64 )Vp.Pc );
      break;
    }
  }
  
  //
  // Dump all general-purpose registers.
  //
  for( i = 0; i < RV_COUNTOF( Vp.Xr ); i++ ) {
    printf( "x%i = 0x%llx\n", ( RV_INT )i, ( RV_UINT64 )Vp.Xr[ i ] );
  }
  
  //
  // Dump program-counter register.
  //
  printf( "pc = 0x%llx\n", ( RV_UINT64 )Vp.Pc );
}
```
## Output
```
x0 = 0x0
x1 = 0x1000
x2 = 0x100
x3 = 0x2
x4 = 0x1002
...
pc = 0x24
```
