#include <stdio.h>
#include "nanorv.h"

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
		0xb7100000,	//    0:	lui	ra,0x1
		0x13010010,	//    4:	li	sp,256
		0x63441100,	//    8:	blt	sp,ra,10 
		0x6f00c000, //    c:	j	18 

					// 0000000000000010 :
		0x93012000,	//   10:	li	gp,2
		0x6f008000, //   14:	j	1c 

		         	// 0000000000000018:
		0x93011000, //   18:	li	gp,1

					// 000000000000001c :
		0x33c23000, //   1c:	xor	tp,ra,gp
		0x73001000,	//   20:	ebreak
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
	// Execute ticks.
	//
	while( RV_TRUE ) {
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

		//
		// Terminate execution upon EBREAK.
		//
		if( Vp.EBreakPending ) {
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