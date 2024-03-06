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
		0x93000001, 0x93901000, 0x13010010, 0x23201100,
		0xb7100000, 0xf32200c0, 0x13010010, 0x63441100,
		0x6f00c000, 0x93012000, 0x6f008000, 0x93011000,
		0x33c23000, 0x732300c0, 0x73001000, 0x00000000,
	};

	//
	// Set up initial processor context.
	//
	Vp = ( RV_PROCESSOR ) {
		/* Flat span of host memory to pass to the guest. */
		.MmuVaSpanHostBase  = VpMemory,
		.MmuVaSpanSize      = sizeof( VpMemory ),
		/* Begin the flat span of memory at guest effective address 0. */
		.MmuVaSpanGuestBase = 0,
		/* Begin executing at guest effective address 0. */
		.Pc                = 0
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

	//
	// Dump test value written to memory.
	//
	printf( "VpMemory[0x40] = 0x%x\n", VpMemory[ 0x40 ] );
}