#pragma once

// This header just provides the C++ translation of the FORTRAN definitions located
// in amplq.par.f and tmq.par.f

namespace tmatrix
{
	/// Provides a few static definitions
	namespace defs
	{
		/// Provides C++ translation of the Fortran definitions in ampld.par.f
		namespace ampld
		{
			const int NPN1 = 150;
			const int NPNG1 = 3*NPN1;
			const int NPNG2 = 2 * NPNG1;
			const int NPN2 = 2*NPN1;
			const int NPL = NPN2+1;
			const int NPN3 = NPN1+1;
			const int NPN4 = NPN1-25;
			const int NPN5 = 2*NPN4;
			const int NPN6 = NPN4+1;
		}
		/*
		namespace tmq
		{
			const int NPN1 = 140;
			const int NPNG1 = 500;
			const int NPNG2 = 2 * NPNG1;
			const int NPN2 = 2*NPN1;
			const int NPL = NPN2+1;
			const int NPN3 = NPN1+1;
			const int NPN4 = 100;
			const int NPN5 = 2*NPN4;
			const int NPN6 = NPN4+1;
			const int NPL1 = NPN5+1;
		}
		*/
	}
}
