#pragma once

namespace rtmath {
	namespace ddscat {
		namespace ddParParsers {

			/// \todo Switch to an enum class
			enum ParId
			{
				CMTORQ,			// string
				CMDSOL,			// string
				CMDFFT,			// string
				CALPHA,			// string
				CBINFLAG,		// string
				DIMENSION,		// size_t	3
				CSHAPE,			// string
				SHAPEPARAMS,	// double	3
				NCOMP,			// size_t
				IREFR,			// string
				NRFLD,			// size_t (bool)				version 7.2+
				FRACT_EXTENS,	// double	6					version 7.2+
				TOL,			// double
				MXITER,			// size_t						version 7.2+
				GAMMA,			// double
				ETASCA,			// double

				WAVELENGTHS,	// SPECIAL (2 double, size_t, string)
				NAMBIENT,		// double
				AEFF,			// SPECIAL (2 double, size_t, string)

				POLSTATE,		// SPECIAL (3 pair doubles)
				IORTH,			// size_t
				IWRKSC,			// size_t (bool)
				IWRPOL,			// size_t (bool)				version 7.0
				NBETA,			// SPECIAL (2 double, size_t)
				NTHETA,			// SPECIAL (2 double, size_t)
				NPHI,			// SPECIAL (2 double, size_t)
				IWAV,			// double	3
				NSMELTS,		// size_t
				INDICESIJ,		// size_t	(NSMELTS)
				CMDFRM,			// string
				NPLANES,		// size_t
				// The scattering planes are now in a separate structure
				// but, PLANE1 is preserved as a special case in the code
				PLANE1,			// double	4
				//PLANE2,			// double	4
				UNKNOWN
			};

		}
	}
}

