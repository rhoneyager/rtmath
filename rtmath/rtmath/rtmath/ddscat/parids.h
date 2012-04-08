#pragma once

namespace rtmath {
	namespace ddscat {
		namespace ddParParsers {

			enum ParId
			{
				CMTORQ,			// string
				CMDSOL,			// string
				CMDFFT,			// string
				CALPHA,			// string
				CBINFLAG,		// string
				DIMENSION,		// size_t	3
				CSHAPE,			// string
				SHAPEPARAMS,	// size_t	3
				NCOMP,			// size_t
				IREFR,			// string
				NRFLD,			// size_t (bool)				version 7.2
				FRACT_EXTENS,	// double	6					version 7.2
				TOL,			// double
				MXITER,			// size_t						version 7.2
				GAMMA,			// double
				ETASCA,			// double

				WAVELENGTHS,	// SPECIAL (2 double, size_t, string)
				NAMBIENT,		// double
				AEFF,			// SPECIAL (2 double, size_t, string)

				POLSTATE,		// SPECIAL (2 pair doubles)
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
				PLANE1,			// SPECIAL (3 double, size_t)
				PLANE2,			// SPECIAL (3 double, size_t)
				UNKNOWN
			};

		}
	}
}

