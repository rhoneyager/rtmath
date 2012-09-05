#pragma once

namespace rtmath
{
	namespace ddscat
	{
		namespace ddDataParsers
		{
			enum id
			{
				VERSION, // string
				TARGET, // string
				CMDSOL, // string
				CMDPOL, // string
				CSHAPE, // string
				NDIPOLES, // size_t
				AEFF, // double
				DAEFF, // double (d/aeff)
				D, // double
				WAVELENGTH, //double
				KAEFF, // double
				NAMBIENT, // double
				REFF, // double
				TOL, // double
				AXISA1, // double 3
				AXISA2, // double 3
				NAVG, // double
				KTF, // double 3
				KLAB, // double 3
				INCPOL1TF, // double 3
				INCPOL2TF, // double 3
				INCPOL1LF, // double 3
				INCPOL2LF, // double 3
				BETA, //
				THETA, //
				PHI, //
				BETARANGE,
				THETARANGE,
				PHIRANGE,
				ETASCA, // DOUBLE
				NUMTARORI,
				NUMINCPOLORI,
				UNKNOWN
			};
		}
	}
}

