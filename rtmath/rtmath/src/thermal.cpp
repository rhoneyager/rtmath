// Provides blackbody stuff and thermal emission information
#include "../rtmath/Stdafx.h"
#include <cmath>
#include "../rtmath/error/error.h"
#include "../rtmath/units.h"


namespace rtmath {
	double planck(double T, double f)
	{
		// T is temperature in K
		// f is frequency in GHz
		rtmath::units::conv_spec conv("GHz", "m");
		double lambda = conv.convert(f);
		return 0;
	}

}; // end namespace rtmath
