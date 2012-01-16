#include "../rtmath/Stdafx.h"
#include <memory>
#include <vector>
#include <map>
#include <iostream>
#include <fstream>
#include <sstream>
#include "../rtmath/atmos.h"
#include "../rtmath/da/daDiagonalMatrix.h"
#include "../rtmath/error/error.h"

namespace rtmath {
	
	namespace atmos {

		double absorber::_wvtofreq(double wvnum)
		{
			double f = wvnum * 2.99792458e8;
			// And, for appropriate dimensionality...
			f *= 1.e7;
			return f;
		}

		double absorber::_freqtowv(double f)
		{
			double wvnum = (f*1.e-7) / 2.99792458e8;
			return wvnum;
		}

	}; // end namespace atmos

}; // end namespace rtmath

