#pragma once
#include <memory>
#include <string>
#include <iostream>
#include <vector>
#include <map>
#include <set>
#include <unordered_map>
#include <complex>
#include "../matrixop.h"
#include "../phaseFunc.h"
#include "../coords.h"

#include "../ddscat/ddScattMatrix.h"

namespace rtmath {
	namespace ddscat {
		// This is a provider for ddScattMatrix. Instead of loading in ddscat files, however,
		// it instead generates the data ab-initio. This is useful for doing comparisons to dda
		// while maintaining the same file output format.
		class mieScattMatrix : public ddScattMatrix
		{
		public:
			mieScattMatrix(double freq, double theta, double reff);
			virtual ~mieScattMatrix();
		protected:
			double _reff;
			double _x;
			std::complex<double> _m;
		};
	}
}

