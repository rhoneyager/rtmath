#pragma once
#include "recursivePolynomial.h"

namespace rtmath {
	class polynomial;
	namespace recPolys {
		/** \brief Laguerre polynomial functions

		Provides ability to calculate Legendre polynomials and 
		their roots.

		Used when calculating Gaussian quadratures.
		**/
		class DLEXPORT_rtmath_core laguerre : public recPoly {
		public:
			virtual void get(unsigned int rank, polynomial &res) const ;
		};
	};
};



