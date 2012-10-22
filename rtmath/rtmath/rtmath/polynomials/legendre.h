#pragma once

/* Legendre polynomial functions
   Provides ability to calculate Legendre polynomials and 
   their roots.

   Used when calculating Gaussian quadratures.
   */

#include "../polynomial.h"
#include "recursivePolynomial.h"
#include <vector>

namespace rtmath {
	namespace recPolys {
		class legendre : public recPoly {
			public:
				virtual void get(unsigned int rank, polynomial &res) const override;
		};
	}; // end recpolys
};



