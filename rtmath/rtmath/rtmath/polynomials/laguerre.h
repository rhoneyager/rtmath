#pragma once

/* Laguerre polynomial functions
   Provides ability to calculate Legendre polynomials and 
   their roots.

   Used when calculating Gaussian quadratures.
   */

#include "recursivePolynomial.h"
#include "../polynomial.h"
#include <vector>

namespace rtmath {
	namespace recPolys {
		class laguerre : public recPoly {
			public:
				virtual void get(unsigned int rank, polynomial &res) const override;
		};
	};
};



