#pragma once

/* Hermite polynomial functions
   Provides ability to calculate Hermite polynomials and 
   their roots.

   Used when calculating gaussian quadratures.

   */

#include "recursivePolynomial.h"
#include "../polynomial.h"
#include <vector>

namespace rtmath {
	namespace recPolys {
		class hermite : public recPoly {
			public:
				virtual void get(unsigned int rank, polynomial &res) const override;
		};
		class hermitePhys : public recPoly {
			public:
				virtual void get(unsigned int rank, polynomial &res) const override;
		};
	};
};


