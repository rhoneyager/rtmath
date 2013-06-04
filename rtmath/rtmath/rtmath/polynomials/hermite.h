#pragma once

/* Hermite polynomial functions
   Provides ability to calculate Hermite polynomials and 
   their roots.

   Used when calculating gaussian quadratures.

   */

#include "recursivePolynomial.h"

namespace rtmath {
	class polynomial;
	namespace recPolys {
		class hermite : public recPoly {
			public:
				virtual void get(unsigned int rank, polynomial &res) const ;
		};
		class hermitePhys : public recPoly {
			public:
				virtual void get(unsigned int rank, polynomial &res) const ;
		};
	}
}


