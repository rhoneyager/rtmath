#pragma once

/* Chebyshev polynomial functions
   Provides ability to calculate Chebyshev polynomials and 
   their roots.

   Used when calculating Clenshaw-Curtis quadratures.

TODO: combine the two Cheby defs, and create a class for the recursion relations
This way, relation code can be automatically handled
   */

#include "recursivePolynomial.h"

namespace rtmath {
	class polynomial;
	namespace recPolys {
		class chebyshevA : public recPoly {
			public:
				virtual void get(unsigned int rank, polynomial &res) const ;
		};
		class chebyshevB : public recPoly {
			public:
				virtual void get(unsigned int rank, polynomial &res) const ;
		};
	}
}



