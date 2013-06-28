#pragma once
#include "recursivePolynomial.h"

namespace rtmath {
	class polynomial;
	namespace recPolys {
		/// \todo Combine the two Cheby defs, and create a class for the recursion relations
		/// This way, relation code can be automatically handled

		/** \brief Chebyshev polynomial function A

		Provides ability to calculate Chebyshev polynomials and 
		their roots.

		Used when calculating Clenshaw-Curtis quadratures.
		**/
		class chebyshevA : public recPoly {
		public:
			virtual void get(unsigned int rank, polynomial &res) const ;
		};
		/** \brief Chebyshev polynomial function B

		Provides ability to calculate Chebyshev polynomials and 
		their roots.

		Used when calculating Clenshaw-Curtis quadratures.
		**/
		class chebyshevB : public recPoly {
		public:
			virtual void get(unsigned int rank, polynomial &res) const ;
		};
	}
}



