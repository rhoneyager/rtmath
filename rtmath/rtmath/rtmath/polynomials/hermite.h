#pragma once

#include "recursivePolynomial.h"

namespace rtmath {
	class polynomial;
	namespace recPolys {
		/** \brief Hermite polynomial function

		Provides ability to calculate Hermite polynomials and 
		their roots.

		Used when calculating Gaussian quadratures.

		\f[
		He_n(x) = (-1)^n e^{\frac{x^2}{2}} \frac{d^n}{dx^n} e^\frac{-x^2}{2}
		\f]
		**/
		class DLEXPORT_rtmath_core hermite : public recPoly {
		public:
			virtual void get(unsigned int rank, polynomial &res) const ;
		};
		/** \brief Hermite polynomial function

		Provides ability to calculate Hermite polynomials and 
		their roots.

		Used when calculating Gaussian quadratures.

		\f[
		H_n(x) = (-1)^n e^{x^2} \frac{d^n}{dx^n} e^{-x^2}
		\f]
		**/
		class DLEXPORT_rtmath_core hermitePhys : public recPoly {
		public:
			virtual void get(unsigned int rank, polynomial &res) const ;
		};
	}
}


