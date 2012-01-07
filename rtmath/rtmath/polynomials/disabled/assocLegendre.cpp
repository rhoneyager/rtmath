#include "assocLegendre.h"
#include <cmath>
namespace rtmath {

	legendre assocLegendre::_legparts;

	double assocLegendre::eval(double x, unsigned int l, int m)
	{
		// First check that the legendre polynomial exists
		rtmath::polynomial *pl = _legparts.calcPoly(l);
		// Take the mth deriv
		rtmath::polynomial deriv = pl->deriv(m);
		// Evaluate at x

		double res = 1;
		// Check if m is odd. If odd, mult by -1
		if (2 * (m / 2) != m) res *= -1;
		// Mult by the sqrt factor
		rtmath::polynomial xop(1,1.0);
		rtmath::polynomial powfact = (xop ^ 2) * -1.0 + 1.0;
		res *= pow(powfact.eval(x), ( (double) m) / 2.0);
		// Mult by the legendre polynomial deriv evaled at x
		res *= deriv.eval(x);

		return res;
	}
};
