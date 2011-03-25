#include <vector>
#include <set>
#include <map>
#include "quadrature.h"

namespace rtmath {
	evalfunction::evalfunction()
	{return;}
	evalfunction::~evalfunction()
	{return;}

	namespace quadrature {
// First, define an initialization function
// And a series of vectors or doubles which 
// hold the zeros and weights.

// For my programmatic convenience, let's use a basic array that can be parsed later
		const double _gaussian_lagrange_prepump[] = {
			2,	0.577350269,	1,
			2,	-0.577350269,	1,
			3,	0.0,			0.88888889,
			3,	0.77459667,		0.55555555,
			3,	-0.77459667,	0.55555555,
			4,	0.33998104,		0.65214515,
			4,	-0.33998104,	0.65214515,
			4,	0.86113631,		0.34785485,
			4,	-0.86113631,	0.34785485,
			5,	0.0,			0.56888889,
			5,	0.53846931,		0.47862867,
			5,	-0.53846931,	0.47862867,
			5,	0.90617985,		0.23692689,
			5,	-0.90617985,	0.23692689,
			6,	0.23861918,		0.46791393,
			6,	-0.23861918,	0.46791393,
			6,	0.66120939,		0.36076157,
			6,	-0.66120939,	0.36076157,
			6,	0.93246951,		0.17132449,
			6,	-0.93246951,	0.17132449,
			7,	0.0,			0.41795918,
			7,	0.40584515,		0.38183005,
			7,	-0.40584515,	0.38183005,
			7,	0.74153119,		0.27970539,
			7,	-0.74153119,	0.27970539,
			7,	0.94910791,		0.12948497,
			7,	-0.94910791,	0.12948497,
			0.0
		};

		// I want these routines to be __fast__
		// Upon run, look and see if degree is part 7
		// If so, will have to extand array and calculate manually
		// TODO: do this

		const double *_gaussian_lagrange = _gaussian_lagrange_prepump;

		double quad_eval_leg(double a, double b, unsigned int degree, evalfunction *f)
		{
			// Assumes legendre polynomials
			// Use the appropriate quadrature and weights (precomputed mostly!)
			double res = (b - a) / 2.0;
			if (degree <= 1) throw;
			// For each n degree, expect n quadrature points
			// Start point in array is 2 + 3 + 4 + ... + n-1
			// So, the starting index is 3 * 

			// Do the necessary evaluation and summation
			for (unsigned int i=1; i <= degree; i++)
			{
			}
			return res;
		}

	}; // end namespace quadrature
}; // end namespace rtmath

