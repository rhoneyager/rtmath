#include "Stdafx-core.h"
#include <vector>
#include <set>
#include <map>

#include "../rtmath/zeros.h"
#include "../rtmath/quadrature.h"
#include "../rtmath/polynomial.h"
#include "../rtmath/polynomials/legendre.h"
#include "../rtmath/error/debug.h"
#include "../rtmath/error/error.h"

namespace
{
	std::map<size_t, std::set<rtmath::quadrature::ptWeight> > cached_gauss_lagrange;
}

namespace rtmath {

	namespace quadrature {

		// For Gauss-Legendre quadrature, use recursion relations to
		// get the points and their weights


		// First, define an initialization function
		// And a series of vectors or doubles which 
		// hold the zeros and weights.

		// For my programmatic convenience, let's use a basic array that can be parsed later
		const double _gaussian_lagrange_prepump[] = {
			// deg, zero,           weight
			1,  0.0,			2,
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

		double quad_eval_leg(double a, double b, unsigned int degree, const std::function<double(double) > &f)
		{
			// Assumes legendre polynomials
			// Use the appropriate quadrature and weights (precomputed mostly!)
			if (degree <= 1) 
				throw rtmath::debug::xBadInput("Bad quadrature degree");

			/*
			// For each n degree, expect n quadrature points
			// Start point in array is 2 + 3 + 4 + ... + n-1
			// So, the starting index is 3 * 
			unsigned int start = 3 * (unsigned int) ( ( (degree * degree) - degree) / 2);
			//unsigned int start = 3 * (unsigned int) (-1.0 * (double) degree / 2.0 + (double) degree * (double) degree / 2.0);
			// Check if the start even exists! If not, generate the necessary points.
			// TODO: add code. For now, RTthrow if insufficient points
			if (degree > 7) RTthrow rtmath::debug::xBadInput("Bad quadrature degree");
			*/

//void getGaussLegPtsStd(size_t deg, std::set<ptWeight>& p)
			std::set<ptWeight> p;
			getQuadPtsLeg(degree, p);

			// Do the evaluation and summation
			double res = 0.0;

			for (auto it = p.begin(); it != p.end(); ++it)
			{
				double x = it->first;
				double w = it->second;
				res += f( ((b-a)/2.0 * x) + ((a+b)/2.0) ) * w;
			}
			/*
			for (unsigned int i = start; i < start + 3 * degree; i+=3)
			{
				res += f( ((b-a)/2.0* _gaussian_lagrange[i+1]) + ((a+b)/2.0) ) * _gaussian_lagrange[i+2];
			}
			*/
			res *= (b-a)/2.0;
			return res;
		}

		/*
		void getQuadPtsLeg(size_t degree, std::set<ptWeight> &p, double min, double max)
		{
			p.clear();
			unsigned int start = 3 * (unsigned int) ( ( (degree * degree) - degree) / 2);
			if (degree > 7) RTthrow rtmath::debug::xBadInput("Bad quadrature degree");
			for (unsigned int i = start; i < start + 3 * degree; i+=3)
				pts.insert(_gaussian_lagrange[i+1]);
		}
		*/

		void getQuadPtsLeg(size_t deg, std::set<ptWeight>& p)
		{
			p.clear();

			if (cached_gauss_lagrange.count(deg))
			{
				p = cached_gauss_lagrange.at(deg);
				return;
			}

			// The degree of the polynomial corresponds to the number of zeros
			// And the zeros are symmetrix about x=0. So, I just need to 
			// find the zeros of the function in the range [0,1].
			rtmath::recPolys::legendre lpgen; // TODO: rewrite the generators
			rtmath::polynomial lp, dlp;
			lpgen.get((unsigned int) deg, lp);
			dlp = lp.deriv(1); // dlp = P'

			//lp.print();
			//dlp.print();

			// This function finds the location of a zero
			// of the legendre polynomial. Recursive division is performed to 
			// extract multiple zeros
			std::multiset<double> known_zeros;
			auto f = [&](double x) -> double
			{
				double val = lp.eval(x);
				for (const double &kz : known_zeros)
					val /= (x-kz);
				return val;
			};

			for (size_t i=0; i<deg; i++)
			{
				// The bounds vary depending on whether the polynomial is even or odd
				// The polynomial state depends on how many zeros have been extracted
				double a = -1.0;
				double b = 1.0;
				// If f(a)*f(b) > 0, then they have the same sign. 
				// This will cause the zero-checker to fail.
				// So, repeatedly subdivide the interval until a zero is found.
				// This doesn't have to be too complex, given the legendre polynomial properties.
				/*
				while (f(a) * f(b) > 0)
				{
					double span = b-a;
					span /= 2.0;
					double c = b - span;
					double fc = f(c);
				}
				*/
				if ((deg + known_zeros.size()) % 2 == 0)
				{
					a = 0.2;
					/*
					if (known_zeros.size() % 2 == 0)
					{ // Search the right or left-hand side
						a = 0.2; // rhs
					} else {
						// Should never hit lhs due to code below.
						// Just included for completeness with an implementation of
						// other polynomial types
						//b = 0; // lhs
						a = 0.2;
					}
					*/
				}
				// Brent's rule has trouble with initial conditions
				// The bounds must have opposite signs, and these are hard to locate.
				//double z = rtmath::zeros::findzero(a,b,f);
				// Using the secant method instead
				double z = rtmath::zeros::secantMethod(f, a, b);
				// It is convenient that the legendre polynomial zeros are 
				// symmetric about the x-axis
				known_zeros.insert(z);
				if ( abs(z) > 0.000001)
				{
					known_zeros.insert(-z);
					i++;
				}
			}

			// And calculate the weights
			for (const double& z : known_zeros)
			{
				// dpnz is P'_n(z)
				double dpnz = dlp.eval(z);
				double w = 2.0 / ( (1.0 - (z*z)) * pow(dpnz,2.0));;
				p.insert(ptWeight(z,w));
			}

			cached_gauss_lagrange[deg] = p;
		}

	}
}

