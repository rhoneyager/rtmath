#include "Stdafx-core.h"
#include "../rtmath/polynomials/chebyshev.h"
#include "../rtmath/polynomial.h"
#include <vector>

/// Holds the polynomial caches for both types of Chebyshev polynomials
namespace {
	std::vector<rtmath::polynomial> _cacheA, _cacheB;
};

namespace rtmath {

	void recPolys::chebyshevA::get(unsigned int rank, polynomial &res) const
	{
		// Begin by querying legpolys to see if initialized
		if (_cacheA.size() == 0)
		{
			polynomial np;
			// P0(x)
			np.coeff(0,1);
			_cacheA.push_back(np);
			np.erase();
			// P1(x)
			np.coeff(1,1);
			_cacheA.push_back(np);
		}
		// Now, see if the element exists
		if (_cacheA.size() > rank)
		{
			// Polynomial exists. Return.
			res = _cacheA[rank];
			return;
		} else {
			// Some assembly required
			// Use recursion relations to generate polynomials
			// Start at first unknown polynomial and build up
			polynomial x(1,1); // The x operator
			polynomial newpoly;
			for (size_t i=_cacheA.size(); i<=rank ; i++)
			{
				newpoly.erase();
				newpoly = (x * _cacheA[i-1] * 2.0 ) - ( _cacheA[i-2] );
				_cacheA.push_back(newpoly);
			}
			res = _cacheA[rank];
			return;
		}
	}

	void recPolys::chebyshevB::get(unsigned int rank, polynomial &res) const
	{
		// Begin by querying legpolys to see if initialized
		if (_cacheB.size() == 0)
		{
			polynomial np;
			// P0(x)
			np.coeff(0,1);
			_cacheB.push_back(np);
			np.erase();
			// P1(x)
			np.coeff(1,2);
			_cacheB.push_back(np);
		}
		// Now, see if the element exists
		if (_cacheB.size() > rank)
		{
			// Polynomial exists. Return.
			res = _cacheB[rank];
			return;
		} else {
			// Some assembly required
			// Use recursion relations to generate polynomials
			// Start at first unknown polynomial and build up
			polynomial x(1,1); // The x operator
			polynomial newpoly;
			for (size_t i=_cacheB.size(); i<=rank ; i++)
			{
				newpoly.erase();
				newpoly = (x * _cacheB[i-1] * 2.0 ) - ( _cacheB[i-2] );
				_cacheB.push_back(newpoly);
			}
			res = _cacheB[rank];
			return;
		}
	}



} // end namespace rtmath

