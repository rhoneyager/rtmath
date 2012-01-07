#include "../rtmath/Stdafx.h"
#include "../rtmath/rtmath.h"
#include <vector>

namespace rtmath {
	std::vector<polynomial> recPolys::hermite::_cache;
	std::vector<polynomial> recPolys::hermitePhys::_cache;

	void recPolys::hermite::get(unsigned int rank, polynomial &res) const
	{
		// Begin by querying legpolys to see if initialized
		if (_cache.size() == 0)
		{
			polynomial np;
			// P0(x)
			np.coeff(0,1);
			_cache.push_back(np);
			np.erase();
			// P1(x)
			np.coeff(1,1);
			_cache.push_back(np);
		}
		// Now, see if the element exists
		if (_cache.size() > rank)
		{
			// Polynomial exists. Return.
			res = _cache[rank];
			return;
		} else {
			// Some assembly required
			// Use recursion relations to generate polynomials
			// Start at first unknown polynomial and build up
			polynomial x(1,1); // The x operator
			polynomial newpoly;
			for (unsigned int i=_cache.size(); i<=rank ; i++)
			{
				newpoly.erase();
				newpoly = (x * _cache[i-1] ) - ( _cache[i-2] * (i-1));
				_cache.push_back(newpoly);
			}
			res = _cache[rank];
			return;
		}
	}

	void recPolys::hermitePhys::get(unsigned int rank, polynomial &res) const
	{
		// Begin by querying legpolys to see if initialized
		if (_cache.size() == 0)
		{
			polynomial np;
			// P0(x)
			np.coeff(0,1);
			_cache.push_back(np);
			np.erase();
			// P1(x)
			np.coeff(1,2);
			_cache.push_back(np);
		}
		// Now, see if the element exists
		if (_cache.size() > rank)
		{
			// Polynomial exists. Return.
			res = _cache[rank];
			return;
		} else {
			// Some assembly required
			// Use recursion relations to generate polynomials
			// Start at first unknown polynomial and build up
			polynomial x(1,1); // The x operator
			polynomial newpoly;
			for (unsigned int i=_cache.size(); i<=rank ; i++)
			{
				newpoly.erase();
				newpoly = (x * _cache[i-1] * 2.0 ) - ( _cache[i-2] * (2.0*i-2.0) );
				_cache.push_back(newpoly);
			}
			res = _cache[rank];
			return;
		}
	}



}; // end namespace rtmath

