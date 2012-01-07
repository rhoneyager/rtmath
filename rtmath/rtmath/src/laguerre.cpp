#include "../rtmath/Stdafx.h"
#include "../rtmath/rtmath.h"
#include <vector>

namespace rtmath {
	std::vector<polynomial> recPolys::laguerre::_cache;

	void recPolys::laguerre::get(unsigned int rank, polynomial &res) const
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
			np.coeff(0,1);
			np.coeff(1,-1);
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
			polynomial xop; // The x operator
			xop.coeff(1,1);
			polynomial newpoly;
			for (unsigned int i=_cache.size(); i<=rank ; i++)
			{
				newpoly.erase();
				newpoly = ( _cache[i-1] * (xop * -1.0 + (2.0*i) - 1.0) * (1.0/i) ) - (( _cache[i-2] * (i-1.0) ) * (1.0/i) );
//				newpoly = (xop * _cache[i-1] * (2.0 - 1.0/i)) - ( _cache[i-2] * (1.0 - 1.0/i) );
				_cache.push_back(newpoly);
			}
			res = _cache[rank];
			return;
		}
	}
}; // end namespace rtmath

