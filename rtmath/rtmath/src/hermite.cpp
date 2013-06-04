#include "../rtmath/Stdafx.h"
#include "../rtmath/polynomial.h"
#include "../rtmath/polynomials/hermite.h"
#include <vector>

namespace {
	std::vector<rtmath::polynomial> _cacheH, _cacheHP;
};

namespace rtmath {

	void recPolys::hermite::get(unsigned int rank, polynomial &res) const
	{
		// Begin by querying legpolys to see if initialized
		if (_cacheH.size() == 0)
		{
			polynomial np;
			// P0(x)
			np.coeff(0,1);
			_cacheH.push_back(np);
			np.erase();
			// P1(x)
			np.coeff(1,1);
			_cacheH.push_back(np);
		}
		// Now, see if the element exists
		if (_cacheH.size() > rank)
		{
			// Polynomial exists. Return.
			res = _cacheH[rank];
			return;
		} else {
			// Some assembly required
			// Use recursion relations to generate polynomials
			// Start at first unknown polynomial and build up
			polynomial x(1,1); // The x operator
			polynomial newpoly;
			for (unsigned int i=_cacheH.size(); i<=rank ; i++)
			{
				newpoly.erase();
				newpoly = (x * _cacheH[i-1] ) - ( _cacheH[i-2] * (i-1));
				_cacheH.push_back(newpoly);
			}
			res = _cacheH[rank];
			return;
		}
	}

	void recPolys::hermitePhys::get(unsigned int rank, polynomial &res) const
	{
		// Begin by querying legpolys to see if initialized
		if (_cacheHP.size() == 0)
		{
			polynomial np;
			// P0(x)
			np.coeff(0,1);
			_cacheHP.push_back(np);
			np.erase();
			// P1(x)
			np.coeff(1,2);
			_cacheHP.push_back(np);
		}
		// Now, see if the element exists
		if (_cacheHP.size() > rank)
		{
			// Polynomial exists. Return.
			res = _cacheHP[rank];
			return;
		} else {
			// Some assembly required
			// Use recursion relations to generate polynomials
			// Start at first unknown polynomial and build up
			polynomial x(1,1); // The x operator
			polynomial newpoly;
			for (unsigned int i=_cacheHP.size(); i<=rank ; i++)
			{
				newpoly.erase();
				newpoly = (x * _cacheHP[i-1] * 2.0 ) - ( _cacheHP[i-2] * (2.0*i-2.0) );
				_cacheHP.push_back(newpoly);
			}
			res = _cacheHP[rank];
			return;
		}
	}



} // end namespace rtmath

