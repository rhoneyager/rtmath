#include "../rtmath/Stdafx.h"
#include "../rtmath/da/daDiagonalMatrix.h"
#include <memory>

namespace rtmath {

	daDiagonalMatrix::~daDiagonalMatrix()
	{
	}

	daDiagonalMatrix::daDiagonalMatrix(double tau, 
		valmap_selector::valmap_selector mumun)
	{
		_tau = tau;
		_mumun = mumun;
	}

	std::shared_ptr<const matrixop> daDiagonalMatrix::eval(const mapid &valmap) const
	{
		using namespace std;
		// _tau should be fixed, and valmap provides the other angles
		
		// Search the cache for existing calculated values
		if (_eval_cache_enabled)
			if (_eval_cache.count(valmap) > 0)
				return _eval_cache[valmap];

		// Existing value not found. Must calculate.
		matrixop pRes(2,4,4);
		
		switch(_mumun)
		{
		case valmap_selector::MU:
			pRes = matrixop::diagonal(exp(-1.0*_tau / valmap.mu),2,4,4);
			break;
		case valmap_selector::MUN:
			pRes = matrixop::diagonal(exp(-1.0*_tau / valmap.mun),2,4,4);
			break;
		};
		
		shared_ptr<const matrixop> res(new matrixop(pRes));

		// Store result and return
		if (_eval_cache_enabled)
			_eval_cache[valmap] = res;
		return res;
	}

}; // end rtmath


