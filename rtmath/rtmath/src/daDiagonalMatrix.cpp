#include "../rtmath/Stdafx.h"
#include "../rtmath/rtmath.h"
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

	std::shared_ptr<matrixop> daDiagonalMatrix::eval(const mapid &valmap) const
	{
		using namespace std;
		HASH_t hash = valmap.hash();
		// _tau should be fixed, and valmap provides the other angles
		
		// Search the cache for existing calculated values
		if (_eval_cache_enabled)
			if (_eval_cache.count(hash) > 0)
				return _eval_cache[hash];

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
		
		shared_ptr<matrixop> res(new matrixop(pRes));

		// Store result and return
		if (_eval_cache_enabled)
			_eval_cache[hash] = res;
		return res;
	}

}; // end rtmath


