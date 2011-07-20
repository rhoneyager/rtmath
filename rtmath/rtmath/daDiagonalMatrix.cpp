#include "Stdafx.h"
#include "matrixop.h"
#include "damatrix.h"
#include "daDiagonalMatrix.h"
#include "phaseFunc.h"
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
		// _tau should be fixed, and valmap provides the other angles
		
		// Search the cache for existing calculated values
		if (_eval_cache.count(valmap) > 0)
		{
			return _eval_cache[valmap];
		}

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
		_eval_cache[valmap] = res;
		return res;
	}

}; // end rtmath


