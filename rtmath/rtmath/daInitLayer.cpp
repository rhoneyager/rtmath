#include "Stdafx.h"
#include "matrixop.h"
#include "damatrix.h"
#include "daInitLayer.h"
#include "phaseFunc.h"
#include <memory>

namespace rtmath {

	daInitLayer::~daInitLayer()
	{
	}

	daInitLayer::daInitLayer(std::shared_ptr<matrixop> pf, 
		double alb, double tau, rtselec::rtselec rt)
	{
		_ssa = alb;
		_tau = tau;
		_rt = rt;
		_phaseMat = pf;
	}

	std::shared_ptr<matrixop> daInitLayer::eval(const mapid &valmap) const
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
		matrixop pResLayer(2,4,4);
		//shared_ptr<matrixop> res(new matrixop(2,4,4));

		// This function rotates the phasefunction to the desired angle
		// TODO: check the last term (should alpha = mu?)
		rtmath::phaseFuncRotator::rotate(_rt, *_phaseMat, valmap, pRes, valmap.mu);

		// Take the rotated phase function and build a layer
		pResLayer = pRes * _tau * _ssa * (1.0 / 4.0 * abs(valmap.mu) * abs(valmap.mun));

		shared_ptr<matrixop> res(new matrixop(pResLayer));

		// Store result and return
		//_eval_cache[valmap] = res;
		return res;
	}

}; // end rtmath


