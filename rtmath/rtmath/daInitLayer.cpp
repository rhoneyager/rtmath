#include "Stdafx.h"
#include "matrixop.h"
#include "damatrix.h"
#include "daInitLayer.h"
#include "daPf.h"
#include "phaseFunc.h"
#include <memory>

namespace rtmath {

	daInitLayer::~daInitLayer()
	{
	}

	void daInitLayer::_init(std::shared_ptr<damatrix> pf, 
		double alb, double tau, rtselec::rtselec rt)
	{
		_ssa = alb;
		_tau = tau;
		_rt = rt;
		using namespace std;
		//shared_ptr<daPf> rotPF(new daPf(rt,pf));
		//_phaseMatRot = static_pointer_cast<damatrix>(rotPF);
		//_pf = pf; // It is now the user's responsibility to declare a rotator.
		// If the damatrix specifies that a rotator is needed, provide it
		if (pf->needsRot() == true)
		{
			using namespace rtmath::daPfReflections;
			//_pf = static_pointer_case<damatrix>( shared_ptr<daPfAlpha> (new daPfAlpha(pf)));
			shared_ptr<daReflection> refl;
			if (rt == T)
			{
				refl = shared_ptr<daReflection> (new daReflection(T, pf));
			} else {
				refl = shared_ptr<daReflection> (new daReflection(R, pf));
			}
			_pf = static_pointer_cast<damatrix> (refl); // This will handle the reflection aspect of the layer.

		} else {
			_pf = pf; // No reflection necessary. Use raw pf.
		}
	}

	daInitLayer::daInitLayer(std::shared_ptr<damatrix> pf, 
		double alb, double tau, rtselec::rtselec rt)
	{
		_init(pf,alb,tau,rt);
	}

	std::shared_ptr<matrixop> daInitLayer::eval(const mapid &valmap) const
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

		// This function rotates the phasefunction to the desired angle
		// TODO: check the last term (should alpha = mu?)
		//rtmath::phaseFuncRotator::rotate(_rt, *_phaseMat, valmap, pRes, valmap.mu);
		// _pf now should already handle rotation.
		std::shared_ptr<matrixop> pf(_pf->eval(valmap));

		// Take the rotated phase function and build a layer
		pRes = *pf * _tau * _ssa * (1.0 / 4.0 * abs(valmap.mu) * abs(valmap.mun));

		shared_ptr<matrixop> res(new matrixop(pRes));

		// Store result and return
		if (_eval_cache_enabled) 
			_eval_cache[hash] = res;
		return res;
	}

}; // end rtmath


