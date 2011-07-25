#include "Stdafx.h"
#include "daPf.h"
#include "matrixop.h"
#include "damatrix.h"
#include "enums.h"
#include <cmath>
#include <memory>

namespace rtmath {
	daPf::daPf(rtselec::rtselec RT, std::shared_ptr<damatrix> pf)
	{
		_init(RT, pf);
	}

	daPf::daPf(rtselec::rtselec RT, std::shared_ptr<matrixop> pf)
	{
		std::shared_ptr<damatrix> damatrixPf(new damatrix(*pf));
		_init(RT, damatrixPf);
	}

	void daPf::_init(rtselec::rtselec RT, std::shared_ptr<damatrix> pf)
	{
		using namespace std;
		_rt = RT;
		_phaseMat = pf;
		// Initialize the damatrix-derived lhs and rhs.
		// These are from a derived class, so use static_pointer_cast.
		shared_ptr<daPfRotators::daRotator> dlhs(new daPfRotators::daRotator(RT, daPfRotators::LHS));
		shared_ptr<daPfRotators::daRotator> drhs(new daPfRotators::daRotator(RT, daPfRotators::RHS));
		_lhs = static_pointer_cast<damatrix>(dlhs);
		_rhs = static_pointer_cast<damatrix>(drhs);
	}

	daPf::~daPf()
	{
	}

	std::shared_ptr<matrixop> daPf::eval(const mapid &valmap) const
	{
		// First, check to see if this has already been calculated
		// If it is in the cache, return the cached value

		if (_eval_cache.count(valmap) > 0)
		{
			return _eval_cache[valmap];
		}

		// Check lock condition
		if (_locked && _parentOp != NONE) throw rtmath::debug::xLockedNotInCache();

		// Desired mapid (valmap) is not in the cache, so the matrixop must be calculated
		// Evaluate the damatrix at the necessary values

		// Apply the rotation matrices.
		std::shared_ptr<matrixop> lhs,rhs,pf;
		lhs = _lhs->eval(valmap);
		rhs = _rhs->eval(valmap);
		pf = _phaseMat->eval(valmap); // TODO: also allow for case where P(alpha)
		matrixop resb = *pf * *rhs;
		matrixop resa = *lhs * resb;
		std::shared_ptr<matrixop> res(new matrixop(resa));

		// Save to the cache
		if (_eval_cache_enabled)
			_eval_cache[valmap] = res;

		return res;
	}


	namespace daPfRotators {

		daRotator::daRotator(rtselec::rtselec RT, daRotatorLR LR)
		{
			_rt = RT;
			_lr = LR;
		}

		std::shared_ptr<matrixop> daRotator::eval(const mapid &valmap) const
		{
			// First, check to see if this has already been calculated
			// If it is in the cache, return the cached value

			if (_eval_cache.count(valmap) > 0)
			{
				return _eval_cache[valmap];
			}

			// Check lock condition
			if (_locked && _parentOp != NONE) throw rtmath::debug::xLockedNotInCache();

			// Desired mapid (valmap) is not in the cache, so the matrixop must be calculated
			// Evaluate the damatrix at the necessary values

			// Calculate alpha
			double calpha;
			calpha = sqrt(1.0 - (valmap.mu * valmap.mu)) * sqrt(1.0 - (valmap.mun * valmap.mun)) * cos(valmap.phi - valmap.phin);
			switch (_rt)
			{
			case rtselec::R:
				calpha -= valmap.mu*valmap.mun;
				break;
			case rtselec::T:
				calpha += valmap.mu*valmap.mun;
				break;
			} // cos(alpha) has now been found

			double alpha = acos(calpha); // not used yet, but soon

			// Create the resultant matrixop
			std::shared_ptr<matrixop> res(new matrixop(2,4,4));


			// Set cosarg (see Hansen 1971)
			double cosarg = 0;
			// Set the matrixop values depending on _rt and _lr
			switch (_lr)
			{
			case RHS: // want cos(i1)
				switch (_rt)
				{
				case rtselec::R:
					cosarg = -1.0*valmap.mun*cos(valmap.phi-valmap.phin)*sqrt(1.0-(valmap.mu*valmap.mu));
					cosarg -= valmap.mu * sqrt(1.0 - (valmap.mun*valmap.mun));
					cosarg /= sqrt(1.0-(calpha*calpha));
					break;
				case rtselec::T:
					cosarg = -1.0*valmap.mun*cos(valmap.phi-valmap.phin)*sqrt(1.0-(valmap.mu*valmap.mu));
					cosarg += valmap.mu * sqrt(1.0 - (valmap.mun*valmap.mun));
					cosarg /= sqrt(1.0-(calpha*calpha));
					break;
				}
				break;
			case LHS: // want cos(i2)
				switch (_rt)
				{
				case rtselec::R:
					cosarg = valmap.mun*sqrt(1.0-(valmap.mu*valmap.mu));
					cosarg += valmap.mu * sqrt(1.0 - (valmap.mun*valmap.mun)) * cos(valmap.phi-valmap.phin);
					cosarg /= sqrt(1.0-(calpha*calpha));
					break;
				case rtselec::T:
					cosarg = valmap.mun*sqrt(1.0-(valmap.mu*valmap.mu));
					cosarg -= valmap.mu * sqrt(1.0 - (valmap.mun*valmap.mun)) * cos(valmap.phi-valmap.phin);
					cosarg /= sqrt(1.0-(calpha*calpha));
					break;
				}
				break;
			}

			double arg = acos(cosarg); // cos^-1(cos(i1 | i2))
			double dcosarg = cos(2.0 * arg); // Cosine(2*arg)
			double dsinarg = sin(2.0 * arg); // Cosine(2*arg)

			// Can finally construct the matrix
			res->set(1.0,2,0,0); // res[0][0]
			res->set(1.0,2,3,3); // res[3][3]
			res->set(dcosarg,2,1,1); // res[1][1]
			res->set(dcosarg,2,2,2); // res[2][2]
			res->set(dsinarg,2,2,1); // res[2][1]
			res->set(-1.0*dsinarg,2,2,1); // res[1][2]

			// Save to the cache
			if (_eval_cache_enabled)
				_eval_cache[valmap] = res;

			return res;
		}

	}; // end namespace daPfRotators

}; // end namespace rtmath


