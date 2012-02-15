#include "../rtmath/Stdafx.h"
#include <cmath>
#include <memory>
#include "../rtmath/da/daPf.h"


namespace rtmath {
	daMatrixop::daMatrixop()
	{
		// Set up the no-scattering matrix
		std::shared_ptr<matrixop> newpf(new matrixop(matrixop::identity(2,4,4)));
		_src = newpf;
	}

	daMatrixop::daMatrixop(const matrixop &src)
	{
		std::shared_ptr<matrixop> newpf(new matrixop(src));
		_src = newpf;
	}

	daMatrixop::daMatrixop(const std::shared_ptr<const matrixop> &src)
	{
		// Doing it this way because othervise there is no guarantee
		// that src remains constant later. This is a problem because
		// operations referencing this matrix cache their results, 
		// which definitely leads to problems.
		_src = std::shared_ptr<matrixop> (new matrixop(*src));
	}

	std::shared_ptr<const matrixop> daMatrixop::eval(const mapid &) const
	{
		// NO SCATTERING CASE!!!!!
		// Normally, I'd copy the pointer and make it a unique_ptr,
		// or I'd have daMatrixop store _src as a unique_ptr, but
		// I can guarantee that _src will not be changed by any of
		// the other classes. The unique_ptr implementation would
		// necessitate copying and wasting memory and time.
		return _src;
	}

	daPfAlpha::daPfAlpha(std::shared_ptr<phaseFunc> pf)
	{
		_init(pf);
	}

	void daPfAlpha::_init(std::shared_ptr<phaseFunc> pf)
	{
		using namespace std;
		_rt = rtselec::T; // The new default.
		_phaseMat = pf;
		_needsRot = true; // signals that reflection / rotation is needed when evaluating this in layers
		// Initialize the damatrix-derived lhs and rhs.
		// These are from a derived class, so use static_pointer_cast.
		shared_ptr<daPfRotators::daRotator> dlhs(new daPfRotators::daRotator(_rt, daPfRotators::LHS));
		shared_ptr<daPfRotators::daRotator> drhs(new daPfRotators::daRotator(_rt, daPfRotators::RHS));
		_lhs = static_pointer_cast<damatrix>(dlhs);
		_rhs = static_pointer_cast<damatrix>(drhs);
	}

	daPfAlpha::~daPfAlpha()
	{
	}

	std::shared_ptr<const matrixop> daPfAlpha::eval(const mapid &valmap) const
	{
		// First, check to see if this has already been calculated
		// If it is in the cache, return the cached value
		if (_eval_cache_enabled)
			if (_eval_cache.count(valmap) > 0)
				return _eval_cache[valmap];

		// Desired mapid (valmap) is not in the cache, so the matrixop must be calculated
		// Evaluate the damatrix at the necessary values

		// Apply the rotation matrices.
		std::shared_ptr<const matrixop> lhs,rhs,pf;
		lhs = _lhs->eval(valmap);
		rhs = _rhs->eval(valmap);
		
		double alpha = valmap.toAlpha(_rt);

		pf = _phaseMat->eval(alpha); 
		matrixop resb = *pf * *rhs;
		matrixop resa = *lhs * resb;
		std::shared_ptr<const matrixop> res(new matrixop(resa));

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

		std::shared_ptr<const matrixop> daRotator::eval(const mapid &valmap) const
		{
			// First, check to see if this has already been calculated
			// If it is in the cache, return the cached value
			if (_eval_cache.count(valmap) > 0)
				return _eval_cache[valmap];

			// Desired mapid (valmap) is not in the cache, so the matrixop must be calculated
			// Evaluate the damatrix at the necessary values

			// Calculate alpha
			double calpha = valmap.toCosAlpha(_rt); // cos(alpha) has now been found
			if (calpha == 1.0)
			{
				//throw;
			}
			//double alpha = acos(calpha); // not used yet, but soon

			// Create the resultant matrixop
			matrixop res(2,4,4);


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
			res.set(1.0,2,0,0); // res[0][0]
			res.set(1.0,2,3,3); // res[3][3]
			res.set(dcosarg,2,1,1); // res[1][1]
			res.set(dcosarg,2,2,2); // res[2][2]
			res.set(dsinarg,2,2,1); // res[2][1]
			res.set(-1.0*dsinarg,2,2,1); // res[1][2]

			std::shared_ptr<const matrixop> resp(new matrixop(res));
			// Save to the cache
			if (_eval_cache_enabled)
				_eval_cache[valmap] = resp;

			return resp;
		}

	}; // end namespace daPfRotators

	namespace daPfReflections
	{
		daReflection::daReflection(daReflectionEnum spec, std::shared_ptr<rtmath::damatrix> pf)
		{
			_type = spec;
			_source = pf;
		}

		std::shared_ptr<const matrixop> daReflection::eval(const mapid &valmap) const
		{
			// This is just a wrapper function for a basic type of symmetry by reflection.
			// Pr(mu,mun,phi,phin) = P(-mu,mun,phi,phin)
			// Pt(mu,mun,phi,phin) = P(mu,mun,phi,phin)
			// Pr*(mu,mun,phi,phin) = P(mu,-mun,phi,phin)
			// Pt*(mu,mun,phi,phin) = P(-mu,-mun,phi,phin)
			mapid resmap;
			resmap.phi = valmap.phi;
			resmap.phin = valmap.phin;

			switch(_type)
			{
			case T:
				resmap.mu = valmap.mu;
				resmap.mun = valmap.mun;
				break;
			case R:
				resmap.mu = -1.0 * valmap.mu;
				resmap.mun = valmap.mun;
				break;
			case T_STAR:
				resmap.mu = -1.0 * valmap.mu;
				resmap.mun = -1.0 * valmap.mun;
				break;
			case R_STAR:
				resmap.mu = valmap.mu;
				resmap.mun = -1.0 * valmap.mun;
				break;
			default:
				throw rtmath::debug::xBadInput("Unknown type of calculation for daPf");
				break;
			}

			return _source->eval(resmap);
		}

	}; // end namespace daPfReflections

}; // end namespace rtmath


