#include "../rtmath/Stdafx.h"
#include <memory>
#include <math.h>
#include "../rtmath/da/damatrix.h"


namespace rtmath {

	damatrix::~damatrix()
	{
		// No need to do anything!
	}

	damatrix::damatrix()
	{
		// Only called by itself internally, when producing a damatrix that 
		// is the result of op(...).
		// Also called by the other constructors, to put any initializations
		// in the right place.

		// Default values: (probably will be overridden)
		__init();
	}

	damatrix::damatrix(const std::shared_ptr<damatrix> lhs, 
			const std::shared_ptr<damatrix> rhs, daOp oper)
	{
		if (oper == NONE) throw rtmath::debug::xBadInput("No damatrix operation specified.");
		if (oper == ADD || oper == MULTNORMAL || oper == MULT)
			if (rhs == nullptr)
				throw rtmath::debug::xBadInput("Need rhs to be a valid damatrix.");
		if (oper == POW)
			throw debug::xBadInput("Inappropriate constructor called.");
		__init();
		_parentOp = oper;
		_rootA = lhs;
		_rootB = rhs;

	}

	void damatrix::__init()
	{
		_pow = 0;
		_needsRot = false;
		_parentOp = NONE;
		_eval_cache_enabled = true;
	}

	std::shared_ptr<damatrix> damatrix::pow(const std::shared_ptr<damatrix> &base, unsigned int power)
	{
		// Use the private constructor here
		std::shared_ptr<damatrix> res(new damatrix());
		res->_pow = power;
		res->_rootA = base;
		res->_parentOp = POW;
		return res;
	}

	std::shared_ptr<damatrix> damatrix::op(std::shared_ptr<damatrix> lhs, 
		std::shared_ptr<damatrix> rhs, daOp oper)
	{
		std::shared_ptr<damatrix> res(new damatrix(lhs,rhs,oper));
		return res;
	}

	std::shared_ptr<const matrixop> damatrix::eval(const mapid &valmap) const
	{
		const double M_PI = boost::math::constants::pi<double>();
		// First, check to see if this has already been calculated
		// If it is in the cache, return the cached value
		if (_eval_cache_enabled) 
			if (_eval_cache.count(valmap) > 0)
				return _eval_cache[valmap];
		
		// Desired mapid (valmap) is not in the cache, so the matrixop must be calculated
		// Evaluate the damatrix at the necessary values

		// Partial result holders, for convenience (no messy shared_ptrs)
		matrixop pRes(2,4,4);
		matrixop aRes(2,4,4);
		matrixop bRes(2,4,4);
		std::shared_ptr<const matrixop> aResS, bResS;

		// Operations are based on _parentOp's option
		switch (_parentOp)
		{
		case NONE:
			// The easiest case
			throw rtmath::debug::xBadInput("Evaluating, but no damatrix operation specified.");
			break;
		case ADD:
			// Direct addition
			aResS = _rootA->eval(valmap);
			bResS = _rootB->eval(valmap);
			pRes = *aResS + *bResS;
			break;
		case INV:
			// Evaluate _rootA and take the inverse
			aResS = _rootA->eval(valmap);
			pRes = aResS->inverse();
			break;
		case MULTNORMAL:
			aResS = _rootA->eval(valmap);
			bResS = _rootB->eval(valmap);
			pRes = *aResS * *bResS;
			break;
		case MULT:
			// Hardest, as integration by quadrature must occur
			{	// Braces matter, as I'm initializing a new shared_ptr
				// Otherwise, it would be in the switch scope, which would fail
				std::shared_ptr<const matrixop> a = daint::outer_int(valmap,_rootA,_rootB);
				pRes = *a * matrixop::diagonal(1.0/M_PI,2,4,4);
			}
			break;
		case POW:
			// Really just successive multiplication for now
			{
				pRes = matrixop::identity(2,4,4);
				// Loop through power
				if (_pow)
				{
					// Evaluate _rootA and repeatedly multiply
					// TODO: use better multiplication scheme
					aResS = _rootA->eval(valmap);
					for (unsigned int i=0;i<_pow;i++)
					{
						pRes = pRes * *aResS;
					}
				}
			}
			break;
		default:
			throw rtmath::debug::xBadInput("No damatrix operation specified.");
			break;
		};
		
		// Create the resultant matrixop
		std::shared_ptr<const matrixop> res(new matrixop(pRes));

		// Cache the calculated value, for later calculations
		if (_eval_cache_enabled)
			_eval_cache[valmap] = res;

		return res;
	}

}; // end namespace rtmath


