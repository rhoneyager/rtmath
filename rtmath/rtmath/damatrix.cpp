#include "Stdafx.h"
#include "damatrix.h"
#include "matrixop.h"
#include "error.h"
#include "common_templates.h"
#include <memory>
#include "damatrix_quad.h"
#define _USE_MATH_DEFINES
#include <math.h>

namespace rtmath {

	damatrix::~damatrix()
	{
		// No need to do anything!
	}

	damatrix::damatrix(const matrixop &source)
	{
		__init();

		// _parentOp remains NONE as this provides a base

		// Clone source and make a shared_ptr to the cloned copy
		std::shared_ptr<matrixop> newprovider(new matrixop(source));
		_provider = newprovider; 
	}

	damatrix::damatrix(const damatrix &rhs)
	{
		__init();

		// Clone things from rhs
		// Shared_ptrs still link to the same objects, but unique_ptrs should be cloned
		_pow = rhs._pow;
		_parentOp = rhs._parentOp;
		_rootA = rhs._rootA;
		_rootB = rhs._rootB;
		_provider = rhs._provider;
		_needsrot = false;
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

	void damatrix::__init()
	{
		_pow = 0;
		_parentOp = NONE;
		_eval_cache_enabled = false;
		_locked = false;
	}

	std::shared_ptr<damatrix> damatrix::operator* (const std::shared_ptr<damatrix> rhs) const
	{
		// Unfortunately necessary to not use *this directly, as the deallocation 
		// of the operator frees this unpredictably. I have no other shared_ptr 
		// for reference.
		std::shared_ptr<damatrix> lhs(new damatrix(*this)); // Clone me
		return op(lhs,rhs,MULT);
	}

	std::shared_ptr<damatrix> damatrix::operator* (double rhs) const
	{
		// This is really just an alias that creates a damatrix out of the 
		// diagonal 4x4 matrix produced by rhs, then performs the multiplication
		std::shared_ptr<damatrix> lhs(new damatrix(*this)); // Clone me
		matrixop rhmat = matrixop::diagonal(rhs,2,4,4);		// Turn rhs into a diagonal matrix
		std::shared_ptr<damatrix> rhDA(new damatrix(rhmat));// Create a damatrix out of the matrixop
		return op(lhs,rhDA,MULT); // Do the standard multiplication
	}

	std::shared_ptr<damatrix> damatrix::operator+ (const std::shared_ptr<damatrix> rhs) const
	{
		std::shared_ptr<damatrix> lhs(new damatrix(*this)); // Clone me
		return op(lhs,rhs,ADD);
	}

	std::shared_ptr<damatrix> damatrix::inverse() const
	{
		std::shared_ptr<damatrix> lhs(new damatrix(*this)); // Clone me
		return op(lhs,lhs,INV); // rhs is not used, so just use a second lhs copy
	}

	std::shared_ptr<damatrix> damatrix::op(std::shared_ptr<damatrix> lhs, 
		std::shared_ptr<damatrix> rhs, daOp oper)
	{
		// This is the general function used for defining a new damatrix as the result 
		// of the operation of two parent damatrices.
		// It does not evaluate the result. eval(mapid) does that.

		if (oper == NONE) throw rtmath::debug::xBadInput();

		// Use the private constructor here
		std::shared_ptr<damatrix> res(new damatrix());
		// But, override some of the values to create a valid object
		res->_parentOp = oper;
		res->_rootA = lhs;
		res->_rootB = rhs;

		return res;
	}

	void damatrix::lock()
	{
		// Locks the damatrix and drops the parents.
		// Used to save memory
		_locked = true;
		_rootA.reset();
		_rootB.reset();
	}

	std::shared_ptr<matrixop> damatrix::eval(const mapid &valmap) const
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

		// Create the resultant matrixop
		std::shared_ptr<matrixop> res(new matrixop(2,4,4));

		// Partial result holders, for convenience (no messy shared_ptrs)
		matrixop pRes(2,4,4);
		matrixop aRes(2,4,4);
		matrixop bRes(2,4,4);
		std::shared_ptr<matrixop> aResS, bResS;

		// Operations are based on _parentOp's option
		bool usePRes = false; // If set to true, convert pRes to a shared pointer and save to res
		switch (_parentOp)
		{
		case NONE:
			// The easiest case
			res = _provider;
			break;
		case ADD:
			// Direct addition
			aResS = _rootA->eval(valmap);
			bResS = _rootB->eval(valmap);
			pRes = *aResS + *bResS;
			usePRes = true;
			break;
		case INV:
			// Evaluate _rootA and take the inverse
			aResS = _rootA->eval(valmap);
			pRes = aResS->inverse();
			usePRes = true;
			break;
		case MULTNORMAL:
			aResS = _rootA->eval(valmap);
			bResS = _rootB->eval(valmap);
			pRes = *aResS * *bResS;
			usePRes = true;
			break;
		case MULT:
			// Hardest, as integration by quadrature must occur
			res = daint::outer_int(valmap,_rootA,_rootB);
			pRes = *res * matrixop::diagonal(1.0/M_PI,2,4,4);
			usePRes = true;
			break;
		case POW:
			// Really just successive multiplication for now
			throw rtmath::debug::xUnimplementedFunction();
			break;
		default:
			throw rtmath::debug::xBadInput();
			break;
		}

		// This is needed because initialization cannot occur in a switch block.
		// If true, convert pRes to a shared_ptr
		if (usePRes)
		{
			std::shared_ptr<matrixop> h(new matrixop(pRes));
			res = h;
		}

		// Cache the calculated value, for later calculations
		if (_eval_cache_enabled)
			_eval_cache[valmap] = res;

		return res;
	}

}; // end namespace rtmath


