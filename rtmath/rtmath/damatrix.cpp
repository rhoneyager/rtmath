#include "Stdafx.h"
#include "damatrix.h"
#include "matrixop.h"
#include "error.h"
#include "common_templates.h"
#include <memory>

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

	std::shared_ptr<matrixop> damatrix::eval(const mapid &valmap)
	{
		// Evaluate the damatrix at the necessary values
		// TODO: store values for future calculations

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
		case MULT:
			// Hardest, as integration by quadrature must occur
			throw rtmath::debug::xUnimplementedFunction();
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

		return res;
	}

}; // end namespace rtmath


