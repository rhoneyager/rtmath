#include "../rtmath/Stdafx.h"
#include <memory>
#include <math.h>
#include "../rtmath/gridded/gridded.h"


namespace rtmath {

	namespace griddata
	{

		gridded::~gridded()
		{
			// No need to do anything!
		}

		gridded::gridded()
		{
			// Only called by itself internally, when producing a gridded that 
			// is the result of op(...).
			// Also called by the other constructors, to put any initializations
			// in the right place.

			// Default values: (probably will be overridden)
			__init();
		}

		gridded::gridded(const std::shared_ptr<gridded> lhs, 
				const std::shared_ptr<gridded> rhs, gridOp oper)
		{
			//if (oper == NONE) throw rtmath::debug::xBadInput("No gridded operation specified.");
			if (oper == ADD || oper == MULT || oper == MULTCOMPONENT)
				if (rhs == nullptr)
					throw rtmath::debug::xBadInput("Need rhs to be a valid gridded.");
			if (oper == POW)
				throw debug::xBadInput("Inappropriate constructor called.");
			__init();
			_parentOp = oper;
			_rootA = lhs;
			_rootB = rhs;

		}

		void gridded::__init()
		{
			_pow = 0;
			_multval = 0;
			_needsRot = false;
			_parentOp = NONE;
			_eval_cache_enabled = false;
		}

		std::shared_ptr<gridded> gridded::pow(const std::shared_ptr<gridded> &base, unsigned int power)
		{
			// Use the private constructor here
			std::shared_ptr<gridded> res(new gridded());
			res->_pow = power;
			res->_rootA = base;
			res->_parentOp = POW;
			return res;
		}

		std::shared_ptr<gridded> gridded::smult(const std::shared_ptr<gridded> &base, double multval)
		{
			// Use the private constructor here
			std::shared_ptr<gridded> res(new gridded());
			res->_multval = multval;
			res->_rootA = base;
			res->_parentOp = MULTVAL;
			return res;
		}

		std::shared_ptr<gridded> gridded::op(std::shared_ptr<gridded> lhs, 
			std::shared_ptr<gridded> rhs, gridOp oper)
		{
			std::shared_ptr<gridded> res(new gridded(lhs,rhs,oper));
			return res;
		}

		void gridded::size(std::vector<size_t> &out) const
		{
			// Return the expected matrix size
			out.clear();
			throw rtmath::debug::xUnimplementedFunction();
		}

		std::shared_ptr<const matrixop> gridded::eval(const coords::cyclic<double> &start, 
				const coords::cyclic<double> &span) const
		{
			// First, check to see if this has already been calculated
			// If it is in the cache, return the cached value
			/*if (_eval_cache_enabled) 
				if (_eval_cache.count(valmap) > 0)
					return _eval_cache[valmap];
		*/
			// Desired mapid (valmap) is not in the cache, so the matrixop must be calculated
			// Evaluate the gridded at the necessary values

			// Partial result holders, for convenience (no messy shared_ptrs)
			//TODO: set matrixop size based on dimensions of gridded
			matrixop pRes(2,4,4);
			matrixop aRes(2,4,4);
			matrixop bRes(2,4,4);
			std::shared_ptr<const matrixop> aResS, bResS;

			// Operations are based on _parentOp's option
			switch (_parentOp)
			{
			case NONE:
				// The easiest case
				aResS = _rootA->eval(start,span);
				pRes = *aResS;
				//throw rtmath::debug::xBadInput("Evaluating, but no gridded operation specified.");
				break;
			case ADD:
				// Direct addition
				aResS = _rootA->eval(start,span);
				bResS = _rootB->eval(start,span);
				pRes = *aResS + *bResS;
				break;
			case INV:
				// Evaluate _rootA and take the inverse
				aResS = _rootA->eval(start,span);
				pRes = aResS->inverse();
				break;
			case MULT:
				aResS = _rootA->eval(start,span);
				bResS = _rootB->eval(start,span);
				pRes = *aResS * *bResS;
				break;
			case MULTCOMPONENT:
				aResS = _rootA->eval(start,span);
				bResS = _rootB->eval(start,span);
				pRes = *aResS % *bResS;
				break;
			case MULTVAL:
				aResS = _rootA->eval(start,span);
				pRes = *aResS * _multval;
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
						aResS = _rootA->eval(start,span);
						for (unsigned int i=0;i<_pow;i++)
						{
							pRes = pRes * *aResS;
						}
					}
				}
				break;
			default:
				throw rtmath::debug::xBadInput("No gridded operation specified.");
				break;
			};
		
			// Create the resultant matrixop
			std::shared_ptr<const matrixop> res(new matrixop(pRes));

			// Cache the calculated value, for later calculations
			/*if (_eval_cache_enabled)
				_eval_cache[valmap] = res;
				*/
			return res;
		}

	} // end namespace griddata

}; // end namespace rtmath


