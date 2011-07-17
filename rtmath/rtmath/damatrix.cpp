#include "Stdafx.h"
#include "damatrix.h"
#include "quadrature.h"
#include "damatrix_quad.h"
#include "common_templates.h"
#define _USE_MATH_DEFINES
#include <math.h>
namespace rtmath {

	std::shared_ptr<damatrix> damatrix::eval(const mapid &valmap)
	{
		using namespace std;
		// NOTE: cannot be constant due to storage of calculations
		// The purpose of this function is to evaluate the matrix at valmap
		// valmap contains mu,mun,phi,phin
		// Assume that this is not the base for some other derived class (a provider)

		// First, look at the precached table. If a result is there, return it
		for (map<mapid,shared_ptr<damatrix>, mmapcomp >::const_iterator it=precalc.begin(); 
			it != precalc.end(); it++)
		{
			// *it->second.get() is awkward phrasing. Eval really should just return a pointer
			if (it->first == valmap) return it->second;
		}

		// Damn. It has to be calculated.
		// Evaluation occurs by performing the necessary matrix multiplication and 
		// integration of the two parent matrices
		// Transfer a copy of valmap to a local object
		mapid val = valmap;
		//shared_ptr<damatrix> res( new damatrix(_rootA->size()) );
		damatrix res(_rootA->size());
		// Annoying to do, as I want to dereference the pointers
		matrixop A(_rootA->size()), B(_rootB->size());
		switch (_parentsource)
		{
		case ADD:
			A = *(_rootA->eval)(val);
			B = *(_rootB->eval)(val);
			res = A + B;
			break;
		case MULT:
			// This is trickier
			// Do the necessary integration by quadrature and store the result
			// AB = 1/pi * int_0^2pi*int_0^1 (A(mu,phi,mu',phi')B(mu',phi',mun,phin))mu' dmu'dphi'
			// Using gaussian quadrature (min. of 7 pts)
			// Note (hardwired for now at 7. TODO: fix for arb. quad. pt. number)
			{
				// The factor of 1/pi, since I'm integrating the whole matrix at once
				matrixop prefact = matrixop::diagonal(_rootA->size(),1.0/M_PI);
				// The outer integral
				// From 0 to 2pi, over phi'
				// Parameters for A and B are expressed as a mapid (mu,mun,phi,phin)
				// A gets (mu,mu',phi,phi'). B gets (mu',mun,phi',phin)
				// factormu will be the diagonal mu' matrix for the integration
				//  it changes with each mu'
				// Invoke the outer integral function (in namespace rtmath::daint)
				// it is not a class function
				// TODO: check that pointers work without reference here
				//   Unsure about shared_ptr details
				daint::outer_int(res,val,_rootA,_rootB); // res is changed here
			}
			break;
		case INV:
			// Also annoying
			// Evaluate _rootA and invert the result
			A = *(_rootA->eval)(val);
			B = A.inverse();
			res = B;
			break;
		case NONE:
			// Nothing to do here
			//*res = *this; // This is BAD!!!!!!!!!! It causes *this to be deleted when the function using res goes out of scope.
			// Instead, return a shared_ptr of this with a null deleter
			return shared_ptr<damatrix> (this, null_deleter());
			break;
		default:
			// Die in disgrace
			throw rtmath::debug::xBadInput();
			break;
		}
		
		shared_ptr<damatrix> resa(new damatrix(res));
		precalc[val] = resa;
		return resa;
	}

	damatrix damatrix::operator* (damatrix& rhs)
	{
		// Create a resultant matrix that acts to multiply the two initial matrices
		damatrix res(this->size());
		res._parentsource = MULT; // For evaluation
		// Use shared_ptr because the parent must be held in memory for any 
		//  new calculations to occur. Auto-delete when object is forgotten!
		res._rootA = std::shared_ptr<damatrix>(new damatrix(*this) );  // Dangerous!!
		res._rootB = std::shared_ptr<damatrix>(&rhs);
		res._precalc_operator();
		// And we're done
		return res;
	}

	damatrix* damatrix::cloneDa() const
	{
		return new damatrix(*this);
	}

	damatrix damatrix::operator* (double rhs)
	{
		// Implemented so that I don't have to keep doing it elsewhere
		damatrix res(this->size());
		res._parentsource = MULT;
		res._rootA = std::shared_ptr<damatrix>(new damatrix(*this)); // Dangerous!!!
		// TODO: check that _precalc_operator works with resB
		res._rootB = std::shared_ptr<damatrix>(new damatrix(matrixop::diagonal(this->size(),rhs)));
		res._precalc_operator();
		return res;
	}

	damatrix damatrix::operator+ (damatrix &rhs)
	{
		// See multiplication operator for relevant comments
		damatrix res(this->size());
		res._parentsource = ADD; // For evaluation
		res._rootA = std::shared_ptr<damatrix>(new damatrix(*this)); // Dangerous!!!
		res._rootB = std::shared_ptr<damatrix>(&rhs);
		res._precalc_operator();
		// And we're done
		return res;
	}

	damatrix damatrix::inverse()
	{
		// This takes no rhs, as it is a unary operator
		damatrix res(this->size());
		res._parentsource = INV;
		res._rootA = std::shared_ptr<damatrix>(new damatrix(*this)); // Dangerous!!
		res._precalc_operator();
		return res;
	}

	void damatrix::_precalc_operator()
	{
		// Do some precalculation (for convenience later on)
		// Take all of precalc mapids in rootA and evaluate in the child
		for (std::map<mapid, std::shared_ptr<damatrix>, mmapcomp >::const_iterator it=_rootA->precalc.begin();
			it != _rootA->precalc.end(); it++)
				eval(it->first);
		// Try with _rootB, if it exists
		if (_rootB)
		{
		for (std::map<mapid, std::shared_ptr<damatrix>, mmapcomp >::const_iterator it=_rootB->precalc.begin();
			it != _rootB->precalc.end(); it++)
				eval(it->first);
		}
	}
	
}; // end rtmath

