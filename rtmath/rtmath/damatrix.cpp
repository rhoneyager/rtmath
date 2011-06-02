#include "Stdafx.h"
#include "damatrix.h"
#include "../rtmath-base/quadrature.h"
#include "damatrix_quad.h"
#define _USE_MATH_DEFINES
#include <math.h>
namespace rtmath {
	/* // Block these for now
	daevalmuint::daevalmuint(damatrix *targeta, damatrix *targetb)
	{
		if (targeta == NULL) throw;
		if (targetb == NULL) throw;
		this->targeta = targeta;
		this->targetb = targetb;
	}

	void daevalmuint::setA(unsigned int i, unsigned int j,
			double a, double b, double c, unsigned int evalindex)
	{
		_Ai = i;
		_Ab = j;
		_Aa = a;
		_Ab = b;
		_Ac = c;
		_An = evalindex;
	}

	void daevalmuint::setB(unsigned int i, unsigned int j,
			double a, double b, double c, unsigned int evalindex)
	{
		_Bi = i;
		_Bb = j;
		_Ba = a;
		_Bb = b;
		_Bc = c;
		_Bn = evalindex;
	}

	double daevalmuint::eval(double val) const
	{
		// Val is phi'
		// I've declared a const function, so I should duplicate the values here (using pointers)
		const double *_pAa = &_Aa, *_pAb = &_Ab, *_pAc = &_Ac;
		const double *_pBa = &_Ba, *_pBb = &_Bb, *_pBc = &_Bc;

		switch(_An)
		{
		case 1:
			_pAa = &val;
			break;
		case 2:
			_pAb = &val;
			break;
		case 3:
			_pAc = &val;
			break;
		};
		switch(_Bn)
		{
		case 1:
			_pBa = &val;
			break;
		case 2:
			_pBb = &val;
			break;
		case 3:
			_pBc = &val;
			break;
		};

		// And now, evaluate!
		double res = 0.0, resa, resb;

		resa = targeta->eval(_Ai, _Aj, *_pAa, *_pAb, *_pAc);
		resb = targeta->eval(_Bi, _Bj, *_pBa, *_pBb, *_pBc);

		res = val * resa * resb;
		return res;
	}

	double daevalmuint::operator() (double val) const
	{
		// Val is phi'
		return eval(val);
	}

	void daevalmu::setvals(double phi, double phin, double php, 
		double mu, double mun, unsigned int i, unsigned int j)
	{
		_phi = phi;
		_phin = phin;
		_php = php;
		_mu = mu;
		_mun = mun;
		_i = i;
		_j = j;
	}

	daevalmu::daevalmu()
	{
		targeta = NULL;
		targetb = NULL;
	}

	double daevalmu::eval(double val) const
	{
		// Val is mu'
		double mup = val;
		// Will evaluate X^ik(mu,mu',phi-phi') and Y^kj(mu',mu0,phi'-phi_0)
		// X and Y must be evaluateable (but not evalfunctions)
		// And, link a derived class to evalfunction that wraps them
		double res = 0.0;
		// For most cases, targets.size()[0] should be 4
		if (targeta == NULL) throw;
		if (targetb == NULL) throw;
		// _i and _j are already set. Iterate through k.
		daevalmuint evalint(targeta, targetb);
		for (unsigned int k=0; k<targeta->size()[0]; k++)
		{
			evalint.setA(_i,k,_mu,0.0,_phi-_php, 2);
			evalint.setB(k,_j,0.0,_mun,_php-_phin, 1);
			res += rtmath::quadrature::quad_eval_leg(0.0,1.0,7,&evalint);
		}
		return res;
	}

	double daevalmu::operator() (double val) const
	{
		// Val is mu'
		return eval(val);
	}

	void daevalphi::setvals(double phi, double phin, double mu, double mun,
		unsigned int i, unsigned int j)
	{
		_phi = phi;
		_phin = phin;
		_mu = mu;
		_mun = mun;
		_i = i;
		_j = j;
		// mueval.setvals(phi,phin,0,mu,mun,i,j); // Not here
	}

	daevalphi::daevalphi()
	{
		mueval = NULL;
	}

	double daevalphi::eval(double val) const
	{
		// Val is phi'
		if (mueval == NULL) throw;
		double res;
		mueval->setvals(_phi,_phin,val,_mu,_mun,_i,_j);
		res = rtmath::quadrature::quad_eval_leg(0.0,1.0,7,mueval);
		return res;
	}

	double daevalphi::operator() (double val) const
	{
		// Val is phi'
		return eval(val);
	}
	*/
/*
	damatrix damatrix::operator* (const damatrix& rhs) const
	{
		// Do matrix multiplication and then integration
		// First, create child copies of this and rhs
		// (we don't want to change anything)
		using namespace std;
		damatrix ls = *this;
		damatrix rs = rhs;
		damatrix res(this->size());

		// Now, perform the integral quadrature
		// Recursion is useful, and I don't mind if it's slow
		// TODO: prestore known quadrature points for faster iteration
		for (unsigned int i=0; i< _dims[0]; i++)
		{
			for (unsigned int j=0; j< _dims[1]; j++)
			{
				double resij = 0;
				std::vector<unsigned int> pos;
				pos.push_back(i);
				pos.push_back(j);
				// Construct the sum at the specified parameters
				// fixed parameters are _mu, _mu0, _ppn
				// varying parameters are mup, phip
				// Sum is k=1..4: X^ik(mu,mu',ppn) Y^kj(mu',mu0,ppn)*mu'
				daevalphi peval;
				peval.setvals(_phi, _phin, _mu, _mun, i, j);
				resij = quadrature::quad_eval_leg(0,1,7,&peval);
				// Set the result in the matrix
				res.set(pos,resij / 3.141592654);
			}
		}
		//res = res.matrixop::operator*( 1.0 / (3.141592654) );
		//throw;
		return res;
	}

	double damatrix::eval(unsigned int i, unsigned int j, 
		double a, double b, double c)
	{
		// In the default invocation, a b and c don't matter
		// So, just return the [i,j] element
		std::vector<unsigned int> loc;
		loc.push_back(i);
		loc.push_back(j);
		return this->get(loc);
	}
	*/

	boost::shared_ptr<damatrix> damatrix::eval(const mapid &valmap)
	{
		// NOTE: cannot be constant due to storage of calculations
		// The purpose of this function is to evaluate the matrix at valmap
		// valmap contains mu,mun,phi,phin
		// Assume that this is not the base for some other derived class (a provider)

		// First, look at the precached table. If a result is there, return it
		for (std::map<mapid,boost::shared_ptr<damatrix>, mmapcomp >::const_iterator it=precalc.begin(); 
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
		boost::shared_ptr<damatrix> res( new damatrix(_rootA->size()) );
		// Annoying to do, as I want to dereference the pointers
		matrixop A(_rootA->size()), B(_rootB->size());
		switch (_parentsource)
		{
		case ADD:
			A = *(_rootA->eval)(val);
			B = *(_rootB->eval)(val);
			*res = A + B;
			// Remember it
			precalc[val] = res;
			return res;
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
				daint::outer_int(res,val,_rootA,_rootB);
				precalc[val] = res;
				return res;
			}
			break;
		case INV:
			// Also annoying
			// Evaluate _rootA and invert the result
			A = *(_rootA->eval)(val);
			B = A.inverse();
			*res = B;
			precalc[val] = res;
			return res;
			break;
		default:
			// Die in disgrace
			throw;
			break;
		}
		// We'll never get here, but it gets rid of a compiler warning
		return res;
	}

	damatrix damatrix::operator* (damatrix& rhs)
	{
		// Create a resultant matrix that acts to multiply the two initial matrices
		damatrix res(this->size());
		res._parentsource = MULT; // For evaluation
		// Use shared_ptr because the parent must be held in memory for any 
		//  new calculations to occur. Auto-delete when object is forgotten!
		res._rootA = boost::shared_ptr<damatrix>(this); 
		res._rootB = boost::shared_ptr<damatrix>(&rhs);
		res._precalc_operator();
		// And we're done
		return res;
	}

	damatrix damatrix::operator+ (damatrix &rhs)
	{
		// See multiplication operator for relevant comments
		damatrix res(this->size());
		res._parentsource = ADD; // For evaluation
		res._rootA = boost::shared_ptr<damatrix>(this); 
		res._rootB = boost::shared_ptr<damatrix>(&rhs);
		res._precalc_operator();
		// And we're done
		return res;
	}

	void damatrix::_precalc_operator()
	{
		// Do some precalculation (for convenience later on)
		// Take all of precalc mapids in rootA and evaluate in the child
		for (std::map<mapid, boost::shared_ptr<damatrix>, mmapcomp >::const_iterator it=_rootA->precalc.begin();
			it != _rootA->precalc.end(); it++)
				eval(it->first);
		// Try with _rootB, if it exists
		if (_rootB)
		{
		for (std::map<mapid, boost::shared_ptr<damatrix>, mmapcomp >::const_iterator it=_rootB->precalc.begin();
			it != _rootB->precalc.end(); it++)
				eval(it->first);
		}
	}
	
}; // end rtmath

