#include "Stdafx.h"
#include "damatrix.h"
#include "../rtmath-base/quadrature.h"
#define _MATH_DEFINES_DEFINED
#include <cmath>
namespace rtmath {

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

	double daevalmu::eval(double val) const
	{
		// Val is mu'
		double mup = val;
		// Will evaluate X^ik(mu,mu',phi-phi') and Y^kj(mu',mu0,phi'-phi_0)
		// X and Y must be evaluateable (but not evalfunctions)
		// And, link a derived class to evalfunction that wraps them
		double res = 0.0;
		// For most cases, targets.size()[0] should be 4
		for (unsigned int k=0; k<targeta->size()[0]; k++)
		{
			//targeta.
		}
		return 0.0;
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

	double daevalphi::eval(double val) const
	{
		// Val is phi'
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
		throw;
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

}; // end rtmath

