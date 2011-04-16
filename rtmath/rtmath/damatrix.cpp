#include "damatrix.h"
#include "../rtmath-base/quadrature.h"
#define _MATH_DEFINES_DEFINED
#include <cmath>
namespace rtmath {

	void daevalmu::setvals(double phi, double phin, double php, double mu, double mun)
	{
		_phi = phi;
		_phin = phin;
		_php = php;
		_mu = mu;
		_mun = mun;
	}

	double daevalmu::eval(double val) const
	{
		// Val is mu'
		double mup = val;
		// Will evaluate X^ik(mu,mu',phi-phi') and Y^kj(mu',mu0,phi'-phi_0)
		// X and Y must be evaluateable (but not evalfunctions)
		// And, link a derived class to evalfunction that wraps them
		return 0.0;
	}

	double daevalmu::operator() (double val) const
	{
		// Val is mu'
		return eval(val);
	}


	void daevalphi::setvals(double phi, double phin, double mu, double mun)
	{
		_phi = phi;
		_phin = phin;
		_mu = mu;
		_mun = mun;
	}

	double daevalphi::eval(double val) const
	{
		// Val is phi'
		double res;
		res = rtmath::quadrature::quad_eval_leg(0.0,1.0,7,&mueval);
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
				// Construct the sum at the specified parameters
				// fixed parameters are _mu, _mu0, _ppn
				// varying parameters are mup, phip
				// Sum is k=1..4: X^ik(mu,mu',ppn) Y^kj(mu',mu0,ppn)*mu'
			}
		}
		throw;
		return ls;
	}

	void damatrix::setparams(double mu, double mun, double ppn)
	{
		_mu = mu;
		_mun = mun;
		_ppn = ppn;
	}

}; // end rtmath

