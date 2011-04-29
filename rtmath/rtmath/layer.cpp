#include "Stdafx.h"
#include "layer.h"
#include "../rtmath-base/phaseFunc.h"
#include "../rtmath-base/matrixop.h"
#include "../rtmath-base/quadrature.h"
#include "damatrix.h"
#include <map>
#include <vector>

namespace rtmath
{

dalayer::dalayer(phaseFunc *pf, double alb)
{
	_pf = pf;
	_alb = alb;
}


dalayer::~dalayer(void)
{
}


damatrix dalayer::calcR(double tau, double phi, double phin, 
	double mu, double mun)
{
	return calcParam(tau,phi,phin,mu,mun,rtselec::R);
}

damatrix dalayer::calcT(double tau, double phi, double phin, 
	double mu, double mun)
{
	return calcParam(tau,phi,phin,mu,mun,rtselec::T);
}

damatrix dalayer::calcParam(double tau, double phi, double phin, 
	double mu, double mun, rtselec::rtselec rt)
{
	std::vector<unsigned int> siz;
	siz.push_back(4);
	siz.push_back(4);
	damatrix nul(siz);
	return nul;
	// Solution by recursion
	// Yeah, this stinks, but it's how DA is defined
	// Written so that recursive calls are not made
	//  so as not to kill the stack

	// rt is used to select for transmission or reflection
	// this changes the phase function angle-change matrix (give better name)

	// Start by recursively dividing until tau is sufficiently small
	// Compute R and T for tau'
	// Compute other matrices, and get R and T for 2*tau'
	// Work up until desired thickness is reached

	// This may be difficult without direct recursion
	// Should cache the angles used in quadrature calculations

	// The phasefunc pointer is *_pf in this class

	// First, create the appropriate rotation matrices for R and T
}

}; // end rtmath


