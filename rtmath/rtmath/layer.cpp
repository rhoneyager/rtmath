#include "Stdafx.h"
#include "layer.h"
#include "../rtmath-base/phaseFunc.h"
#include "../rtmath-base/matrixop.h"
#include "../rtmath-base/quadrature.h"
#include "damatrix.h"
#include <boost/shared_ptr.hpp>
#include <map>
#include <vector>

namespace rtmath
{

	dalayer_init::~dalayer_init()
	{
	}

	boost::shared_ptr<damatrix> dalayer_init::eval(const mapid &valmap)
	{
		// valmap contains mu, mun, phi, phin
		// _tau gives optical depth - value is FIXED!!!!!!
		
		// This function overrides damatrix::eval. So,
		// search the cache for existing values
		for (std::map<mapid,boost::shared_ptr<damatrix>, mmapcomp >::const_iterator it=precalc.begin(); 
			it != precalc.end(); it++)
		{
			// *it->second.get() is awkward phrasing. Eval really should just return a pointer
			if (it->first == valmap) return it->second;
		}

		// TODO: CHECK THIS CODE!

		// Too bad. Must calculate the values for R or T.
		std::vector<unsigned int> size;
		size.push_back(4);size.push_back(4);
		boost::shared_ptr<matrixop> resa( new matrixop(size) );
		boost::shared_ptr<matrixop> res( new matrixop(size) );
		// _phaseMat already holds the phase matrix at mun
		// Invoke phasefuncrotator to move to proper orientation
		// TODO: is alpha equal to mu, or zero, or what?
		phaseFuncRotator::rotate(_rt, _phaseMat, valmap, *resa, valmap.mu);

		// Take the rotated phase function and build a layer
		*res = *resa * _tau * _ssa * (1.0/ (4.0 * abs(valmap.mu) * abs(valmap.mun) ));
		precalc[valmap] = boost::shared_ptr<damatrix> (new damatrix(*res));
		return boost::shared_ptr<damatrix> (new damatrix(*res));
	}

dalayer::dalayer(matrixop &pf, double alb)
{
	_pf = &pf;
	_ssa = alb;
	_tau = 0;
}


dalayer::~dalayer(void)
{
}

void dalayer::generateLayer()
{
	// Generate the layer with properties defined by tau,
	// ssa, phasefunction

	// Begin by determining the number of doublings - must 
	// begin with a very small tau
	unsigned int numDoubles = 0;
	double taueff = _tau;
	do {
		taueff /= 2.0;
		numDoubles++;
	} while (taueff >= 1e-10);
	// I have the initial layer thickness and the number of 
	// doublings needed to generate a layer of the correct depth

	// Generate the initial R and T matrices
//	boost::shared_ptr<damatrix> _Rinit, _Tinit;
	// Cannot use phasefunc directly - must encapsulate in a damatrix
	
	dalayer_init initR(*_pf, _ssa, taueff, rtselec::R);
	dalayer_init initT(*_pf, _ssa, taueff, rtselec::T);
	// TODO: see if any other method is possible
	boost::shared_ptr<damatrix> _Rinit = boost::shared_ptr<damatrix> (new dalayer_init(initR));
	boost::shared_ptr<damatrix> _Tinit = boost::shared_ptr<damatrix> (new dalayer_init(initT));

	// Perform doubling until desired tau is reached
	throw;
	// Individ. layers are homog. here, so R*=R, T*=T
	for (unsigned int i=0;i<numDoubles;i++)
	{
	}
}

/*
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
*/
}; // end rtmath


