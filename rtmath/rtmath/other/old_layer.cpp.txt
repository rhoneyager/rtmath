#include "Stdafx.h"
#include "layer.h"
#include "phaseFunc.h"
#include "matrixop.h"
#include "quadrature.h"
#include "damatrix.h"
#include <map>
#include <vector>

#include "debug_mem.h"

namespace rtmath
{

	dalayerInit::~dalayerInit()
	{
	}

	damatrix* dalayerInit::eval(const mapid &valmap)
	{
		using namespace std;
		// valmap contains mu, mun, phi, phin
		// _tau gives optical depth - value is FIXED!!!!!!
		
		// This function overrides damatrix::eval. So,
		// search the cache for existing values
		for (map<mapid,damatrix*, mmapcomp >::const_iterator it=precalc.begin();
			it != precalc.end(); it++)
		{
			// *it->second.get() is awkward phrasing. Eval really should just return a pointer
			if (it->first == valmap) return it->second;
		}

		// TODO: CHECK THIS CODE!

		// Too bad. Must calculate the values for R or T.
		vector<unsigned int> size;
		size.push_back(4);size.push_back(4);
		shared_ptr<matrixop> resa( new matrixop(size) );
		shared_ptr<matrixop> res( new matrixop(size) );
		// _phaseMat already holds the phase matrix at mun
		// Invoke phasefuncrotator to move to proper orientation
		// TODO: is alpha equal to mu, or zero, or what?
		phaseFuncRotator::rotate(_rt, *_phaseMat, valmap, *resa, valmap.mu);

		// Take the rotated phase function and build a layer
		*res = *resa * _tau * _ssa * (1.0/ (4.0 * abs(valmap.mu) * abs(valmap.mun) ));
		damatrix* sharedres = new damatrix(*res); // res is cloned per constructor
		
		precalc[valmap] = sharedres;
		return sharedres;
	}

dalayer::dalayer(matrixop &pf, double alb)
{
	_pf = &pf;
	_ssa = alb;
	_tau = 0;
	_R = 0;
	_T = 0;
	_U = 0;
	_D = 0;
}

dalayer::dalayer()
{
	static std::vector<unsigned int> size(2,4);
	static matrixop idmat = matrixop::identity(size);
	_pf = &idmat; // Set the phase function to the identity matrix
	_ssa = 0; // No single-scattering albedo, as no scattering
	_tau = 0;
	_R = 0;
	_T = 0;
	_U = 0;
	_D = 0;
}


dalayer::~dalayer(void)
{
}

void dalayer::generateLayer(const mapid &valmap)
{
	using namespace std;
	// Generate the layer with properties defined by tau,
	// ssa, phasefunction
	//throw rtmath::debug::xUnimplementedFunction();
	// Begin by determining the number of doublings - must 
	// begin with a very small tau
	unsigned int numDoubles = 0; // The number of doubles necessary to achieve the desired thickness
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
	
	//dalayerInit initR(*_pf, _ssa, taueff, rtselec::R);
	//dalayerInit initT(*_pf, _ssa, taueff, rtselec::T);
	// TODO: see if any other method is possible
	damatrix* _Rinit = (new dalayerInit(*_pf, _ssa, taueff, rtselec::R));
	damatrix* _Tinit = (new dalayerInit(*_pf, _ssa, taueff, rtselec::T));

	// Feed in a dummy variable
	//mapid valmap(0,0,0,0);

	// The initial R and T matrices
	damatrix* _Rorig = _Rinit->eval(valmap);
	damatrix* _Torig = _Tinit->eval(valmap);

	_R = _Rorig;
	_T = _Torig;

	// TODO: check all pointers here to make sure that they remain valid!!!!!!!

	// Perform doubling until desired tau is reached
	// Individ. layers are homog. here, so R*=R, T*=T
	for (unsigned int i=0;i<numDoubles;i++)
	{
		damatrix Q = *_R * *_R; // The new value for Q depends on the previous calculation (or the initial thin layer)

		// TODO: fix inverse calculation
		// TODO: extend damatrix to allow addition and mult. of constant numbers
		// Also, see if this is the correct S...
		//damatrix S = Q * (Q * damatrix(matrixop::diagonal(-1.0,2,4,4)) + 1.0).inverse();
		// The alternate formulation for S
		damatrix S(Q.size());
		for (unsigned int j=1; j<10; j++) // TODO: check for sufficient convergence
		{
			// TODO: check that this works correctly (lose track of initial S?)
			S = Q * Q;
			damatrix* r = S.eval(valmap);
			r->print();
		}
		// TODO: fix U,D mun value
		//    The value for mu_0 is determined by the val map! It can only be generated when the desired 
		//    angle is given. This poses a minor problem, as this information is not yet known.....
		//damatrix D = *_T + (S * *_T) + (S * exp(-1.0*_tau / valmap.mun));
		//damatrix U = *_R * D + *_R * exp(-1.0*_tau / valmap.mun);
		//damatrix Rnew = *_R + U * exp(-1.0*_tau / valmap.mu) + *_T * U;
		//damatrix Tnew = *_T * D + *_T * exp(-1.0*_tau/valmap.mun) + D * exp(-1.0*_tau / valmap.mu);
		//_R = shared_ptr<damatrix> (new damatrix(Rnew));
		//_T = shared_ptr<damatrix> (new damatrix(Tnew));
		
	}
	// And we have a fully-generated layer!!!!!!!
}

}; // end rtmath


