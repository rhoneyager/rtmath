#pragma once

#include "../rtmath-base/phaseFunc.h"
#include "../rtmath-base/matrixop.h"
#include "../rtmath-base/quadrature.h"
#include "damatrix.h"
#include <map>
#include <vector>

namespace rtmath {

class dalayer
{
public:
	/* Any layer takes on some parameters
	 * These include optical depth (tau) at zenith,
	 * The type of layer (from a derived class),
	 * a generating function, 
	 * single-scattering albedo,
	 * a base phase function,
	 * etc.
	 * Other parameters (mu, mun, phi, phin) are used in layer 
	 * calculation and are calculated repeatedly for each angle
	 * (TODO: cache these)
	 */
	dalayer(phaseFunc *pf, double alb);
	virtual ~dalayer(void);
	damatrix calcR(double tau, double phi, double phin, 
			double mu, double mun);
	damatrix calcT(double tau, double phi, double phin, 
			double mu, double mun);
protected:
	phaseFunc *_pf;
	double _alb;
};



}; // end rtmath


