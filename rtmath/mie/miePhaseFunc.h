#pragma once

#include "../rtmath-base/rtmath-base.h"

namespace mie {

	// This depends on rtmath-base
class miePhaseFunc :
	public rtmath::phaseFunc
{
public:
	miePhaseFunc(void);
	virtual ~miePhaseFunc(void);
	//virtual void eval(double mu, double mun, double phi, double pn, double *res[4][4]);
	virtual void calc(double mu, std::complex<double> &m, double x, double Pnn[4][4]);
};

}; // end namespace mie

