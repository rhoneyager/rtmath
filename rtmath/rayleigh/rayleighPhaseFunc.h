#pragma once

#include "../rtmath-base/rtmath-base.h"

namespace rayleigh {

	// This depends on rtmath-base
class rayleighPhaseFunc :
	public rtmath::phaseFunc
{
public:
	rayleighPhaseFunc(void);
	virtual ~rayleighPhaseFunc(void);
	//virtual void eval(double mu, double mun, double phi, double pn, double *res[4][4]);
	virtual void calc(double mu, std::complex<double> &m, double x, double Pnn[4][4]);
};

}; // end namespace rayleigh

