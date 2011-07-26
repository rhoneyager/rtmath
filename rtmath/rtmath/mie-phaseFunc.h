#pragma once

#include "rtmath-base.h"
#include <complex>
#include "matrixop.h"
#include <memory>

namespace mie {

	// This depends on rtmath-base
class miePhaseFunc :
	public rtmath::phaseFunc
{
public:
	miePhaseFunc(double x, std::complex<double> &m) : rtmath::phaseFunc()
	{
		_m = m;
		_x = x;
	}
	virtual ~miePhaseFunc(void) {}
	virtual std::shared_ptr<rtmath::matrixop> eval(double alpha) const;
	//virtual void eval(double mu, double mun, double phi, double pn, double *res[4][4]);
	//virtual void calc(double mu, std::complex<double> &m, double x, double Pnn[4][4]);
private:
	double _x;
	std::complex<double> _m;
};

}; // end namespace mie

