#pragma once

//#include "../rtmath-base/rtmath-base.h"
#include "../matrixop.h"
#include "../rtmath.h"
#include <memory>

namespace rayleigh {

	// This depends on rtmath-base
class rayleighPhaseFunc :
	public rtmath::phaseFunc
{
public:
	rayleighPhaseFunc(double x, std::complex<double> &m) : rtmath::phaseFunc()
	{
		_m = m;
		_x = x;
	}
	virtual ~rayleighPhaseFunc(void) {}
	//virtual void calc(double mu, std::complex<double> &m, double x, double Pnn[4][4]);
	virtual std::shared_ptr<rtmath::matrixop> eval(double alpha) const;
private:
	double _x;
	std::complex<double> _m;
};

}; // end namespace rayleigh

