#pragma once
#include<complex>
#include "abNCalc.h"

namespace mie {


class Qcalc
{
public:
	Qcalc(const std::complex<double> &m);
	Qcalc(const std::complex<double> &m, double tolerance, double atol);
	~Qcalc(void);
	void calc(double x, double &Qext, double &Qsca, double &Qabs, double &g);
private:
	std::complex<double> _m;
	double _tolerance;
	double _atol;
};

}; // end mie

