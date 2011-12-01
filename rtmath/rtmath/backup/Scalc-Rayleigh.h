#pragma once
#include <complex>
#include "rtmath.h"
namespace rayleigh {

class Scalc : 
	public rtmath::scattMatrix
{
public:
	Scalc(const std::complex<double> &m, double x);
	Scalc(const std::complex<double> &m, double x, double tol, double atol);
	~Scalc(void);
	void calc(double mu, double Snn[4][4], std::complex<double> Sn[4]);
	//void calc(double mu, std::complex<double> &Sa, std::complex<double> &Sb, double &Ssq);
private:
	std::complex<double> _m;
	double _x;
	double _tolerance;
	double _atol;
};

}; // end rayleigh

