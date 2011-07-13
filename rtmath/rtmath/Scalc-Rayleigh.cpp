#include "Stdafx.h"
#include "Scalc-Rayleigh.h"
#include "Qcalc-Rayleigh.h"
#include <complex>

#include <cmath>

namespace rayleigh {

Scalc::Scalc(const std::complex<double> &m, double x)
{
	_m = m;
	_x = x;
	_tolerance = 1.01;
	_atol = 1e-8;
}

Scalc::Scalc(const std::complex<double> &m, double x, double tol, double atol)
{
	_tolerance = tol;
	_atol = atol;
	_m = m;
	_x = x;
}

Scalc::~Scalc(void)
{
}

//void Scalc::calc(double mu, std::complex<double> &Sa, std::complex<double> &Sb, double &Ssq)
void Scalc::calc(double mu, double Snn[4][4], std::complex<double> Sn[4])
{
	//rayleigh::Qcalc Q(_m); // Not needed until phase function calc...
	// Zero Snn and Sn
	std::complex<double> zero(0,0);
	for (unsigned int i=0;i<4;i++)
		Sn[i] = zero;
	for (unsigned int i=0; i<4;i++)
		for (unsigned int j=0; j<4; j++)
			Snn[i][j] = 0;
	// Don't call _genMuellerMatrix - the values for Snn are trivial to canculate
	double theta = std::acos(mu);
	// See Bickel et al. 1976
	Snn[0][0] = 1.0 + (mu*mu);
	Snn[1][1] = 1.0 + (mu*mu);
	Snn[0][1] = std::sin(theta) * std::sin(theta);
	Snn[1][0] = std::sin(theta) * std::sin(theta);
	Snn[2][2] = mu;
	Snn[3][3] = mu;

	// And, the elements of Sn are:
	Sn[0] = -1.0 * std::pow(_x,3) * (_m*_m-1.0)/(_m*_m+2.0);
	Sn[1] = -1.0 * std::pow(_x,3) * (_m*_m-1.0)/(_m*_m+2.0) * mu;
	// Spheres should have S1(0)=S2(0)
	// Spheres have S3=S4=0
	return;
}

}; // end rayleigh


