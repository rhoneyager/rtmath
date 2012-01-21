#include "../rtmath/Stdafx.h"
#include <complex>
#include <cmath>
#include "../rtmath/rayleigh/rayleigh.h"

namespace rayleigh {

Qcalc::Qcalc(const std::complex<double> &m)
{
	_m = m;
	_tolerance = 1.01;
	_atol = 1e-8;
}

Qcalc::Qcalc(const std::complex<double> &m, double tolerance, double atol)
{
	_m = m;
	_tolerance = tolerance;
	_atol = atol;
}

Qcalc::~Qcalc(void)
{
}

void Qcalc::calc(double x, double &Qext, double &Qsca, double &Qabs, double &g)
{
	// The Rayleigh case is much easier than the mie case
	// Formulas for calculating Qabs and Qsca are in VDH, p.70
	std::complex<double> mtop, mbot, mquo;
	mtop = _m*_m - 1.0;
	mbot = _m*_m + 2.0;
	mquo = mtop / mbot;
	Qsca = (8.0/3.0)* pow(x,4.0) * pow(std::abs(mquo),2.0);
	Qabs = -4.0*x*mquo.imag();
	Qext = Qsca + Qabs;
	// That was easy. Now find g.
	g = 0; // Rayleigh is symmetric!
}

}; // end rayleigh

