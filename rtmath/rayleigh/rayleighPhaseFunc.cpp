#include "rayleighPhaseFunc.h"
#define _USE_MATH_DEFINES
#include <cmath>
#include <complex>
#include "Scalc.h"
#include "Qcalc.h"

namespace rayleigh {

rayleighPhaseFunc::rayleighPhaseFunc(void)
{
}


rayleighPhaseFunc::~rayleighPhaseFunc(void)
{
}

void rayleighPhaseFunc::calc(double mu, std::complex<double> &m, double x, double Pnn[4][4])
{
	rayleigh::Scalc sc(m,x);
	double Snn[4][4];
	std::complex<double> Sn[4];
	sc.calc(mu,Snn,Sn);
	rayleigh::Qcalc q(m);
	// For mie scattering, only P11, P12, P33 and P34 are unique
	// All terms not based on them are zero
	double Qext, Qabs, Qsca, g;
	q.calc(x,Qext,Qsca,Qabs,g);
	for (unsigned int i=0;i<4;i++)
		for (unsigned int j=0;j<4;j++)
			Pnn[i][j] = 4 * Snn[i][j] / (x * x * Qext);
}

}; // end rayleigh

