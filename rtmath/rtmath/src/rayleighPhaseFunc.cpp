#include "../rtmath/Stdafx.h"
#include "../rtmath/rayleigh/rayleigh.h"
#include <cmath>
#include <complex>

namespace rayleigh {

	std::shared_ptr<rtmath::matrixop> rayleighPhaseFunc::eval(double alpha) const
	{
		if (_eval_cache.count(alpha) > 0)
		{
			return _eval_cache[alpha];
		}

		rayleigh::Scalc sc(_m,_x);

		double _mu = cos(alpha);

		double Snn[4][4];
		std::complex<double> Sn[4];
		sc.calc(_mu,Snn,Sn);
		rayleigh::Qcalc q(_m);
		// For mie scattering, only P11, P12, P33 and P34 are unique
		// All terms not based on them are zero
		double Qext, Qabs, Qsca, g;
		rtmath::matrixop resa(2,4,4);

		q.calc(_x,Qext,Qsca,Qabs,g);
		for (unsigned int i=0;i<4;i++)
			for (unsigned int j=0;j<4;j++)
				resa.set(4 * Snn[i][j] / (_x * _x * Qext), 2, i,j);
				//Pnn[i][j] = 4 * Snn[i][j] / (x * x * Qext);

		std::shared_ptr<rtmath::matrixop> res(new rtmath::matrixop(resa));
		_eval_cache[alpha] = res;

		return res;
	}

	/*
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
	*/

}; // end rayleigh

