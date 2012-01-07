#include "../rtmath/Stdafx.h"
#include "../rtmath/rtmath.h"
#define _USE_MATH_DEFINES
#include <cmath>
#include <complex>
namespace mie {

	//TODO: rewrite this function, giving dependency on rtmath-base provided matrix operations!
	// See Liou page 261 for the correct form of the phase function based on the scattering 
	// amplitude functions
	/*
	void miePhaseFunc::eval(double mu, double mun, double phi, double pn, double *res[4][4])
	{
	// This is the class-based array for calculating the phase function
	double phiangle=phi-pn;
	double muangle = mu-mun;
	// Spherical particles have S3=S4=0, so only have S1,S2(theta)
	// S are amplitude functions, p is the phase function
	// zero res
	for (unsigned int i=0; i<4; i++)
	{
	for (unsigned int j=0; j<4; j++)
	res[i][j] = 0;
	}

	}
	*/

	std::shared_ptr<rtmath::matrixop> miePhaseFunc::eval(double alpha) const
	{
		if (_eval_cache.count(alpha) > 0)
		{
			return _eval_cache[alpha];
		}

		double _mu = cos(alpha);

		mie::Scalc sc(_m,_x);
		double Snn[4][4];
		std::complex<double> Sn[4];
		sc.calc(_mu,Snn,Sn);
		mie::Qcalc q(_m);
		// For mie scattering, only P11, P12, P33 and P34 are unique
		// All terms not based on them are zero
		double Qext, Qabs, Qsca, g;
		q.calc(_x,Qext,Qsca,Qabs,g);
		rtmath::matrixop resa(2,4,4);

		for (unsigned int i=0;i<4;i++)
			for (unsigned int j=0;j<4;j++)
				resa.set(4 * Snn[i][j] / (_x * _x * Qext), 2, i, j);
				//Pnn[i][j] = 4 * Snn[i][j] / (_x * _x * Qext);

		// TODO: check these!
		/*
		Pnn[0][0] = 4 * Snn[0][0] / (x * x * Qext);
		Pnn[0][1] = 4 * Snn[0][1] / (x * x * Qext);
		Pnn[2][2] = 4 * Snn[2][2] / (x * x * Qext);
		Pnn[2][3] = 4 * Snn[2][3] / (x * x * Qext);
		// After this, fill iin the duplicates
		Pnn[1][0] = Pnn[0][1];
		Pnn[1][1] = Pnn[0][0];
		Pnn[3][3] = Pnn[2][2];
		*/

		std::shared_ptr<rtmath::matrixop> res(new rtmath::matrixop(resa));
		_eval_cache[alpha] = res;

		return res;
	}


	/*
	void miePhaseFunc::calc(double mu, std::complex<double> &m, double x, double Pnn[4][4])
	{
	mie::Scalc sc(m,x);
	double Snn[4][4];
	std::complex<double> Sn[4];
	sc.calc(mu,Snn,Sn);
	mie::Qcalc q(m);
	// For mie scattering, only P11, P12, P33 and P34 are unique
	// All terms not based on them are zero
	double Qext, Qabs, Qsca, g;
	q.calc(x,Qext,Qsca,Qabs,g);
	for (unsigned int i=0;i<4;i++)
	for (unsigned int j=0;j<4;j++)
	Pnn[i][j] = 4 * Snn[i][j] / (x * x * Qext);

	} */

}; // end mie

