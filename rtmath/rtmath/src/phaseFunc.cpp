#include "../rtmath/Stdafx.h"
#include "../rtmath/rtmath.h"
#include <complex>
#include <cmath>

namespace rtmath {
	/*
	void phaseFunc::eval(double mu, double mun, double phir, double Pnn[4][4], double res[4][4])
	{
	// Actually evaluate the phase function
	//double ia = acos();

	}
	*/

	/*
	void phaseFunc::calc(double mu, std::complex<double> &m, double x, matrixop &Pnn)
	{
		double Pnnres[4][4];
		calc(mu,m,x,Pnnres);
		// Now, convert values to a matrixop class
		Pnn.clear();
		Pnn.resize(2,4,4);
		// Put the appropriate values in the right cells
		for (unsigned int i=0;i<4;i++)
		{
			for (unsigned int j=0;j<4;j++)
			{
				Pnn.set(Pnnres[i][j],2,i,j);
			}
		}
		// Well, that was easy!
	}
	*/

	scattMatrix::scattMatrix(void)
	{
	}

	scattMatrix::~scattMatrix(void)
	{
	}

	void scattMatrix::_genExtinctionMatrix(double Knn[4][4], const std::complex<double> Sn[4], double k)
	{
		double f = 2.0 * M_PI / k;
		// Do the diagonals first
		for (size_t i=0;i<4;i++)
		{
			Knn[i][i] = (Sn[0] + Sn[3]).imag();
		}

		// And the remaining 12 (really duplicates, so only six)
		for (size_t i=0; i<4; i++)
			for (size_t j=0; j<4; j++)
			{
				if (i == 0 && j == 1) Knn[i][j] = (Sn[0] - Sn[3]).imag();
				if (i == 0 && j == 2) Knn[i][j] = -(Sn[1] + Sn[2]).imag();
				if (i == 0 && j == 3) Knn[i][j] = (-Sn[1] + Sn[2]).real();
				if (i == 1 && j == 2) Knn[i][j] = (Sn[2] - Sn[1]).imag();
				if (i == 1 && j == 3) Knn[i][j] = -(-Sn[1] + Sn[2]).real();
				if (i == 2 && j == 3) Knn[i][j] = (Sn[3] - Sn[0]).real();

				if (j == 0 && i == 1) Knn[i][j] = (Sn[0] - Sn[3]).imag(); // Same
				if (j == 0 && i == 2) Knn[i][j] = -(Sn[1] + Sn[2]).imag();
				if (j == 0 && i == 3) Knn[i][j] = (-Sn[1] + Sn[2]).real();
				if (j == 1 && i == 2) Knn[i][j] = -(Sn[2] - Sn[1]).imag(); // Negative
				if (j == 1 && i == 3) Knn[i][j] = (-Sn[1] + Sn[2]).real();
				if (j == 2 && i == 3) Knn[i][j] = -(Sn[3] - Sn[0]).real();
			}

		// Go back and multiply by f
		for (size_t i=0; i<4;i++)
			for (size_t j=0; j<4; j++)
				Knn[i][j] *= f;
	}

	void scattMatrix::_genMuellerMatrix(double Snn[4][4], const std::complex<double> Sn[4])
	{
		// Note, following ddscat conventions, matrix is [[S2, S3][S4,S1]]
		// It's annoying, but it's how ddscat does it and how the fml loading code provides it.
		// Now, Sn is the matrix in linear form {S1, S2, S3, S4}, so it should avoid any 
		// of the subsequent issues with forgetting the index transformations.

		std::complex<double> scratch;
		
		Snn[0][0] = 0.5 * ( (Sn[0]*conj(Sn[0])) + (Sn[1]*conj(Sn[1])) 
			+ (Sn[2]*conj(Sn[2])) + (Sn[3]*conj(Sn[3])) ).real();

		Snn[0][1] = 0.5 * ( (Sn[1]*conj(Sn[1])) - (Sn[0]*conj(Sn[0])) 
			+ (Sn[3]*conj(Sn[3])) - (Sn[2]*conj(Sn[2]))).real();
		
		scratch = ( (Sn[1] * (conj(Sn[2]))) + (Sn[0] * (conj(Sn[3])) ));
		Snn[0][2] = 1.0 * scratch.real();

		scratch = ( (Sn[1] * (conj(Sn[2]))) - (Sn[0] * (conj(Sn[3])) ));
		Snn[0][3] = 1.0 * scratch.imag();

		Snn[1][0] = 0.5 * ( -(Sn[0]*conj(Sn[0])) + (Sn[1]*conj(Sn[1])) 
			+ (Sn[2]*conj(Sn[2])) - (Sn[3]*conj(Sn[3])) ).real();

		Snn[1][1] = 0.5 * ( (Sn[0]*conj(Sn[0])) + (Sn[1]*conj(Sn[1]))
			- (Sn[2]*conj(Sn[2])) - (Sn[3]*conj(Sn[3])) ).real();

		scratch = ( (Sn[1] * (conj(Sn[2]))) - (Sn[0] * (conj(Sn[3])) ));
		Snn[1][2] = 1.0 * scratch.real();
		scratch = ( (Sn[1] * (conj(Sn[2]))) + (Sn[0] * (conj(Sn[3])) ));
		Snn[1][3] = 1.0 * scratch.imag();

		scratch = ( (Sn[0] * (conj(Sn[2]))) + (Sn[3] * (conj(Sn[1])) ));
		Snn[2][0] = 1.0 * scratch.real();
		scratch = ( -(Sn[0] * (conj(Sn[2]))) + (Sn[3] * (conj(Sn[1])) ));
		Snn[2][1] = 1.0 * scratch.real();

		scratch = ( (Sn[0] * (conj(Sn[1]))) + (Sn[2] * (conj(Sn[3])) ));
		Snn[2][2] = scratch.real();
		scratch = ( (Sn[1] * (conj(Sn[0]))) + (Sn[3] * (conj(Sn[2])) ));
		Snn[2][3] = scratch.imag();

		scratch = ( (conj(Sn[2]) * (Sn[0])) + (conj(Sn[1]) * (Sn[3]) ));
		Snn[3][0] = 1.0 * scratch.imag();
		scratch = ( -(conj(Sn[2]) * (Sn[0])) + (conj(Sn[1]) * (Sn[3]) ));
		Snn[3][1] = 1.0 * scratch.imag();

		scratch = ( (Sn[0] * (conj(Sn[1]))) - (Sn[2] * (conj(Sn[3])) ));
		Snn[3][2] = scratch.imag();
		scratch = ( (Sn[0] * (conj(Sn[1]))) - (Sn[2] * (conj(Sn[3])) ));
		Snn[3][3] = scratch.real();
	}

	/*
	phaseFuncRotator::phaseFuncRotator(phaseFunc &target, std::complex<double> &m, double x)
	{
		throw rtmath::debug::xObsolete();
		this->target = &target;
		this->x = x;
		this->m = m;
	}

	void phaseFuncRotator::rotate(rtselec::rtselec RT, const matrixop &Pa, 
		const mapid &varmap, matrixop &res, double alpha)
	{
		throw rtmath::debug::xObsolete();
		double cosia = 0;
		double cosib = 0;
		double mu = varmap.mu;
		double mun = varmap.mun;
		double phi = varmap.phi;
		double phin = varmap.phin;
		// Note that in phasefunc, mu is alpha!
		if (mu != mun && mu != 1.0)
		{ // Regular case
			if (RT == rtmath::rtselec::R)
			{
				// Reflection
				cosia = (-1.0*mu*sqrt(1.0-mun*mun)-mun*sqrt(1.0-mu*mu)*cos(phi-phin)) / sqrt(1.0- cos(alpha*alpha));
				cosib = (mun*sqrt(1.0-mu*mu)+mu*sqrt(1.0-mun*mun)*cos(phi-phin)) / sqrt(1.0- cos(alpha*alpha));
			} else {
				// Transmission
				cosia = (1.0*mu*sqrt(1.0-mun*mun)-mun*sqrt(1.0-mu*mu)*cos(phi-phin)) / sqrt(1.0- cos(alpha*alpha));
				cosib = (mun*sqrt(1.0-mu*mu)-mu*sqrt(1.0-mun*mun)*cos(phi-phin)) / sqrt(1.0- cos(alpha*alpha));
			}
		} else
		{ // Special case: mu=mun=1
			cosia = -1.0*cos(phi-phin);
			cosib = 1.0;
		}

		double ia,ib;
		ia=acos(cosia);
		ib=acos(cosib);

		// Construct the two matrixop rotation matrices in the rotation operator
		matrixop rota(2,4,4), rotb(2,4,4);
		// rota
		rota.set(1.0,2,0,0);
		rota.set(cos(2.0*ib),2,1,1);
		rota.set(cos(2.0*ib),2,2,2);
		rota.set(1.0,2,3,3);
		rota.set(sin(2.0*ib),2,2,1);
		rota.set(sin(-2.0*ib),2,1,2);
		// rotb
		rotb.set(1.0,2,0,0);
		rotb.set(cos(2.0*ia),2,1,1);
		rotb.set(cos(2.0*ia),2,2,2);
		rotb.set(1.0,2,3,3);
		rotb.set(sin(2.0*ia),2,2,1);
		rotb.set(sin(-2.0*ia),2,1,2);

		// Do the rotation!
		res.clear();
		res.resize(2,4,4);
		res = rota * (Pa * rotb);
	}

	void phaseFuncRotator::rotate(rtselec::rtselec RT, double mu, double mun, double alpha, 
		double phi, double phin, matrixop &res)
	{
		throw rtmath::debug::xObsolete();
		// Following Hansen 1971
		// cos alpha = +- mu*mun + sqrt(1-mu^2) sqrt(1-mun^2) cos (phi-phin)
		// Need to find rotation angles i1 and i2
		double cosia = 0;
		double cosib = 0;

		matrixop Pa(2,4,4);
		target->calc(alpha, m, x, Pa);

		// Note that in phasefunc, mu is alpha!
		if (mu != mun && mu != 1.0)
		{ // Regular case
			if (RT == rtmath::rtselec::R)
			{
				// Reflection
				cosia = (-1.0*mu*sqrt(1.0-mun*mun)-mun*sqrt(1.0-mu*mu)*cos(phi-phin)) / sqrt(1.0- cos(alpha*alpha));
				cosib = (mun*sqrt(1.0-mu*mu)+mu*sqrt(1.0-mun*mun)*cos(phi-phin)) / sqrt(1.0- cos(alpha*alpha));
			} else {
				// Transmission
				cosia = (1.0*mu*sqrt(1.0-mun*mun)-mun*sqrt(1.0-mu*mu)*cos(phi-phin)) / sqrt(1.0- cos(alpha*alpha));
				cosib = (mun*sqrt(1.0-mu*mu)-mu*sqrt(1.0-mun*mun)*cos(phi-phin)) / sqrt(1.0- cos(alpha*alpha));
			}
		} else
		{ // Special case: mu=mun=1
			cosia = -1.0*cos(phi-phin);
			cosib = 1.0;
		}

		double ia,ib;
		ia=acos(cosia);
		ib=acos(cosib);

		// Construct the two matrixop rotation matrices in the rotation operator
		matrixop rota(2,4,4), rotb(2,4,4);
		// rota
		rota.set(1.0,2,0,0);
		rota.set(cos(2.0*ib),2,1,1);
		rota.set(cos(2.0*ib),2,2,2);
		rota.set(1.0,2,3,3);
		rota.set(sin(2.0*ib),2,2,1);
		rota.set(sin(-2.0*ib),2,1,2);
		// rotb
		rotb.set(1.0,2,0,0);
		rotb.set(cos(2.0*ia),2,1,1);
		rotb.set(cos(2.0*ia),2,2,2);
		rotb.set(1.0,2,3,3);
		rotb.set(sin(2.0*ia),2,2,1);
		rotb.set(sin(-2.0*ia),2,1,2);

		// Do the rotation!
		res.clear();
		res.resize(2,4,4);
		res = rota * (Pa * rotb);
	}
	*/

}; // end rtmath

