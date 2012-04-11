#include "../rtmath/Stdafx.h"
#include <complex>
#include <cmath>
#include "../rtmath/phaseFunc.h"
#include "../rtmath/units.h"

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

	void scattMatrix::_genExtinctionMatrix(double Knn[4][4], const std::complex<double> Sn[4], double fGHz)
	{
		// TODO: make sure it's correct via testing. Am I solving for the correct matrix?


		// Convert f in GHz to Hz
		units::conv_spec cnv("GHz","Hz");
		double f = cnv.convert(fGHz);

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

	void scattMatrix::_invertS(const double Snn[4][4], const double Knn[4][4], double fGHz, std::complex<double> Sn[4])
	{
		// TODO: make sure it's correct via testing. Am I solving for the correct matrix?
		const double PI = boost::math::constants::pi<double>();

		// Using the Mueller matrix (Snn) and the Stokes Extinction Matrix (Knn), the 
		// forward-scattering amplitude matrix (Sn) may be retreived. This goes in the 
		// reverse direction from the normal procedure.
		// It cannot be done purely from the extinction matrix (S0 and S3 real values are coupled)
		// and it is _quite_ dificult from the phase matrix alone.
		units::conv_spec fc("GHz","Hz");
		const double f = fc.convert(fGHz);

		double val;

		// Imaginary parts

		val = (Knn[0][0] + Knn[0][1]) * f / (4.0 * PI);
		Sn[0].imag(val);

		val = Knn[0][0] * f / (2.0 * PI);
		val -= Sn[0].imag();
		Sn[3].imag(val);

		val = -f * (Knn[0][2] + Knn[1][2]) / (4.0 * PI);
		Sn[1].imag(val);

		val = -f * Knn[0][2] / (2.0 * PI);
		val -= Sn[1].imag();
		Sn[2].imag(val);

		// Real parts

		val = -f * (Knn[0][3] + Knn[1][3]) / (4.0 * PI);
		Sn[1].real(val);

		val = -f * Knn[1][3] / (2.0 * PI);
		val -= Sn[1].real();
		Sn[2].real(val);

		// Need Snn for the last two real components

		val = Snn[0][0] + Snn[1][0];
		val -= (Sn[1] * conj(Sn[1])).real();
		val -= Sn[0].imag() * Sn[0].imag();
		Sn[0].real(val);

		val = Snn[0][0] - Snn[1][0];
		val -= (Sn[2] * conj(Sn[2])).real();
		val -= Sn[3].imag() * Sn[3].imag();
		Sn[3].real(val);
	}

}; // end rtmath

