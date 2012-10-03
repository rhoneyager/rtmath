#include "../rtmath/Stdafx.h"
#include <complex>
#include <algorithm>
#include <cmath>
#include "../rtmath/phaseFunc.h"
#include "../rtmath/units.h"
#include <algorithm>
#include <boost/math/constants/constants.hpp>

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
		throw rtmath::debug::xUnimplementedFunction();
		//phaseFuncs::mueller
	}

	namespace phaseFuncs
	{
		void convertFtoS(const std::complex<double> f[2][2], std::complex<double> Sn[4], double phi, 
			std::complex<double> a, std::complex<double> b, std::complex<double> c, std::complex<double> d)
		{
			using namespace std;
			typedef complex<double> CD;
			complex<double> i(0,1);

			double rphi = phi * boost::math::constants::pi<double>() / 180.0;
			double cp = cos(rphi);
			double sp = sin(rphi);
			complex<double> CP(cp,0), SP(sp,0);
			Sn[0] = -i*((f[0][0]*((a*CP)+(b*SP)))
				+(f[0][1]*((c*CP)+(d*SP))));
			Sn[1] = i*((f[0][0]*((b*CP)-(a*SP)))
				+(f[0][1]*((d*CP)-(c*SP))));
			Sn[2] = i*((f[1][0]*((a*CP)+(b*SP)))
				+(f[1][1]*((c*CP)+(d*SP))));
			Sn[3] = -i*((f[1][0]*((b*CP)-(a*SP)))
				+(f[1][1]*((d*CP)-(c*SP))));
		}

		void selectMueller(const std::string &id,
			std::function<void(const std::complex<double> Sn[4], double Snn[4][4])>& f)
		{
			using namespace std;
			string tid = id;
			// Put in lower case
			std::transform(tid.begin(),tid.end(),tid.begin(),::tolower);

			if (tid == "bh" || tid == "ddscat")
				f = muellerBH;
			else if (tid == "tmatrix")
				f = muellerTMATRIX;
			else
				throw rtmath::debug::xBadInput(id.c_str()); // TODO: use another xError? Create a new one?
		}

		void muellerBH(const std::complex<double> Sn[4], double Snn[4][4])
		{
			std::complex<double> scratch;

			Snn[0][0] = 0.5 * ( (Sn[0]*conj(Sn[0])) + (Sn[1]*conj(Sn[1])) 
				+ (Sn[2]*conj(Sn[2])) + (Sn[3]*conj(Sn[3])) ).real();

			Snn[0][1] = 0.5 * ( (Sn[0]*conj(Sn[0])) - (Sn[1]*conj(Sn[1])) 
				+ (Sn[2]*conj(Sn[2])) - (Sn[3]*conj(Sn[3]))).real();

			scratch = ( (Sn[0] * (conj(Sn[1]))) + (Sn[3] * (conj(Sn[2])) ));
			Snn[0][2] = 1.0 * scratch.real();

			scratch = ( (Sn[0] * (conj(Sn[1]))) - (Sn[3] * (conj(Sn[2])) ));
			Snn[0][3] = 1.0 * scratch.imag();

			Snn[1][0] = 0.5 * ( (Sn[0]*conj(Sn[0])) + (Sn[1]*conj(Sn[1])) 
				- (Sn[2]*conj(Sn[2])) - (Sn[3]*conj(Sn[3])) ).real();

			Snn[1][1] = 0.5 * ( (Sn[0]*conj(Sn[0])) - (Sn[1]*conj(Sn[1]))
				- (Sn[2]*conj(Sn[2])) + (Sn[3]*conj(Sn[3])) ).real();

			scratch = ( (Sn[0] * (conj(Sn[1]))) - (Sn[3] * (conj(Sn[2])) ));
			Snn[1][2] = 1.0 * scratch.real();
			scratch = ( (Sn[0] * (conj(Sn[1]))) + (Sn[3] * (conj(Sn[2])) ));
			Snn[1][3] = 1.0 * scratch.imag();

			scratch = ( (Sn[0] * (conj(Sn[2]))) + (Sn[3] * (conj(Sn[1])) ));
			Snn[2][0] = 1.0 * scratch.real();
			scratch = ( (Sn[0] * (conj(Sn[2]))) - (Sn[3] * (conj(Sn[1])) ));
			Snn[2][1] = 1.0 * scratch.real();

			scratch = ( (Sn[3] * (conj(Sn[0]))) + (Sn[1] * (conj(Sn[2])) ));
			Snn[2][2] = scratch.real();
			scratch = ( (Sn[0] * (conj(Sn[3]))) + (Sn[2] * (conj(Sn[1])) ));
			Snn[2][3] = 1.0 * scratch.imag();

			scratch = ( (conj(Sn[0]) * (Sn[2])) + (conj(Sn[1]) * (Sn[3]) ));
			Snn[3][0] = 1.0 * scratch.imag();
			scratch = ( (conj(Sn[0]) * (Sn[2])) - (conj(Sn[1]) * (Sn[3]) ));
			Snn[3][1] = 1.0 * scratch.imag();

			scratch = ( (Sn[3] * (conj(Sn[0]))) - (Sn[1] * (conj(Sn[2])) ));
			Snn[3][2] = scratch.imag();
			scratch = ( (Sn[3] * (conj(Sn[0]))) - (Sn[1] * (conj(Sn[2])) ));
			Snn[3][3] = scratch.real();
		}

		void muellerTMATRIX(const std::complex<double> Sn[4], double Snn[4][4])
		{
			std::complex<double> scratch;

			Snn[0][0] = 0.5 * ( (Sn[0]*conj(Sn[0])) + (Sn[1]*conj(Sn[1])) 
				+ (Sn[2]*conj(Sn[2])) + (Sn[3]*conj(Sn[3])) ).real();

			Snn[0][1] = 0.5 * ( (Sn[0]*conj(Sn[0])) - (Sn[1]*conj(Sn[1])) 
				+ (Sn[2]*conj(Sn[2])) - (Sn[3]*conj(Sn[3]))).real();

			scratch = ( (Sn[3] * (conj(Sn[2]))) + (Sn[0] * (conj(Sn[1])) ));
			Snn[0][2] = -1.0 * scratch.real();

			scratch = ( (Sn[0] * (conj(Sn[1]))) - (Sn[3] * (conj(Sn[2])) ));
			Snn[0][3] = 1.0 * scratch.imag();

			Snn[1][0] = 0.5 * ( (Sn[0]*conj(Sn[0])) + (Sn[1]*conj(Sn[1])) 
				- (Sn[2]*conj(Sn[2])) - (Sn[3]*conj(Sn[3])) ).real();

			Snn[1][1] = 0.5 * ( (Sn[0]*conj(Sn[0])) - (Sn[1]*conj(Sn[1]))
				- (Sn[2]*conj(Sn[2])) + (Sn[3]*conj(Sn[3])) ).real();

			scratch = ( (Sn[3] * (conj(Sn[2]))) - (Sn[0] * (conj(Sn[1])) ));
			Snn[1][2] = 1.0 * scratch.real();
			scratch = ( (Sn[0] * (conj(Sn[1]))) + (Sn[3] * (conj(Sn[2])) ));
			Snn[1][3] = 1.0 * scratch.imag();

			scratch = ( (Sn[0] * (conj(Sn[2]))) + (Sn[3] * (conj(Sn[1])) ));
			Snn[2][0] = -1.0 * scratch.real();
			scratch = ( -(Sn[0] * (conj(Sn[2]))) + (Sn[3] * (conj(Sn[1])) ));
			Snn[2][1] = 1.0 * scratch.real();

			scratch = ( (Sn[0] * (conj(Sn[3]))) + (Sn[1] * (conj(Sn[2])) ));
			Snn[2][2] = scratch.real();
			scratch = ( (Sn[0] * (conj(Sn[3]))) + (Sn[2] * (conj(Sn[1])) ));
			Snn[2][3] = -1.0 * scratch.imag();

			scratch = ( (conj(Sn[0]) * (Sn[2])) + (conj(Sn[1]) * (Sn[3]) ));
			Snn[3][0] = 1.0 * scratch.imag();
			scratch = ( (conj(Sn[0]) * (Sn[2])) - (conj(Sn[1]) * (Sn[3]) ));
			Snn[3][1] = 1.0 * scratch.imag();

			scratch = ( (Sn[3] * (conj(Sn[0]))) - (Sn[1] * (conj(Sn[2])) ));
			Snn[3][2] = scratch.imag();
			scratch = ( (Sn[3] * (conj(Sn[0]))) - (Sn[1] * (conj(Sn[2])) ));
			Snn[3][3] = scratch.real();
		}

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

