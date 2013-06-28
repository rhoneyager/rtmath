#include "../rtmath/Stdafx.h"
#include <complex>
#include <algorithm>
#include <cmath>
#include <algorithm>
#include <boost/math/constants/constants.hpp>
#include "../rtmath/phaseFunc.h"
#include "../rtmath/units.h"
#include "../rtmath/error/debug.h"

namespace rtmath {

	namespace phaseFuncs
	{
		void convertFtoS(const Eigen::Matrix2cd &f, Eigen::Matrix2cd& Sn, double phi, 
			std::complex<double> a, std::complex<double> b, std::complex<double> c, std::complex<double> d)
		{
			using namespace std;
			typedef complex<double> CD;
			complex<double> i(0,1);

			double rphi = phi * boost::math::constants::pi<double>() / 180.0;
			double cp = cos(rphi);
			double sp = sin(rphi);
			complex<double> CP(cp,0), SP(sp,0);
			Sn(0,0) = -i*((f(0,0)*((a*CP)+(b*SP)))
				+(f(0,1)*((c*CP)+(d*SP))));
			Sn(0,1) = i*((f(0,0)*((b*CP)-(a*SP)))
				+(f(0,1)*((d*CP)-(c*SP))));
			Sn(1,0) = i*((f(1,0)*((a*CP)+(b*SP)))
				+(f(1,1)*((c*CP)+(d*SP))));
			Sn(1,1) = -i*((f(1,0)*((b*CP)-(a*SP)))
				+(f(1,1)*((d*CP)-(c*SP))));
		}

		void selectMueller(const std::string &id,
			std::function<void(const Eigen::Matrix2cd&, Eigen::Matrix4d&)>& f)
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

		void muellerBH(const Eigen::Matrix2cd& Sn, Eigen::Matrix4d& Snn)
		{
			std::complex<double> scratch;

			Snn(0,0) = 0.5 * ( (Sn(0,0)*conj(Sn(0,0))) + (Sn(0,1)*conj(Sn(0,1))) 
				+ (Sn(1,0)*conj(Sn(1,0))) + (Sn(1,1)*conj(Sn(1,1))) ).real();

			Snn(0,1) = 0.5 * ( (Sn(0,0)*conj(Sn(0,0))) - (Sn(0,1)*conj(Sn(0,1))) 
				+ (Sn(1,0)*conj(Sn(1,0))) - (Sn(1,1)*conj(Sn(1,1)))).real();

			scratch = ( (Sn(0,0) * (conj(Sn(0,1)))) + (Sn(1,1) * (conj(Sn(1,0))) ));
			Snn(0,2) = 1.0 * scratch.real();

			scratch = ( (Sn(0,0) * (conj(Sn(0,1)))) - (Sn(1,1) * (conj(Sn(1,0))) ));
			Snn(0,3) = 1.0 * scratch.imag();

			Snn(1,0) = 0.5 * ( (Sn(0,0)*conj(Sn(0,0))) + (Sn(0,1)*conj(Sn(0,1))) 
				- (Sn(1,0)*conj(Sn(1,0))) - (Sn(1,1)*conj(Sn(1,1))) ).real();

			Snn(1,1) = 0.5 * ( (Sn(0,0)*conj(Sn(0,0))) - (Sn(0,1)*conj(Sn(0,1)))
				- (Sn(1,0)*conj(Sn(1,0))) + (Sn(1,1)*conj(Sn(1,1))) ).real();

			scratch = ( (Sn(0,0) * (conj(Sn(0,1)))) - (Sn(1,1) * (conj(Sn(1,0))) ));
			Snn(1,2) = 1.0 * scratch.real();
			scratch = ( (Sn(0,0) * (conj(Sn(0,1)))) + (Sn(1,1) * (conj(Sn(1,0))) ));
			Snn(1,3) = 1.0 * scratch.imag();

			scratch = ( (Sn(0,0) * (conj(Sn(1,0)))) + (Sn(1,1) * (conj(Sn(0,1))) ));
			Snn(2,0) = 1.0 * scratch.real();
			scratch = ( (Sn(0,0) * (conj(Sn(1,0)))) - (Sn(1,1) * (conj(Sn(0,1))) ));
			Snn(2,1) = 1.0 * scratch.real();

			scratch = ( (Sn(1,1) * (conj(Sn(0,0)))) + (Sn(0,1) * (conj(Sn(1,0))) ));
			Snn(2,2) = scratch.real();
			scratch = ( (Sn(0,0) * (conj(Sn(1,1)))) + (Sn(1,0) * (conj(Sn(0,1))) ));
			Snn(2,3) = 1.0 * scratch.imag();

			scratch = ( (conj(Sn(0,0)) * (Sn(1,0))) + (conj(Sn(0,1)) * (Sn(1,1)) ));
			Snn(3,0) = 1.0 * scratch.imag();
			scratch = ( (conj(Sn(0,0)) * (Sn(1,0))) - (conj(Sn(0,1)) * (Sn(1,1)) ));
			Snn(3,1) = 1.0 * scratch.imag();

			scratch = ( (Sn(1,1) * (conj(Sn(0,0)))) - (Sn(0,1) * (conj(Sn(1,0))) ));
			Snn(3,2) = scratch.imag();
			scratch = ( (Sn(1,1) * (conj(Sn(0,0)))) - (Sn(0,1) * (conj(Sn(1,0))) ));
			Snn(3,3) = scratch.real();
		}

		void muellerTMATRIX(const Eigen::Matrix2cd& Sn, Eigen::Matrix4d& Snn)
		{
			std::complex<double> scratch;

			Snn(0,0) = 0.5 * ( (Sn(0,0)*conj(Sn(0,0))) + (Sn(0,1)*conj(Sn(0,1))) 
				+ (Sn(1,0)*conj(Sn(1,0))) + (Sn(1,1)*conj(Sn(1,1))) ).real();

			Snn(0,1) = 0.5 * ( (Sn(0,0)*conj(Sn(0,0))) - (Sn(0,1)*conj(Sn(0,1))) 
				+ (Sn(1,0)*conj(Sn(1,0))) - (Sn(1,1)*conj(Sn(1,1)))).real();

			scratch = ( (Sn(1,1) * (conj(Sn(1,0)))) + (Sn(0,0) * (conj(Sn(0,1))) ));
			Snn(0,2) = -1.0 * scratch.real();

			scratch = ( (Sn(0,0) * (conj(Sn(0,1)))) - (Sn(1,1) * (conj(Sn(1,0))) ));
			Snn(0,3) = 1.0 * scratch.imag();

			Snn(1,0) = 0.5 * ( (Sn(0,0)*conj(Sn(0,0))) + (Sn(0,1)*conj(Sn(0,1))) 
				- (Sn(1,0)*conj(Sn(1,0))) - (Sn(1,1)*conj(Sn(1,1))) ).real();

			Snn(1,1) = 0.5 * ( (Sn(0,0)*conj(Sn(0,0))) - (Sn(0,1)*conj(Sn(0,1)))
				- (Sn(1,0)*conj(Sn(1,0))) + (Sn(1,1)*conj(Sn(1,1))) ).real();

			scratch = ( (Sn(1,1) * (conj(Sn(1,0)))) - (Sn(0,0) * (conj(Sn(0,1))) ));
			Snn(1,2) = 1.0 * scratch.real();
			scratch = ( (Sn(0,0) * (conj(Sn(0,1)))) + (Sn(1,1) * (conj(Sn(1,0))) ));
			Snn(1,3) = 1.0 * scratch.imag();

			scratch = ( (Sn(0,0) * (conj(Sn(1,0)))) + (Sn(1,1) * (conj(Sn(0,1))) ));
			Snn(2,0) = -1.0 * scratch.real();
			scratch = ( -(Sn(0,0) * (conj(Sn(1,0)))) + (Sn(1,1) * (conj(Sn(0,1))) ));
			Snn(2,1) = 1.0 * scratch.real();

			scratch = ( (Sn(0,0) * (conj(Sn(1,1)))) + (Sn(0,1) * (conj(Sn(1,0))) ));
			Snn(2,2) = scratch.real();
			scratch = ( (Sn(0,0) * (conj(Sn(1,1)))) + (Sn(1,0) * (conj(Sn(0,1))) ));
			Snn(2,3) = -1.0 * scratch.imag();

			scratch = ( (conj(Sn(0,0)) * (Sn(1,0))) + (conj(Sn(0,1)) * (Sn(1,1)) ));
			Snn(3,0) = 1.0 * scratch.imag();
			scratch = ( (conj(Sn(0,0)) * (Sn(1,0))) - (conj(Sn(0,1)) * (Sn(1,1)) ));
			Snn(3,1) = 1.0 * scratch.imag();

			scratch = ( (Sn(1,1) * (conj(Sn(0,0)))) - (Sn(0,1) * (conj(Sn(1,0))) ));
			Snn(3,2) = scratch.imag();
			scratch = ( (Sn(1,1) * (conj(Sn(0,0)))) - (Sn(0,1) * (conj(Sn(1,0))) ));
			Snn(3,3) = scratch.real();
		}

		/// \todo Test rtmath::phasefuncs::invertS
		#pragma message("phaseFunc.cpp: Test invertS")
		void invertS(const Eigen::Matrix4d &Snn, const Eigen::Matrix4d &Knn, double fGHz, Eigen::Matrix2cd& Sn)
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

			val = (Knn(0,0) + Knn(0,1)) * f / (4.0 * PI);
			Sn(0,0).imag(val);

			val = Knn(0,0) * f / (2.0 * PI);
			val -= Sn(0,0).imag();
			Sn(1,1).imag(val);

			val = -f * (Knn(0,2) + Knn(1,2)) / (4.0 * PI);
			Sn(0,1).imag(val);

			val = -f * Knn(0,2) / (2.0 * PI);
			val -= Sn(0,1).imag();
			Sn(1,0).imag(val);

			// Real parts

			val = -f * (Knn(0,3) + Knn(1,3)) / (4.0 * PI);
			Sn(0,1).real(val);

			val = -f * Knn(1,3) / (2.0 * PI);
			val -= Sn(0,1).real();
			Sn(1,0).real(val);

			// Need Snn for the last two real components

			val = Snn(0,0) + Snn(1,0);
			val -= (Sn(0,1) * conj(Sn(0,1))).real();
			val -= Sn(0,0).imag() * Sn(0,0).imag();
			Sn(0,0).real(val);

			val = Snn(0,0) - Snn(1,0);
			val -= (Sn(1,0) * conj(Sn(1,0))).real();
			val -= Sn(1,1).imag() * Sn(1,1).imag();
			Sn(1,1).real(val);
		}

#pragma message("phaseFunc.cpp: Test genExtinctionMatrix")
		/// \todo Test rtmath::phaseFuncs::genExtinctionMatrix
		void genExtinctionMatrix(Eigen::Matrix4d &Knn, const Eigen::Matrix2cd &Sn, double fGHz)
		{
			// TODO: make sure it's correct via testing. Am I solving for the correct matrix?


			// Convert f in GHz to Hz
			units::conv_spec cnv("GHz","Hz");
			double f = cnv.convert(fGHz);

			// Do the diagonals first
			for (size_t i=0;i<4;i++)
			{
				Knn(i,i) = (Sn(0,0) + Sn(1,1)).imag();
			}

			// And the remaining 12 (really duplicates, so only six)
			for (size_t i=0; i<4; i++)
				for (size_t j=0; j<4; j++)
				{
					if (i == 0 && j == 1) Knn(i,j) = (Sn(0,0) - Sn(1,1)).imag();
					if (i == 0 && j == 2) Knn(i,j) = -(Sn(0,1) + Sn(1,0)).imag();
					if (i == 0 && j == 3) Knn(i,j) = (-Sn(0,1) + Sn(1,0)).real();
					if (i == 1 && j == 2) Knn(i,j) = (Sn(1,0) - Sn(0,1)).imag();
					if (i == 1 && j == 3) Knn(i,j) = -(-Sn(0,1) + Sn(1,0)).real();
					if (i == 2 && j == 3) Knn(i,j) = (Sn(1,1) - Sn(0,0)).real();

					if (j == 0 && i == 1) Knn(i,j) = (Sn(0,0) - Sn(1,1)).imag(); // Same
					if (j == 0 && i == 2) Knn(i,j) = -(Sn(0,1) + Sn(1,0)).imag();
					if (j == 0 && i == 3) Knn(i,j) = (-Sn(0,1) + Sn(1,0)).real();
					if (j == 1 && i == 2) Knn(i,j) = -(Sn(1,0) - Sn(0,1)).imag(); // Negative
					if (j == 1 && i == 3) Knn(i,j) = (-Sn(0,1) + Sn(1,0)).real();
					if (j == 2 && i == 3) Knn(i,j) = -(Sn(1,1) - Sn(0,0)).real();
				}

				// Go back and multiply by f
				for (size_t i=0; i<4;i++)
					for (size_t j=0; j<4; j++)
						Knn(i,j) *= f;
		}

	}


}

