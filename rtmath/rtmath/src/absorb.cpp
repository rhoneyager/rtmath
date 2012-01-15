#include <cmath>
#include "../rtmath/absorb.h"

namespace rtmath {
	namespace absorb {

		absorb::~absorb()
		{
			// Does nothing. Entry exists to ensure 
			// that vtable builds here.
		}

		double absorb::T(double tau)
		{
			// Quick and easy return of 
			// the transmittance
			return exp(-tau);
		}

		double absorb::_wvtofreq(double wvnum)
		{
			// Really should use full speed of light
			// TODO: replace definition
			return wvnum * 2.997925e8;
		}

		// TODO: rewrite the function call to take an entire atmospheric layer's parameters, such as temp, pressure, HUMIDITY and others for use in the absorption calculation
		double o2abs::tau(double T, double p, double wvnum) const
		{
		}

		// N2 collision absorption
		// P. Rosenkranz (1998)
		double n2abs::tau(double T, double p, double wvnum) const
		{
			double th = 300./T;
			double f = _wvtofreq(wvnum);
			return 6.4e-14*p*p*f*f*pow(th,3.55);
		}

		// Collision-induced absorption by Pardo et al. (2000)
		// Pardo, J. R., E. Serabyn, and J. Cernicharo, 
		//  Submillimeter atmospheric transmission measurements
		// on Mauna Kea during extremely dry El Nino
		// consitions: Implications for broadband opacity 
		// contributions. J.Q.S.R.T., 67, 169-180, 2000.
		// 1.29 times of N2-N2 collision absorption to account
		// for N2-O2 and O2-O2 collisions
		double collide::tau(double T, double p, double wvnum) const
		{
			const double a1 = 7.7e-10;
			const double a2 = 1.0e-13;
			const double b = 1.7;
			const double c1 = 1.5e-3;
			const double c2 = 1.0e-4;
			const double d = 60.0;
			double th = 300.0/T;
			double f2 = wvnum * wvnum;
			double an2 = (a1*exp(-c1*th*f2)+a2*exp(-c2*th*f2)*(d*d+f2))*pow(th,b);
			double abscollide = 0.65*(P/1013.)*(P/1013.)*th*th*2.0*f2*an2;
			// abscollide has units of 1/cm. convert to 1/km
			abscollide *= 1.e5;
			return abscollide;
		}
	};
}; // end namespace rtmath

