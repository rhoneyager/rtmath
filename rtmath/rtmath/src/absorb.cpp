#include "../rtmath/Stdafx.h"
#include <cmath>
#include "../rtmath/absorb.h"
#include "../rtmath/error/error.h"

namespace rtmath {
	namespace atmos {

		double abs_N2::deltaTau(double nu) const
		{
			double th = 300./(*_T);
			double f = _wvtofreq(nu);
			return 6.4e-14*(*_p)*(*_p)*f*f*pow(th,3.55);
		}

		double collide::deltaTau(double f) const
		{
			const double a1 = 7.7e-10;
			const double a2 = 1.0e-13;
			const double b = 1.7;
			const double c1 = 1.5e-3;
			const double c2 = 1.0e-4;
			const double d = 60.0;
			double th = 300.0/(*_T);
			double nu = _freqtowv(f);
			double f2 = nu * nu;
			double an2 = (a1*exp(-c1*th*f2)+a2*exp(-c2*th*f2)*(d*d+f2))*pow(th,b);
			double abscollide = 0.65*((*_p)/1013.)*((*_p)/1013.)*th*th*2.0*f2*an2;
			// abscollide has units of 1/cm. convert to 1/km
			abscollide *= 1.e5;
			// But, we have _dz (in km)!
			return abscollide* (*_dz); // TODO: check absorption coefficient results
		}

		double abs_H2O::deltaTau(double f) const
		{
			// _T is in Kelvin
			// _p is in mb
			// rho is water vapor density (in G/m^3)
			// abh2o is output in nepers / km
			// abh20, given knowledge of dz (in km), will calculate tau

			// Now, for a more-or-less direct translation of Liu's code
			const int nlines = 15;
			// Line frequencies
			const double fl[] = { 22.2351, 183.3101, 321.2256, 325.1529, 
				380.1974, 439.1508, 443.0183, 448.0011, 470.8890, 474.6891,  
				488.4911, 556.9360, 620.7008, 752.0332, 916.1712 };
			// Line intensities at 300 K
			const double S1[] = { .1310E-13, .2273E-11, .8036E-13, .2694E-11,
				.2438E-10, .2179E-11, .4624E-12, .2562E-10, .8369E-12,
				.3263E-11, .6659E-12, .1531E-08, .1707E-10, .1011E-08, .4227E-10 };
			// T Coeff of intensities
			const double B2[] = { 2.144, .668, 6.179, 1.541, 1.048,
				3.595, 5.048, 1.405, 3.597, 2.379, 2.852, .159, 2.391, 
				.396, 1.441 };
			// Air-Broadened width parameters at 300 K
			const double W3[] = { .00281, .00281, .0023, .00278, .00287,
				.0021, .00186, .00263, .00215, .00236, .0026, .00321,
				.00244, .00306, .00267 };
			// T-exponent of air broadening
			const double X[] = { .69, .64, .67, .68, .54, .63, .60, .66, .66, 
				.65, .69, .69, .71, .68, .70 };
			// Self-broadened width parameters at 300 K
			const double WS[] = { .01349, .01491, .0108, .0135, .01541,
				.0090, .00788, .01275, .00983, .01095, .01313, .01320,
				.01140, .01253, .01275 };
			// T-exponent of self-broadening
			const double XS[] = { .61, .85, .54, .74, .89, .52, .50, .67, .65, 
				.64, .72, 1.0, .68, .84, .78 };


			//double rho = absorber::_rhoWVap(*_p,*_T,_psfrac);
			double rho = _wvden; // Well, I already calculated it anyways.....
			if (rho < 0) return 0;

			double pvap = rho * (*_T) / 217.;
			double pda = (*_p) - pvap;
			double den = 3.335e16*rho;
			double ti = 300./(*_T);
			double ti2 = pow(ti,2.5);

			// Water vapor continuum terms
			double con = (5.43e-10*pda*pow(ti,3) + 1.8e-8*pvap*pow(ti,7.5))
				* pvap*f*f;

			// Add resonances
			double sum = 0;
			double df[2] = {0, 0};
			for (size_t i=0;i<nlines;i++)
			{
				double width = 0, s = 0, base = 0;
				width = W3[i]*pda*pow(ti,X[i]) + WS[i]*pvap*pow(ti,XS[i]);
				double wsq = width*width;
				s = S1[i]*ti2*exp(B2[i]*(1.0-ti));
				df[0] = f - fl[i];
				df[1] = f + fl[i];
				// Use Clough's definition of local line contribution
				base = width / (562500.0 + wsq);
				double res = 0;
				for (size_t j=0; j<2; j++)
				{
					if(abs(df[j] < 750)) 
						res += width/(df[j]*df[j]+wsq) - base;
				}
				sum += s*res*pow(f/fl[i],2);
			}

			// abh2o needs to be converted
			// it is in nepers per km, but I want dB. I also have _dz.
			double abh2o = 0.3183e-4*den*sum + con;
			return abh2o * (*_dz) * 20.0*log10(M_E);
		}

		double abs_O2::deltaTau(double f) const
		{
			// _T is temp in Kelvin
			// _p is pressure in mb
			// vapden is water vapor density in g/m^3
			// f is freq in GHz

			double vapden = _wvden; // NOTE: this is set during initialization pass 2

			const double F[] = { 118.7503, 56.2648, 62.4863, 58.4466,
				60.3061, 59.5910, 59.1642, 60.4348, 58.3239, 61.1506,
				57.6125, 61.8002, 56.9682, 62.4112, 56.3634, 62.9980, 
				55.7838, 63.5685, 55.2214, 64.1278, 54.6712, 64.6789, 
				54.1300, 65.2241, 53.5957, 65.7648, 53.0669, 66.3021, 
				52.5424, 66.8368, 52.0214, 67.3696, 51.5034, 67.9009, 
				368.4984, 424.7632, 487.2494, 715.3931, 773.8397, 834.1458 };
			const double S300[] = { .2936E-14,.8079E-15, .2480E-14,.2228E-14,
				.3351E-14,.3292E-14, .3721E-14, .3891E-14,
				.3640E-14,.4005E-14, .3227E-14, .3715E-14,
				.2627E-14,.3156E-14, .1982E-14, .2477E-14,
				.1391E-14,.1808E-14, .9124E-15, .1230E-14,
				.5603E-15,.7842E-15, .3228E-15, .4689E-15,
				.1748E-15,.2632E-15, .8898E-16, .1389E-15,
				.4264E-16,.6899E-16, .1924E-16, .3229E-16,
				.8191E-17,.1423E-16, .6494E-15, .7083E-14, 
				.3025E-14,.1835E-14, .1158E-13, .3993E-14 };
			const double BE[] = { .009,.015, .083,.084, .212, .212, .391, .391, .626, .626,
				.915, .915, 1.260, 1.260, 1.660,1.665, 2.119,2.115, 2.624, 2.625,
				3.194, 3.194, 3.814, 3.814, 4.484, 4.484, 5.224, 5.224, 6.004, 6.004, 6.844, 6.844,
				7.744, 7.744, .048, .044, .049, .145, .141, .145};
			// Widths in MHZ/mb
			const double WB300 = 0.56;
			const double X = 0.8;
			const double W300[] = { 1.63, 1.646, 1.468, 1.449, 1.382, 1.360,
				1.319, 1.297, 1.266, 1.248, 1.221, 1.207, 1.181, 1.171,
				1.144, 1.139, 1.110, 1.108, 1.079, 1.078, 1.05, 1.05,
				1.02, 1.02, 1.00, 1.00, 0.97, .97, .94, .94, .92, .92, .89, .89, 
				1.92, 1.92, 1.92, 1.81, 1.81, 1.81};
			const double Y300[] = { -0.0233,  0.2408, -0.3486,  0.5227,
				-0.5430,  0.5877, -0.3970,  0.3237, -0.1348,  0.0311,
				0.0725, -0.1663,  0.2832, -0.3629,  0.3970, -0.4599,
				0.4695, -0.5199,  0.5187, -0.5597,  0.5903, -0.6246,
				0.6656, -0.6942,  0.7086, -0.7325,  0.7348, -0.7546,
				0.7702, -0.7864,  0.8083, -0.8210,  0.8439, -0.8529, 
				0, 0, 0, 0, 0, 0};
			const double V[] = { 0.0079, -0.0978,  0.0844, -0.1273,
				0.0699, -0.0776,  0.2309, -0.2825,  0.0436, -0.0584,
				0.6056, -0.6619,  0.6451, -0.6759,  0.6547, -0.6675,
				0.6135, -0.6139,  0.2952, -0.2895,  0.2654, -0.2590,
				0.3750, -0.3680,  0.5085, -0.5002,  0.6206, -0.6091,
				0.6526, -0.6393,  0.6640, -0.6475,  0.6729, -0.6545, 
				0, 0, 0, 0, 0, 0};

			// Finally...
			double th = 300./(*_T);
			double th1 = th - 1.;
			double b = pow(th,X);
			
			double preswv = vapden * (*_T)/217.;
			double presda = (*_p) - preswv;
			double den = 0.001*(presda*b + 1.1*preswv*th);
			double dens = 0.001*(presda + 1.1*preswv)*th;
			double dfnr = WB300*den;
			double sum = 1.6e-17*f*f*dfnr/(th*(f*f+dfnr*dfnr));
			for (size_t k=0;k<40;k++)
			{
				double df;
				if (k==0)
					df = W300[0] * dens;
				else
					df = W300[k] * den;
				double y = 0.001*(*_p)*b*(Y300[k]+V[k]*th1);
				double str = S300[k]*exp(-BE[k]*th1);
				double sf1 = (df + (f-F[k])*y)/(pow(f-F[k],2)+df*df);
				double sf2 = (df - (f+F[k])*y)/(pow(f+F[k],2)+df*df);
				double s = str*(sf1+sf2)*(pow(f/F[k],2));
				sum += s;
			}
			double o2abs = 0.5034e12*sum*presda*pow(th,3)/3.14159;
			// o2abs needs to be converted
			// it is in nepers per km, but I want dB. I also have _dz.
			return o2abs * (*_dz) * 20.0*log10(M_E);
		}
	};
}; // end namespace rtmath

