/// \brief Provides silo file IO
#define _SCL_SECURE_NO_WARNINGS

#include <string>
#include <boost/math/constants/constants.hpp>

#include "../../rtmath/rtmath/defs.h"
#include "../../rtmath/rtmath/refract.h"
#include "../../rtmath/rtmath/phaseFunc.h"
//#include "../../rtmath/rtmath/ddscat/shapefile.h"
//#include "../../rtmath/rtmath/ddscat/shapestats.h"
//#include "../../rtmath/rtmath/ddscat/ddOutput.h"
#include "../../rtmath/rtmath/conversions/convertLength.h"
#include "../../rtmath/rtmath/plugin.h"
#include <Ryan_Debug/debug.h>
#include <Ryan_Debug/error.h>

#include "plugin-nondda-misc.h"

namespace rtmath
{
	namespace plugins
	{
		namespace nondda_misc
		{

			namespace HRGi {
				void doCrossSection(
					const rtmath::phaseFuncs::pf_class_registry::setup &s,
					const rtmath::phaseFuncs::pf_class_registry::inputParamsPartial& i,
					rtmath::phaseFuncs::pf_class_registry::cross_sections& c)
				{
					using namespace ::rtmath::phaseFuncs;
					const double pi = boost::math::constants::pi<double>();

					// First, scale the effective radius and refractive index?
					double scaledAeff = i.aeff;

					// Force refractive index of ice
					std::complex<double> mRes = i.m;
					//std::complex<double> mAir(1.0, 0);
					//i.rmeth(i.m, mAir, i.vFrac, mRes);

					// Perform the calculation
					double ar = 1.;
					bool prolate = false;
					if (ar > 1.0) {
						prolate = true;
						ar = 1./ar;
					}
					//double Dlong = i.maxDiamFull;
					//double Dshort = Dlong * ar;
					using namespace rtmath::units;
					using namespace rtmath::units::keywords;
					double Dshort = convertLength(
						_in_length_value = i.maxDiamFull,
						_in_length_type = "Max_Diameter",
						_out_length_type = "Mean_Diameter",
						_ar = ar);

					const double k = 2. * pi / s.wavelength;
					const double size_p = k * Dshort;
					float qext = -1, qsca = -1, qback = -1, gsca = -1;
					const double f = i.vFrac;
					const double V = 4. / 3. * pi * std::pow(i.aeff, 3.); //i.aeff, scaledAeff
					//const double dmax = 2. * std::pow(V*3./(4.*pi*ar),1./3.); // oblate particles
					//const double d = ar * dmax; // d is the vertical dimension, d is the horizontal dimension.

					const double gamma = 5./3., beta = 0.23, kappa = 0.19;

					try {
						std::complex<double> K = ((i.m*i.m) - 1.) / ((i.m*i.m) + 2.);
						std::complex<double> Km = ((mRes*mRes) - 1.) / ((mRes*mRes) + 2.);
						double K2 = pow(abs(Km),2.);
						qext = -1;
						qsca = -1;
						double prefactor = 9. * pi * pow(k,4.)* K2 * V * V / 16.;
						double term1 = cos(size_p) * ( (1.+kappa/3.)*(1./(2.*size_p+pi) - 1./(2.*size_p-pi))
								- kappa * (1./(2.*size_p+3*pi) - 1./(2.*size_p - 3.*pi)));
						term1 *= term1;
						int jmax = (int) ( 5.*size_p / pi) + 1;
						qback = 0;
						for (int j=1; j <= jmax; ++j) {
							qback += pow(2.*j,-gamma) * sin(size_p) * sin(size_p)
								* ( pow(1./(2.*(size_p+(pi*j))),2.)
								+ pow(1./(2.*(size_p - (pi*j))),2.) );
						}
						qback *= beta;
						qback += term1;
						qback *= prefactor;
						qback /= pi * pow(i.aeff,2.);
						gsca = -1;

						c.Qsca_iso = -1; // qsca;// *pow(scaledAeff / i.aeff, 2.);
						c.Qext_iso = -1;// qext;// *pow(scaledAeff / i.aeff, 2.);
						c.Qabs_iso = -1; // c.Qext_iso - c.Qsca_iso;
						c.g_iso = -1;// gsca;
						c.Qbk_iso = qback; // * pow(scaledAeff / i.aeff, 2.);

						c.Qsca = c.Qsca_iso;
						c.Qbk = c.Qbk_iso;
						c.g = c.g_iso;
						c.Qext = c.Qext_iso;
						c.Qabs = c.Qabs_iso;

						/// iso values are validated with solid spheres and soft spheres using liu code
						/// \todo need to validate with ellipsoids

						//std::cerr << c.Qabs_iso << "\t" << c.Qsca_iso << "\t" << c.Qext_iso << "\t" << c.Qbk_iso << std::endl;
					}
					catch (...) {
						//std::cerr << "\t" << t.what() << std::endl;
						RDthrow(Ryan_Debug::error::xOtherError()) 
							<< Ryan_Debug::error::otherErrorText("A rayleigh-gans error has occurred");
					}
				}
			}
		}
	}
}


