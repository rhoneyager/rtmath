/// \brief Provides silo file IO
#define _SCL_SECURE_NO_WARNINGS

#include <string>
#include <boost/math/constants/constants.hpp>

#include "../../rtmath/rtmath/defs.h"
#include "../../rtmath/rtmath/refract.h"
#include "../../rtmath/rtmath/phaseFunc.h"
#include "../../rtmath/rtmath/ddscat/shapefile.h"
#include "../../rtmath/rtmath/ddscat/shapestats.h"
#include "../../rtmath/rtmath/ddscat/ddOutput.h"
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

			namespace Rayleigh_2 {
				void doCrossSection(
					const rtmath::phaseFuncs::pf_class_registry::setup &s,
					const rtmath::phaseFuncs::pf_class_registry::inputParamsPartial& i,
					rtmath::phaseFuncs::pf_class_registry::cross_sections& c)
				{
					using namespace ::rtmath::phaseFuncs;
					const double pi = boost::math::constants::pi<double>();

					// First, scale the effective radius and refractive index?
					double scaledAeff = i.aeff;
					if (i.aeff_rescale)
					{
						if (i.aeff_version ==
							pf_class_registry::inputParamsPartial::aeff_version_type::EQUIV_V_SPHERE)
						{
							double scaledVolume = pow(i.aeff, 3.0);
							scaledVolume /= i.vFrac;
							scaledAeff = pow(scaledVolume, 1. / 3.);
						}
						else {
							double scaledSA = pow(i.aeff, 2.0);
							scaledSA /= i.vFrac;
							scaledAeff = pow(scaledSA, 0.5);
						}
					}

					std::complex<double> mRes = i.m;
					//std::complex<double> mAir(1.0, 0);
					//i.rmeth(i.m, mAir, i.vFrac, mRes);

					// Perform the calculation
					double ar = i.eps;
					const double k = 2. * pi / s.wavelength;
					const double size_p = 2. * pi * scaledAeff / s.wavelength;
					float qext = -1, qsca = -1, qback = -1, gsca = -1;
					const double f = i.vFrac;
					const double V = 4. / 3. * pi * std::pow(i.aeff, 3.); //i.aeff
					const double dmax = 2. * std::pow(V*3./(4.*pi*ar),1./3.); // oblate particles
					const double d = ar * dmax; // d is the vertical dimension, d is the horizontal dimension.

					try {
						std::complex<double> K = ((i.m*i.m) - 1.) / ((i.m*i.m) + 2.);
						std::complex<double> Km = ((mRes*mRes) - 1.) / ((mRes*mRes) + 2.);
						double K2 = pow(abs(Km),2.);
						//qext = 4. * size_p * (-1. * d).imag();
						//qsca = 8. * std::pow(size_p, 4.) / 3.
						//	* (d * conj(d)).real();
						qback = 9. * V * V * K2 * pow(k,4.)/(4.*pi);
						qback /= pi * pow(i.aeff,2.);
						gsca = 0;

						double C_sphere = pi * pow(scaledAeff, 2.0);
						c.Qsca = -1;
						c.Qbk = qback;
						c.g = 0;
						c.Qext = -1;
						c.Qabs = -1;
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


