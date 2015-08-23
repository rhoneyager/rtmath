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

			namespace RG {
				void doCrossSection(
					const rtmath::phaseFuncs::pf_class_registry::setup &s,
					const rtmath::phaseFuncs::pf_class_registry::inputParamsPartial& i,
					rtmath::phaseFuncs::pf_class_registry::cross_sections& c)
				{
					using namespace ::rtmath::phaseFuncs;
					const double pi = boost::math::constants::pi<double>();

					// Force the dielectric constant of solid ice
					std::complex<double> mRes = i.m;
					//std::complex<double> mAir(1.0, 0);
					//i.rmeth(i.m, mAir, i.vFrac, mRes);

					// Expressed in terms of the max diameter
					// ar gives oblate/prolate indicator.

					// Perform the calculation
					double ar = i.eps;
					bool prolate = false;
					if (ar > 1.0) {
						prolate = true;
						ar = 1./ar;
					}
					double Dlong = i.maxDiamFull;
					double Dshort = Dlong * ar;

					const double k = 2. * pi / s.wavelength;
					float qext = -1, qsca = -1, qback = -1, gsca = -1;
					const double f = i.vFrac;
					// V is volume of solid ice in particle
					const double V = 4. / 3. * pi * std::pow(i.aeff, 3.); //i.aeff, scaledAeff

					try {
						std::complex<double> K = ((i.m*i.m) - 1.) / ((i.m*i.m) + 2.);
						std::complex<double> Km = ((mRes*mRes) - 1.) / ((mRes*mRes) + 2.);
						double K2 = (Km * conj(Km)).real();
						qext = -1;
						qsca = -1;
						qback = (float) K2  * 81./4. * V * V
							/ (pi * k * k * pow(Dshort,6.));
						qback *= pow(sin(k*Dshort) - (k*Dshort*cos(k*Dshort)), 2.);
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


