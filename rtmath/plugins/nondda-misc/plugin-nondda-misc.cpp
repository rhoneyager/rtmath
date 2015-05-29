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

			namespace Rayleigh {
				void doCrossSection(
					const rtmath::phaseFuncs::pf_class_registry::setup &s,
					const rtmath::phaseFuncs::pf_class_registry::inputParamsPartial& i,
					rtmath::phaseFuncs::pf_class_registry::cross_sections& c)
				{
					using namespace ::rtmath::phaseFuncs;
					const double pi = boost::math::constants::pi<double>();

					if (abs(i.eps - 1) > 0.001)
					{
						c.valid = false;
						return;
					}

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
						} else {
							double scaledSA = pow(i.aeff, 2.0);
							scaledSA /= i.vFrac;
							scaledAeff = pow(scaledSA, 0.5);
						}
					}

					std::complex<double> mRes = i.m; 
					std::complex<double> mAir(1.0, 0);
					i.rmeth(i.m, mAir, i.vFrac, mRes);

					// Perform the calculation

					/// \todo Do testing to see if this is a valid method for the input parameters!
					//double rat = (i.aeff_version ==
					//	pf_class_registry::inputParamsPartial::aeff_version_type::EQUIV_V_SPHERE)
					//	? 1 : 0;
					//int np = (i.shape == pf_class_registry::inputParamsPartial::shape_type::SPHEROID)
					//	? -1 : -2;
					//double ar = i.eps;
					//if (abs(ar - 1.0) < 0.00001) ar = 1.0001;

					const double size_p = 2. * pi * scaledAeff / s.wavelength;
					if (size_p > 0.2)
					{
						c.valid = false;
						return;
					}
					float qext = -1, qsca = -1, qback = -1, gsca = -1;

					try {
						std::complex<double> d = ((mRes*mRes)-1.)/((mRes*mRes)+2.);
						qext = (float) (4. * size_p * (-1. * d).imag());
						qsca = (float) (8. * std::pow(size_p, 4.) / 3.
							* (d * conj(d)).real());
						qback = (float) (d * conj(d)).real() * 8. * std::pow(size_p, 4.) / 2.f;
						gsca = (float) -1;

						const double k = 2. * pi / s.wavelength;

						c.Qsca_iso = qsca * pow(scaledAeff / i.aeff, 2.);
						c.Qext_iso = qext * pow(scaledAeff / i.aeff, 2.);
						c.Qabs_iso = c.Qext_iso - c.Qsca_iso;
						c.g_iso = gsca;
						c.Qbk_iso = qback * pow(scaledAeff / i.aeff, 2.);

						double C_sphere = pi * pow(scaledAeff, 2.0);
						c.Qsca = c.Qsca_iso;
						c.Qbk = c.Qbk_iso;
						c.g = c.g_iso;
						c.Qext = c.Qext_iso;
						c.Qabs = c.Qabs_iso;

						/// iso values are validated with solid spheres and soft spheres using liu code
						/// \todo need to validate with ellipsoids

						//std::cerr << c.Qabs_iso << "\t" << c.Qsca_iso << "\t" << c.Qext_iso << "\t" << c.Qbk_iso << std::endl;
					} catch (...) {
						//std::cerr << "\t" << t.what() << std::endl;
						RDthrow(Ryan_Debug::error::xOtherError()) <<
							Ryan_Debug::error::otherErrorText("A bhmie error has occurred");
					}
				}
			}
			namespace RG {
				void doCrossSection(
					const rtmath::phaseFuncs::pf_class_registry::setup &s,
					const rtmath::phaseFuncs::pf_class_registry::inputParamsPartial& i,
					rtmath::phaseFuncs::pf_class_registry::cross_sections& c);
			}
		}
	}
}

D_Ryan_Debug_validator();
D_rtmath_validator();

D_Ryan_Debug_start()
{
	using namespace Ryan_Debug::registry;
	using namespace rtmath::plugins::nondda_misc;
	static const Ryan_Debug::registry::DLLpreamble id(
		"Plugin-nondda-misc",
		"Miscellaneous non-dda scattering plugins (Rayleigh, RG, SSRGA)",
		PLUGINID);
	dllInitResult res = Ryan_Debug_registry_register_dll(id, (void*)dllStart);
	if (res != SUCCESS) return res;

	rtmath::phaseFuncs::pf_class_registry pc;
	pc.name = "Rayleigh";
	pc.orientations = rtmath::phaseFuncs::pf_class_registry::orientation_type::ISOTROPIC;
	pc.fCrossSections = rtmath::plugins::nondda_misc::Rayleigh::doCrossSection;
	rtmath::phaseFuncs::pf_provider::registerHook(pc);

	//rtmath::phaseFuncs::pf_class_registry pcrg;
	//pcrg.name = "Rayleigh-Gans";
	//pcrg.orientations = rtmath::phaseFuncs::pf_class_registry::orientation_type::ISOTROPIC;
	//pcrg.fCrossSections = rtmath::plugins::nondda_misc::RG::doCrossSection;
	//rtmath::phaseFuncs::pf_provider::registerHook(pcrg);
	return SUCCESS;
}
