/// \brief Provides Rayleigh and Rayleigh-Gans
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
					rtmath::phaseFuncs::pf_class_registry::cross_sections& c);
			}
			namespace HRG {
				void doCrossSection(
					const rtmath::phaseFuncs::pf_class_registry::setup &s,
					const rtmath::phaseFuncs::pf_class_registry::inputParamsPartial& i,
					rtmath::phaseFuncs::pf_class_registry::cross_sections& c);
			}
			namespace RGi {
				void doCrossSection(
					const rtmath::phaseFuncs::pf_class_registry::setup &s,
					const rtmath::phaseFuncs::pf_class_registry::inputParamsPartial& i,
					rtmath::phaseFuncs::pf_class_registry::cross_sections& c);
			}
			namespace HRGi {
				void doCrossSection(
					const rtmath::phaseFuncs::pf_class_registry::setup &s,
					const rtmath::phaseFuncs::pf_class_registry::inputParamsPartial& i,
					rtmath::phaseFuncs::pf_class_registry::cross_sections& c);
			}
			namespace Rayleigh_2 {
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

	rtmath::phaseFuncs::pf_class_registry pc2;
	pc2.name = "Rayleigh";
	pc2.fCrossSections = rtmath::plugins::nondda_misc::Rayleigh_2::doCrossSection;
	rtmath::phaseFuncs::pf_provider::registerHook(pc2);

	rtmath::phaseFuncs::pf_class_registry pcrg;
	pcrg.name = "Rayleigh-Gans";
	pcrg.fCrossSections = rtmath::plugins::nondda_misc::RG::doCrossSection;
	rtmath::phaseFuncs::pf_provider::registerHook(pcrg);

	rtmath::phaseFuncs::pf_class_registry hpcrg;
	hpcrg.name = "SSRG";
	hpcrg.fCrossSections = rtmath::plugins::nondda_misc::HRG::doCrossSection;
	rtmath::phaseFuncs::pf_provider::registerHook(hpcrg);

	rtmath::phaseFuncs::pf_class_registry pcrgi;
	pcrgi.name = "Rayleigh-Gans-iso";
	pcrgi.fCrossSections = rtmath::plugins::nondda_misc::RGi::doCrossSection;
	rtmath::phaseFuncs::pf_provider::registerHook(pcrgi);

	return SUCCESS;
}
