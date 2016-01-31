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

#include "plugin-scatdb-ryan.h"

namespace rtmath
{
	namespace plugins
	{
		namespace scatdb_ryan
		{
			namespace main {
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
	using namespace rtmath::plugins::scatdb_ryan;
	static const Ryan_Debug::registry::DLLpreamble id(
		"Plugin-scatdb-ryan",
		"Queries Ryan's particle scattering database",
		PLUGINID);
	dllInitResult res = Ryan_Debug_registry_register_dll(id, (void*)dllStart);
	if (res != SUCCESS) return res;

	rtmath::phaseFuncs::pf_class_registry pc;
	pc.name = "scatdb_ryan";
	pc.fCrossSections = rtmath::plugins::scatdb_ryan::main::doCrossSection;
	rtmath::phaseFuncs::pf_provider::registerHook(pc);

	return SUCCESS;
}
