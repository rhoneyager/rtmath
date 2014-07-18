/// \brief Provides silo file IO
#define _SCL_SECURE_NO_WARNINGS

#include <string>

#include "../../rtmath/rtmath/defs.h"
#include "../../rtmath/rtmath/phaseFunc.h"
#include "../../rtmath/rtmath/ddscat/shapefile.h"
#include "../../rtmath/rtmath/ddscat/shapestats.h"
#include "../../rtmath/rtmath/ddscat/ddOutput.h"
#include "../../rtmath/rtmath/plugin.h"

#include "plugin-tmatrix.h"

void dllEntry();
rtmath_plugin_init(dllEntry);

namespace rtmath
{
	namespace plugins
	{
		namespace tmatrix
		{
		}
	}
}



void dllEntry()
{
	using namespace rtmath::registry;
	using namespace rtmath::plugins::tmatrix;
	static const rtmath::registry::DLLpreamble id(
		"Plugin-Tmatrix-ori",
		"Links to Mishchenko T-matrix code (oriented version)",
		PLUGINID);
	rtmath_registry_register_dll(id);

	//genAndRegisterIOregistry<::rtmath::ddscat::shapefile::shapefile, 
	//	rtmath::ddscat::shapefile::shapefile_IO_output_registry>("silo",PLUGINID);
	rtmath::phaseFuncs::pf_class_registry pc;
	pc.name = "tmatrix-ori";
	pc.orientations = rtmath::phaseFuncs::pf_class_registry::orientation_type::ORIENTED;
	pc.fCrossSections;
	pc.fPfs;
	rtmath::phaseFuncs::pf_provider::registerHook(pc);
}
