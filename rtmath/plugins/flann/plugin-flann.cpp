/// \brief Provides flann
#define _SCL_SECURE_NO_WARNINGS

#include <string>

#include "../../rtmath/rtmath/defs.h"
#include "../../rtmath/rtmath/ddscat/shapefile.h"
#include "../../rtmath/rtmath/ddscat/shapestats.h"
#include "../../rtmath/rtmath/ddscat/hulls.h"
#include "../../rtmath/rtmath/plugin.h"
#include <Ryan_Debug/debug.h>
#include <Ryan_Debug/error.h>

#include "plugin-flann.h"

D_Ryan_Debug_validator();
D_rtmath_validator();

D_Ryan_Debug_start()
{
	using namespace Ryan_Debug::registry;
	using namespace rtmath::plugins::flann;
	static const Ryan_Debug::registry::DLLpreamble id(
		"Plugin-flann",
		"Provides K-d tree searches",
		PLUGINID);
	dllInitResult res = Ryan_Debug_registry_register_dll(id, (void*)dllStart);
	if (res != SUCCESS) return res;

	rtmath::Voronoi::Voronoi_provider reg_flann;
	reg_flann.name = PLUGINID;
	reg_flann.generator = ::rtmath::plugins::flann::VoroVoronoiDiagram::generateStandard;
	reg_flann.flannnoiBlankGenerator = ::rtmath::plugins::flann::VoroVoronoiDiagram::generateBlank;
	reg_flann.flannnoiUpcastGenerator = ::rtmath::plugins::flann::VoroVoronoiDiagram::generateUpcast;

	doRegisterHook<rtmath::Voronoi::VoronoiDiagram, ::rtmath::Voronoi::Voronoi_provider_registry,
		::rtmath::Voronoi::Voronoi_provider >(reg_flann);
	return SUCCESS;
}
