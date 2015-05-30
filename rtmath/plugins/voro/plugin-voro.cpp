/// \brief Provides vtk
#define _SCL_SECURE_NO_WARNINGS

#include <string>

#include "../../rtmath/rtmath/defs.h"
#include "../../rtmath/rtmath/ddscat/shapefile.h"
#include "../../rtmath/rtmath/ddscat/shapestats.h"
#include "../../rtmath/rtmath/ddscat/hulls.h"
#include "../../rtmath/rtmath/Voronoi/Voronoi.h"
#include "../../rtmath/rtmath/plugin.h"
#include <Ryan_Debug/debug.h>
#include <Ryan_Debug/error.h>
#include "voro-Voronoi.h"

#include "plugin-voro.h"

D_Ryan_Debug_validator();
D_rtmath_validator();

D_Ryan_Debug_start()
{
	using namespace Ryan_Debug::registry;
	using namespace rtmath::plugins::voro;
	static const Ryan_Debug::registry::DLLpreamble id(
		"Plugin-voro",
		"Provides voronoi diagram generation",
		PLUGINID);
	dllInitResult res = Ryan_Debug_registry_register_dll(id, (void*)dllStart);
	if (res != SUCCESS) return res;

	//genAndRegisterIOregistry<::rtmath::ddscat::shapefile::shapefile, 
	//	rtmath::ddscat::shapefile::shapefile_IO_output_registry>("silo",PLUGINID);

	//genAndRegisterIOregistry<::rtmath::Voronoi::VoronoiDiagram,
	//	rtmath::Voronoi::Voronoi_IO_output_registry>("silo", PLUGINID);

	rtmath::Voronoi::Voronoi_provider reg_Voronoi;
	reg_Voronoi.name = PLUGINID;
	reg_Voronoi.generator = ::rtmath::plugins::voro::VoroVoronoiDiagram::generateStandard;
	reg_Voronoi.voronoiBlankGenerator = ::rtmath::plugins::voro::VoroVoronoiDiagram::generateBlank;
	reg_Voronoi.voronoiUpcastGenerator = ::rtmath::plugins::voro::VoroVoronoiDiagram::generateUpcast;

	doRegisterHook<rtmath::Voronoi::VoronoiDiagram, ::rtmath::Voronoi::Voronoi_provider_registry,
		::rtmath::Voronoi::Voronoi_provider >(reg_Voronoi);
	return SUCCESS;
}
