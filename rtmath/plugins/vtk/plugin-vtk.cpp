/// \brief Provides vtk
#define _SCL_SECURE_NO_WARNINGS

#include <string>

#include "../../rtmath/rtmath/defs.h"
#include "../../rtmath/rtmath/ddscat/shapefile.h"
#include "../../rtmath/rtmath/ddscat/shapestats.h"
#include "../../rtmath/rtmath/ddscat/hulls.h"
#include "../../rtmath/rtmath/Voronoi/Voronoi.h"
#include "../../rtmath/rtmath/plugin.h"

#include "plugin-vtk.h"

void dllEntry();
rtmath_plugin_init(dllEntry);

namespace rtmath
{
	namespace plugins
	{
		namespace vtk
		{

		}
	}
}


void dllEntry()
{
	using namespace rtmath::registry;
	using namespace rtmath::plugins::vtk;
	static const rtmath::registry::DLLpreamble id(
		"Plugin-vtk",
		"Provides hull generation, image processing, and file i/o.",
		PLUGINID);
	rtmath_registry_register_dll(id);

	//genAndRegisterIOregistry<::rtmath::ddscat::shapefile::shapefile, 
	//	rtmath::ddscat::shapefile::shapefile_IO_output_registry>("silo",PLUGINID);

	//genAndRegisterIOregistry<::rtmath::Voronoi::VoronoiDiagram,
	//	rtmath::Voronoi::Voronoi_IO_output_registry>("silo", PLUGINID);

	rtmath::ddscat::hull_provider<rtmath::ddscat::convexHull> reg_convex_hull;
	reg_convex_hull.name = "vtk";
	reg_convex_hull.generator = rtmath::plugins::vtk::vtkConvexHull::generate;

	doRegisterHook<rtmath::ddscat::convexHull, ::rtmath::ddscat::hull_provider_registry, 
		::rtmath::ddscat::hull_provider<::rtmath::ddscat::convexHull> >(reg_convex_hull);
}
