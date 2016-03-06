/// \brief Provides qhull
#define _SCL_SECURE_NO_WARNINGS

#include <string>

#include "../../rtmath/rtmath/defs.h"
#include "../../rtmath/rtmath/ddscat/shapefile.h"
#include "../../rtmath/rtmath/ddscat/shapestats.h"
#include "../../rtmath/rtmath/ddscat/hulls.h"
#include "../../rtmath/rtmath/Voronoi/Voronoi.h"
#include "../../rtmath/rtmath/plugin.h"

#include "plugin-qhull.h"

D_Ryan_Debug_validator();
D_rtmath_validator();


D_Ryan_Debug_start()
{
	using namespace Ryan_Debug::registry;
	using namespace rtmath::plugins::qhull;
	static const Ryan_Debug::registry::DLLpreamble id(
		"Plugin-qhull",
		"Provides hull generation",
		PLUGINID);
	dllInitResult res = Ryan_Debug_registry_register_dll(id, (void*)dllStart);
	if (res != SUCCESS) return res;

	rtmath::ddscat::hull_provider<rtmath::ddscat::convexHull> reg_convex_hull;
	reg_convex_hull.name = "qhull";
	reg_convex_hull.generator = rtmath::plugins::qhull::qhullConvexHull::generate;

	doRegisterHook<rtmath::ddscat::convexHull, ::rtmath::ddscat::hull_provider_registry, 
		::rtmath::ddscat::hull_provider<::rtmath::ddscat::convexHull> >(reg_convex_hull);

	return SUCCESS;
}
