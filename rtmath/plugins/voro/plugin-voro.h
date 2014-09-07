#pragma once
#pragma warning( disable : 4251 ) // warning C4251: dll-interface
#include <Eigen/Core>
#include <Eigen/Dense>
#include <memory>
#include <string>

#include "../../rtmath/rtmath/defs.h"
#include "../../rtmath/rtmath/common_templates.h"
#include "../../rtmath/rtmath/ddscat/hulls.h"
#include "../../rtmath/rtmath/plugin.h"
#include "../../rtmath/rtmath/error/debug.h"

#define PLUGINID "8673790D-278A-4523-91FB-18FE76192830"

namespace rtmath {
	namespace ddscat {
		class ddOutput;
		class ddOutputSingle;
		class ddScattMatrix;
		namespace shapefile { class shapefile; }
		namespace stats { class shapeFileStats; }
	}
	namespace plugins {
		namespace voro {
			

		}
	}
}

