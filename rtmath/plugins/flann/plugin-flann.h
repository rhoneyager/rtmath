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

#define PLUGINID "68220688-e6cd-4563-8cdb-992dda35df7b"

namespace rtmath {
	namespace ddscat {
		namespace shapefile { class shapefile; }
		namespace stats { class shapeFileStats; }
	}
	namespace plugins {
		namespace flann {

		}
	}
}

