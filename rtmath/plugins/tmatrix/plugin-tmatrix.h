#pragma once
#pragma warning( disable : 4251 ) // warning C4251: dll-interface

#include <memory>
#include <string>

#include "../../rtmath/rtmath/common_templates.h"
#include "../../rtmath/rtmath/plugin.h"
#include "../../rtmath/rtmath/error/debug.h"


#define PLUGINID "390CDC28-62BF-43C9-AB07-BC19899C7891"


namespace rtmath {
	namespace ddscat {
		class ddOutput;
		namespace shapefile { class shapefile; }
		namespace stats { class shapeFileStats; }
	}
	namespace plugins {
		namespace tmatrix {

		}
	}
}

