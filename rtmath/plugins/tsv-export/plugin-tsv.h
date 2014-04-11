#pragma once
#pragma warning( disable : 4251 ) // warning C4251: dll-interface

#include <memory>
#include <string>

#include "../../rtmath/rtmath/common_templates.h"
#include "../../rtmath/rtmath/plugin.h"
#include "../../rtmath/rtmath/error/debug.h"


#define PLUGINID "B77570C4-C2D2-471C-B24A-A279061E363B"
#define PLUGINID_ARS "B77570C4-C2D2-471C-B24A-A279061E363B-ars"
#define PLUGINID_VORO "B77570C4-C2D2-471C-B24A-A279061E363B-voro"
#define PLUGINID_DDORI "B77570C4-C2D2-471C-B24A-A279061E363B-ddori"
#define PLUGINID_DDISO "B77570C4-C2D2-471C-B24A-A279061E363B-ddiso"
#define PLUGINID_IMAGE "B77570C4-C2D2-471C-B24A-A279061E363B-image"
#define PLUGINID_SACR_REFL "B77570C4-C2D2-471C-B24A-A279061E363B-sacr-refl"

namespace rtmath {
	namespace ddscat {
		class ddOutput;
		class ddOutputSingle;
		class ddScattMatrix;
		namespace shapefile { class shapefile; }
		namespace stats { class shapeFileStats; }
	}
	namespace images {
		class image;
	}
}

