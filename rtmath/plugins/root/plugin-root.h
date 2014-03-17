#pragma once
#pragma warning( disable : 4251 ) // warning C4251: dll-interface

#include <memory>
#include <string>

#include "../../rtmath/rtmath/common_templates.h"
#include "../../rtmath/rtmath/plugin.h"
#include "../../rtmath/rtmath/error/debug.h"


#define PLUGINID "94DB89ED-B1B4-412C-8438-F5F54FFD36BF"


namespace rtmath {
	namespace ddscat {
		class ddOutput;
		class ddOutputSingle;
		class ddScattMatrix;
		namespace shapefile { class shapefile; }
		namespace stats { class shapeFileStats; }
	}
	namespace plugins {
		namespace root {
			//class rootFile;

			/// \note Covers .root files, not image export
			struct root_handle : public rtmath::registry::IOhandler
			{
				root_handle(const char* filename, IOtype t);
				virtual ~root_handle() {}
				void open(const char* filename, IOtype t);
				//std::shared_ptr<siloFile> file;
			};


		}
	}
}

