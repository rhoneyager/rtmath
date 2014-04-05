#pragma once
#pragma warning( disable : 4251 ) // warning C4251: dll-interface

#include <memory>
#include <string>

#include "../../rtmath/rtmath/common_templates.h"
#include "../../rtmath/rtmath/plugin.h"
#include "../../rtmath/rtmath/error/debug.h"


#define PLUGINID "6341FE06-83C0-4EA6-87E0-8CFABD170C32"


namespace rtmath {
	namespace ddscat {
		class ddOutput;
		class ddOutputSingle;
		class ddScattMatrix;
		namespace shapefile { class shapefile; }
		namespace stats { class shapeFileStats; }
	}
	namespace plugins {
		namespace netcdf {
			class netcdfFile;


			struct netcdf_handle : public rtmath::registry::IOhandler
			{
				netcdf_handle(const char* filename, IOtype t);
				virtual ~netcdf_handle() {}
				void open(const char* filename, IOtype t);
				std::shared_ptr<netcdfFile> file;
			};


		}
	}
}

