#pragma once
#pragma warning( disable : 4251 ) // warning C4251: dll-interface

#include <memory>
#include <string>

#include "../../rtmath/rtmath/common_templates.h"
#include "../../rtmath/rtmath/plugin.h"
#include "../../rtmath/rtmath/error/debug.h"


#define PLUGINID "B30B35AF-8F61-4DB2-B42D-BADA08928717"


namespace rtmath {
	namespace ddscat {
		class ddOutput;
		class ddOutputSingle;
		class ddScattMatrix;
		namespace shapefile { class shapefile; }
		namespace stats { class shapeFileStats; }
	}
	namespace plugins {
		/** \brief Namespace for the netCDF file readers.
		*
		* This plugin will probably only be used to read netcdf files. I already have HDF5 
		* write support and thus have no reason to write netcdf files. The plugin will 
		* provide reading abilities for the rtmath_data library for ARM program data 
		* for various operational products.
		**/
		namespace netcdf {
			//class netcdfFile;


			struct netcdf_handle : public rtmath::registry::IOhandler
			{
				netcdf_handle(const char* filename, IOtype t);
				virtual ~netcdf_handle();
				void open(const char* filename, IOtype t);
				int file;
				bool headerOpen;
				bool readable;
				bool writeable;
				static void handle_error(int status);
				//std::shared_ptr<netcdfFile> file;
				void openHeader();
				void closeHeader();
			};


		}
	}
}

