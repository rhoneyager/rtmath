#pragma once
#pragma warning( disable : 4251 ) // warning C4251: dll-interface

#include <memory>
#include <string>

#include <hdf5.h>
#include <H5Cpp.h>

#include "../../rtmath/rtmath/common_templates.h"
#include "../../rtmath/rtmath/plugin.h"
#include "../../rtmath/rtmath/error/debug.h"


#define PLUGINID "3733FE13-F12A-4AEA-A377-F3FAAE28D5A8"


namespace rtmath {
	namespace ddscat {
		class ddOutput;
		class ddOutputSingle;
		class ddScattMatrix;
		namespace shapefile { class shapefile; }
		namespace stats { class shapeFileStats; }
	}
	namespace plugins {
		namespace hdf5 {

			//void write_hdf5_shapefile(const char*,
			//	const rtmath::ddscat::shapefile::shapefile *shp);
			
			std::shared_ptr<H5::Group> write_hdf5_shaperawdata(std::shared_ptr<H5::Group> base, 
				const rtmath::ddscat::shapefile::shapefile *shp);

			//void write_hdf5_shapestats(const char*,
			//	const rtmath::ddscat::stats::shapeFileStats *s);
			std::shared_ptr<H5::Group> write_hdf5_statsrawdata(std::shared_ptr<H5::Group> base, 
				const rtmath::ddscat::stats::shapeFileStats *s);

			//void write_hdf5_ddOutput(const char* filename,
			//	const rtmath::ddscat::ddOutput*);

			struct hdf5_handle : public rtmath::registry::IOhandler
			{
				hdf5_handle(const char* filename, IOtype t);
				virtual ~hdf5_handle() {}
				void open(const char* filename, IOtype t);
				std::shared_ptr<H5::H5File> file;
			};


		}
	}
}

