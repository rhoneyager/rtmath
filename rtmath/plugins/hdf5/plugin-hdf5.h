#pragma once
#include <memory>
#include <string>

#include "../../rtmath/rtmath/common_templates.h"
#include "../../rtmath/rtmath/ddscat/shapefile.h"
#include "../../rtmath/rtmath/plugin.h"
#include "../../rtmath/rtmath/error/debug.h"


#define PLUGINID "3733FE13-F12A-4AEA-A377-F3FAAE28D5A8"

namespace H5
{
	class H5File;
}

namespace rtmath {
	namespace plugins {
		namespace hdf5 {

			bool match_hdf5_shapefile(const char*, const char*);
			bool match_hdf5_multi(const char*, const char*, 
				std::shared_ptr<rtmath::registry::IOhandler>);
			void write_hdf5_shapefile(const char*,
				const rtmath::ddscat::shapefile::shapefile *shp);
			std::shared_ptr<rtmath::registry::IOhandler> write_hdf5_multi_shapefile
				(std::shared_ptr<rtmath::registry::IOhandler> h, 
				const char* filename, 
				const rtmath::ddscat::shapefile::shapefile *shp, 
				const char* key, 
				rtmath::registry::IOhandler::IOtype iotype);

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

