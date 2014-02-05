/// \brief Provides HDF5 io routines
#define _SCL_SECURE_NO_WARNINGS

#include <cstdio>
#include <cstring>
#include <iostream>
#include <fstream>
#include <string>
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>

#include <hdf5.h>
#include <H5Cpp.h>


#include "../../rtmath/rtmath/common_templates.h"
#include "../../rtmath/rtmath/ddscat/shapefile.h"
#include "../../rtmath/rtmath/plugin.h"
#include "../../rtmath/rtmath/error/debug.h"

#include "plugin-hdf5.h"

void dllEntry();
rtmath_plugin_init(dllEntry);

namespace rtmath {
	namespace plugins {
		namespace hdf5 {
			hdf5_handle::hdf5_handle(const char* filename, IOtype t)
				: IOhandler(PLUGINID)
			{
				open(filename, t);
			}

			void hdf5_handle::open(const char* filename, IOtype t)
				{
					using namespace H5;
					switch (t)
					{
					case IOtype::READWRITE:
						file = std::shared_ptr<H5File>(new H5File(filename, H5F_ACC_RDWR ));
						break;
					case IOtype::EXCLUSIVE:
						file = std::shared_ptr<H5File>(new H5File(filename, H5F_ACC_EXCL ));
						break;
					case IOtype::DEBUG:
						file = std::shared_ptr<H5File>(new H5File(filename, H5F_ACC_DEBUG ));
						break;
					case IOtype::CREATE:
						file = std::shared_ptr<H5File>(new H5File(filename, H5F_ACC_CREAT ));
						break;
					case IOtype::READONLY:
						file = std::shared_ptr<H5File>(new H5File(filename, H5F_ACC_RDONLY ));
						break;
					case IOtype::TRUNCATE:
						file = std::shared_ptr<H5File>(new H5File(filename, H5F_ACC_TRUNC ));
						break;
					}
				}
		}
	}
}


void dllEntry()
{
	static const rtmath::registry::DLLpreamble id(
		"Plugin-HDF5",
		"Plugin that provides HDF5 io routines for various classes.",
		PLUGINID);
	rtmath_registry_register_dll(id);

	static rtmath::registry::IO_class_registry<
		::rtmath::ddscat::shapefile::shapefile> s;
	s.io_matches = rtmath::plugins::hdf5::match_hdf5_shapefile;
	s.io_processor = rtmath::plugins::hdf5::write_hdf5_shapefile;
	s.io_multi_matches = rtmath::plugins::hdf5::match_hdf5_multi;
	s.io_multi_processor = rtmath::plugins::hdf5::write_hdf5_multi_shapefile;
	rtmath::ddscat::shapefile::shapefile::usesDLLregistry<
		rtmath::ddscat::shapefile::shapefile_IO_output_registry,
		rtmath::registry::IO_class_registry<::rtmath::ddscat::shapefile::shapefile> >
		::registerHook(s);

}
