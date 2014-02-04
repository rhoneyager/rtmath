/// \brief Provides HDF5 io routines
#define _SCL_SECURE_NO_WARNINGS

#include <cstdio>
#include <cstring>
#include <iostream>
#include <fstream>
#include <string>
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>

//#include <hdf5.h>

#include "../../rtmath/rtmath/common_templates.h"
#include "../../rtmath/rtmath/ddscat/shapefile.h"
#include "../../rtmath/rtmath/plugin.h"
#include "../../rtmath/rtmath/error/debug.h"

void dllEntry();
rtmath_plugin_init(dllEntry);

namespace rtmath {
	namespace plugins {
		namespace hdf5 {

			bool match_hdf5_shapefile(const char*, const char*);
			void write_hdf5_shapefile(const char*,
				const rtmath::ddscat::shapefile::shapefile *shp);

		}
	}
}


void dllEntry()
{
	static const rtmath::registry::DLLpreamble id(
		"Plugin-HDF5",
		"Plugin that provides HDF5 io routines for various classes.",
		"3733FE13-F12A-4AEA-A377-F3FAAE28D5A8");
	rtmath_registry_register_dll(id);

	static rtmath::registry::IO_class_registry<
		::rtmath::ddscat::shapefile::shapefile> s;
	s.io_matches = rtmath::plugins::hdf5::match_hdf5_shapefile;
	s.io_processor = rtmath::plugins::hdf5::write_hdf5_shapefile;
	rtmath::ddscat::shapefile::shapefile::usesDLLregistry<
		rtmath::ddscat::shapefile::shapefile_IO_output_registry,
		rtmath::registry::IO_class_registry<::rtmath::ddscat::shapefile::shapefile> >
		::registerHook(s);

}
