/// \brief Provides HDF5 io routines
#pragma warning( disable : 4251 ) // warning C4251: dll-interface
#define _SCL_SECURE_NO_WARNINGS

//#include <cstdio>
//#include <cstring>
#include <iostream>
#include <fstream>
#include <string>
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>

#include "../../rtmath/rtmath/common_templates.h"
#include "../../rtmath/rtmath/ddscat/shapefile.h"
#include "../../rtmath/rtmath/ddscat/shapestats.h"
#include "../../rtmath/rtmath/Voronoi/Voronoi.h"
#include "../../rtmath/rtmath/phaseFunc.h"
#include "../../rtmath/rtmath/plugin.h"
#include "../../rtmath/rtmath/ddscat/ddOutput.h"
#include "../../rtmath/rtmath/error/debug.h"

#include "plugin-hdf5.h"
#include "../../related/rtmath_hdf5_cpp/export-hdf5.h"
#include "cmake-settings.h"

#include <hdf5.h>
#include <H5Cpp.h>


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
	using namespace rtmath::registry;
	using namespace rtmath::plugins::hdf5;
	static const DLLpreamble id(
		"Plugin-HDF5",
		"Plugin that provides HDF5 io routines for various classes.",
		PLUGINID);
	rtmath_registry_register_dll(id);

#if COMPRESS_ZLIB
	useZLIB(true);
#else
	useZLIB(false);
#endif

	genAndRegisterIOregistry_writer<::rtmath::ddscat::shapefile::shapefile,
		rtmath::ddscat::shapefile::shapefile_IO_output_registry>("hdf5", PLUGINID);
	genAndRegisterIOregistry_writer<::rtmath::ddscat::stats::shapeFileStats,
		rtmath::ddscat::stats::shapeFileStats_IO_output_registry>("hdf5", PLUGINID);
	genAndRegisterIOregistry_writer<::rtmath::ddscat::ddOutput,
		rtmath::ddscat::ddOutput_IO_output_registry>("hdf5", PLUGINID);
	genAndRegisterIOregistry_writer<::rtmath::Voronoi::VoronoiDiagram,
		::rtmath::Voronoi::Voronoi_IO_output_registry>("hdf5", PLUGINID);
	genAndRegisterIOregistry_writer<::rtmath::phaseFuncs::pfRunSetContainer,
		::rtmath::phaseFuncs::pfRunSetContainer_IO_input_registry>("hdf5", PLUGINID);


	genAndRegisterIOregistry_reader<::rtmath::ddscat::shapefile::shapefile,
		rtmath::ddscat::shapefile::shapefile_IO_input_registry>("hdf5", PLUGINID);
	genAndRegisterIOregistry_reader<::rtmath::ddscat::stats::shapeFileStats,
		rtmath::ddscat::stats::shapeFileStats_IO_input_registry>("hdf5", PLUGINID);
	genAndRegisterIOregistry_reader<::rtmath::ddscat::ddOutput,
		rtmath::ddscat::ddOutput_IO_input_registry>("hdf5", PLUGINID);
	genAndRegisterIOregistry_reader<::rtmath::Voronoi::VoronoiDiagram,
		::rtmath::Voronoi::Voronoi_IO_input_registry>("hdf5", PLUGINID);
	genAndRegisterIOregistry_reader<::rtmath::phaseFuncs::pfRunSetContainer,
		::rtmath::phaseFuncs::pfRunSetContainer_IO_input_registry>("hdf5", PLUGINID);
}
