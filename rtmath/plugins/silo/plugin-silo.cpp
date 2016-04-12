/// \brief Provides silo file IO
#define _SCL_SECURE_NO_WARNINGS

#include <string>

#include "../../rtmath/rtmath/defs.h"
#include "../../rtmath/rtmath/ddscat/shapefile.h"
#include "../../rtmath/rtmath/ddscat/shapestats.h"
#include "../../rtmath/rtmath/ddscat/ddOutput.h"
#include "../../rtmath/rtmath/ddscat/points.h"
#include "../../rtmath/rtmath/Voronoi/Voronoi.h"
#include "../../rtmath/rtmath/plugin.h"
#include <Ryan_Debug/debug.h>
#include <Ryan_Debug/error.h>

#include "plugin-silo.h"
#include "../../related/rtmath_silo_cpp/WritePoints.h"

D_Ryan_Debug_validator();
D_rtmath_validator();

namespace rtmath
{
	namespace plugins
	{
		namespace silo
		{
			silo_handle::silo_handle(const char* filename, IOtype t)
				: IOhandler(PLUGINID)
			{
				open(filename, t);
			}

			void silo_handle::open(const char* filename, IOtype t)
			{
				switch (t)
				{
				case IOtype::READWRITE:
					//file = std::shared_ptr<siloFile>(new siloFile(filename, H5F_ACC_RDWR ));
					//break;
				case IOtype::EXCLUSIVE:
					//file = std::shared_ptr<siloFile>(new siloFile(filename, H5F_ACC_EXCL ));
					//break;
				case IOtype::DEBUG:
					//file = std::shared_ptr<siloFile>(new siloFile(filename, H5F_ACC_DEBUG ));
					//break;
				case IOtype::CREATE:
					//file = std::shared_ptr<siloFile>(new siloFile(filename, H5F_ACC_CREAT ));
					//break;
				case IOtype::READONLY:
					//file = std::shared_ptr<siloFile>(new siloFile(filename, H5F_ACC_RDONLY ));
					RDthrow(Ryan_Debug::error::xUnimplementedFunction());
					break;
				case IOtype::TRUNCATE:
					file = std::shared_ptr<siloFile>(new siloFile(filename));
					break;
				}
			}

		}
	}
}


D_Ryan_Debug_start()
{
	using namespace Ryan_Debug::registry;
	using namespace rtmath::plugins::silo;
	static const Ryan_Debug::registry::DLLpreamble id(
		"Plugin-SILO",
		"Example plugin to provide shapefile class with the ability to "
		"read and write SILO files.",
		PLUGINID);
	dllInitResult res = Ryan_Debug_registry_register_dll(id, (void*)dllStart);
	if (res != SUCCESS) return res;

	genAndRegisterIOregistry_writer<::rtmath::ddscat::shapefile::shapefile, 
		rtmath::ddscat::shapefile::shapefile_IO_output_registry>("silo",PLUGINID);

	genAndRegisterIOregistry_writer<::rtmath::ddscat::points::sphereVol, 
		rtmath::ddscat::points::sphereVol_IO_output_registry>("silo",PLUGINID);

	//genAndRegisterIOregistry_writer<::rtmath::ddscat::ddOutput, 
	//	rtmath::ddscat::ddOutput_IO_output_registry>("silo",PLUGINID);

	//genAndRegisterIOregistry_writer<::rtmath::ddscat::stats::shapeFileStats, 
	//	rtmath::ddscat::stats::shapeFileStats_IO_output_registry>("silo",PLUGINID);

	genAndRegisterIOregistry_writer<::rtmath::Voronoi::VoronoiDiagram,
		rtmath::Voronoi::Voronoi_IO_output_registry>("silo", PLUGINID);
	return SUCCESS;
}
