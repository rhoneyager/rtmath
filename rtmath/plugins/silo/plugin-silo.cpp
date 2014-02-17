/// \brief Provides silo file IO
#define _SCL_SECURE_NO_WARNINGS

#include <string>

#include "../../rtmath/rtmath/defs.h"
#include "../../rtmath/rtmath/ddscat/shapefile.h"
#include "../../rtmath/rtmath/ddscat/shapestats.h"
#include "../../rtmath/rtmath/ddscat/ddOutput.h"
#include "../../rtmath/rtmath/plugin.h"

#include "plugin-silo.h"
#include "WritePoints.h"

void dllEntry();
rtmath_plugin_init(dllEntry);

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
					RTthrow rtmath::debug::xUnimplementedFunction();
					break;
				case IOtype::TRUNCATE:
					file = std::shared_ptr<siloFile>(new siloFile(filename));
					break;
				}
			}

		}
	}
}


void dllEntry()
{
	using namespace rtmath::registry;
	using namespace rtmath::plugins::silo;
	static const rtmath::registry::DLLpreamble id(
		"Plugin-SILO",
		"Example plugin to provide shapefile class with the ability to "
		"read and write SILO files.",
		PLUGINID);
	rtmath_registry_register_dll(id);

	genAndRegisterIOregistry<::rtmath::ddscat::shapefile::shapefile, 
		rtmath::ddscat::shapefile::shapefile_IO_output_registry>("silo",PLUGINID);

	genAndRegisterIOregistry<::rtmath::ddscat::ddOutput, 
		rtmath::ddscat::ddOutput_IO_output_registry>("silo",PLUGINID);

	genAndRegisterIOregistry<::rtmath::ddscat::stats::shapeFileStats, 
		rtmath::ddscat::stats::shapeFileStats_IO_output_registry>("silo",PLUGINID);
}
