/// \brief Provides Cern ROOT file IO
#define _SCL_SECURE_NO_WARNINGS

#include <string>

#include "../../rtmath/rtmath/defs.h"
#include "../../rtmath/rtmath/ddscat/shapefile.h"
#include "../../rtmath/rtmath/ddscat/shapestats.h"
#include "../../rtmath/rtmath/ddscat/ddOutput.h"
#include "../../rtmath/rtmath/Voronoi/Voronoi.h"
#include "../../rtmath/rtmath/plugin.h"

#include "plugin-root.h"

void dllEntry();
rtmath_plugin_init(dllEntry);

namespace rtmath
{
	namespace plugins
	{
		namespace root
		{
			root_handle::root_handle(const char* filename, IOtype t)
				: IOhandler(PLUGINID)
			{
				open(filename, t);
			}

			void root_handle::open(const char* filename, IOtype t)
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
					RTthrow rtmath::debug::xUnimplementedFunction();
					//file = std::shared_ptr<rootFile>(new rootFile(filename));
					break;
				}
			}

		}
	}
}


void dllEntry()
{
	using namespace rtmath::registry;
	using namespace rtmath::plugins::root;
	static const rtmath::registry::DLLpreamble id(
		"Plugin-ROOT",
		"Provides ROOT export of some data, as well as visualizations for "
		"a few data objects using Cern ROOT.",
		PLUGINID);
	rtmath_registry_register_dll(id);

	const size_t nExts = 14;
	const char* exportExts[nExts] = { "png", "jpg", "jpeg", "bmp", "eps", 
		"ps", "gif", "pdf", "xml", "xpm", "svg", "tiff", "root", "cxx" };
	genAndRegisterIOregistryPlural<::rtmath::ddscat::stats::shapeFileStats,
		rtmath::ddscat::stats::shapeFileStats_IO_output_registry>(
		nExts, exportExts, PLUGINID, "ar_rot_data");

}
