/// \brief Provides ImageMagick file IO
#define _SCL_SECURE_NO_WARNINGS

#include <string>

#include "../../rtmath/rtmath/defs.h"
#include "../../rtmath/rtmath/images/image.h"
#include "../../rtmath/rtmath/plugin.h"

#include "plugin-netcdf.h"

void dllEntry();
rtmath_plugin_init(dllEntry);

namespace rtmath
{
	namespace plugins
	{
		namespace netcdf
		{
			netcdf_handle::netcdf_handle(const char* filename, IOtype t)
				: IOhandler(PLUGINID)
			{
				open(filename, t);
			}

			void netcdf_handle::open(const char* filename, IOtype t)
			{
				switch (t)
				{
					/*
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
					*/
				case IOtype::READWRITE:
				case IOtype::EXCLUSIVE:
				case IOtype::DEBUG:
					RTthrow rtmath::debug::xUnimplementedFunction();
					break;
				case IOtype::READONLY:
				case IOtype::CREATE:
				case IOtype::TRUNCATE:
					break;
				}
			}

		}
	}
}


void dllEntry()
{
	using namespace rtmath::registry;
	static const rtmath::registry::DLLpreamble id(
		"Plugin-netcdf",
		"Provides netcdf IO for reading and writing ARM data",
		PLUGINID);
	rtmath_registry_register_dll(id);

	const size_t nExts = 2;
	const char* exts[nExts] = { "cdf", "nc" };
	/*
	genAndRegisterIOregistryPlural_writer
		<::rtmath::images::image,
		rtmath::images::image_IO_output_registry>(
		nExts, exts, PLUGINID, "");

	genAndRegisterIOregistryPlural_reader
		<::rtmath::images::image,
		rtmath::images::image_IO_input_registry>(
		nExts, exts, PLUGINID);
	*/
}
