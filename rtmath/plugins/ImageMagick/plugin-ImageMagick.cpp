/// \brief Provides ImageMagick file IO
#define _SCL_SECURE_NO_WARNINGS

#include <string>

#include "../../rtmath/rtmath/defs.h"
#include "../../rtmath/rtmath/images/image.h"
#include "../../rtmath/rtmath/plugin.h"

#include "plugin-ImageMagick.h"

void dllEntry();
rtmath_plugin_init(dllEntry);

namespace rtmath
{
	namespace plugins
	{
		namespace ImageMagick
		{
			ImageMagick_handle::ImageMagick_handle(const char* filename, IOtype t)
				: IOhandler(PLUGINID)
			{
				open(filename, t);
			}

			void ImageMagick_handle::open(const char* filename, IOtype t)
			{
				switch (t)
				{
				case IOtype::READWRITE:
				case IOtype::EXCLUSIVE:
				case IOtype::DEBUG:
					RTthrow(debug::xUnimplementedFunction());
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
		"Plugin-ImageMagick",
		"Provides ImageMagick IO for reading and writing images",
		PLUGINID);
	rtmath_registry_register_dll(id);

	const size_t nExts = 5;
	const char* exts[nExts] = { "png", "bmp", "jpg", "jpeg", "gif" };
	genAndRegisterIOregistryPlural_writer
		<::rtmath::images::image,
		rtmath::images::image_IO_output_registry>(
		nExts, exts, PLUGINID, "");

	genAndRegisterIOregistryPlural_reader
		<::rtmath::images::image,
		rtmath::images::image_IO_input_registry>(
		nExts, exts, PLUGINID);
}
