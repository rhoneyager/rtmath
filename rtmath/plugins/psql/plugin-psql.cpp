/// \brief Provides ImageMagick file IO
#define _SCL_SECURE_NO_WARNINGS

#include <Eigen/Core>
#include <Eigen/Dense>
#include <algorithm>
#include <string>
#include <boost/algorithm/string/trim.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include "../../rtmath/rtmath/defs.h"
#include "../../rtmath/rtmath/data/arm_info.h"
#include "../../rtmath/rtmath/data/arm_scanning_radar_sacr.h"
#include "../../rtmath/rtmath/plugin.h"

#include "plugin-psql.h"

void dllEntry();
rtmath_plugin_init(dllEntry);

namespace rtmath
{
	namespace plugins
	{
		namespace psql
		{
			psql_handle::psql_handle(const char* filename, IOtype t)
				: IOhandler(PLUGINID), file(-1), headerOpen(false), readable(false), writeable(false)
			{
				open(filename, t);
			}

			void psql_handle::open(const char* filename, IOtype t)
			{
				try {
					switch (t)
					{
					case IOtype::EXCLUSIVE:
					case IOtype::DEBUG:
						RTthrow rtmath::debug::xUnimplementedFunction();
						break;
					case IOtype::READONLY:
					{
						int status = nc_open(filename, 0, &file);
						if (status != NC_NOERR) handle_error(status);
						readable = true;
					}
						break;
					case IOtype::READWRITE:
					{
						int status = nc_open(filename, NC_WRITE, &file);
						if (status != NC_NOERR) handle_error(status);
						readable = true;
						writeable = true;
					}
						break;
					case IOtype::CREATE:
						if (boost::filesystem::exists(boost::filesystem::path(filename)))
							RTthrow debug::xFileExists(filename);
					case IOtype::TRUNCATE:
					{
						int status = nc_create(filename, 0, &file);
						if (status != NC_NOERR) handle_error(status);
						headerOpen = true;
						writeable = true;
					}
						break;
					}
				}
				catch (std::exception &e) {
					std::cerr << "Error caught in psql_handle::open!\n"
						<< "\tFilename: " << filename << "\n"
						<< "\tIOtype: " << t << std::endl;
					RTthrow e;
				}
			}

			void psql_handle::handle_error(int status)
			{
				std::cerr << "psql library error " << status << std::endl;
				RTthrow debug::xOtherError();
			}

			psql_handle::~psql_handle()
			{
				if (file >= 0)
				{
					nc_close(file);
				}
			}

			void psql_handle::openHeader()
			{
				// It's a bit useless for psql4, but occasionally a v3 file could be written...
				if (headerOpen) return;
				nc_redef(file);
			}

			void psql_handle::closeHeader()
			{
				// It's a bit useless for psql4, but occasionally a v3 file could be written...
				if (!headerOpen) return;
				nc_enddef(file);
			}


		}

	}
}


void dllEntry()
{
	using namespace rtmath::registry;
	static const rtmath::registry::DLLpreamble id(
		"Plugin-psql",
		"Provides postgresql database access (used for indexing stuff)",
		PLUGINID);
	rtmath_registry_register_dll(id);

	const size_t nExts = 2;
	const char* exts[nExts] = { "cdf", "nc" };

	genAndRegisterIOregistryPlural_reader
		<::rtmath::data::arm::arm_info,
		::rtmath::data::arm::arm_IO_input_registry>(
		nExts, exts, PLUGINID);

	/*
	genAndRegisterIOregistryPlural_writer
	<::rtmath::images::image,
	rtmath::images::image_IO_output_registry>(
	nExts, exts, PLUGINID, "");
	*/

	genAndRegisterIOregistryPlural_reader
		<::rtmath::data::arm::arm_scanning_radar_sacr,
		::rtmath::data::arm::arm_IO_sacr_input_registry>(
		nExts, exts, PLUGINID);
}
