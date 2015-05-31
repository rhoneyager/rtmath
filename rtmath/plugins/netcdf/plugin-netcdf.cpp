/// \brief Provides ImageMagick file IO
#define _SCL_SECURE_NO_WARNINGS

#include <Eigen/Core>
#include <Eigen/Dense>
#include <algorithm>
#include <string>
#include <boost/algorithm/string/trim.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/throw_exception.hpp>

#include <Ryan_Debug/error.h>
#include "../../rtmath/rtmath/defs.h"
#include <Ryan_Debug/debug.h>
#include <Ryan_Debug/error.h>
#include "../../rtmath/rtmath/data/arm_info.h"
#include "../../rtmath/rtmath/data/arm_scanning_radar_sacr.h"
#include "../../rtmath/rtmath/plugin.h"

#include "plugin-netcdf.h"

D_Ryan_Debug_validator();
D_rtmath_validator();

namespace rtmath
{
	namespace plugins
	{
		namespace netcdf
		{
			netcdf_handle::netcdf_handle(const char* filename, IOtype t)
				: IOhandler(PLUGINID), file(-1), headerOpen(false), readable(false), writeable(false)
			{
				open(filename, t);
			}

			void netcdf_handle::open(const char* filename, IOtype t)
			{
				try {
					switch (t)
					{
					case IOtype::EXCLUSIVE:
					case IOtype::DEBUG:
						RDthrow(Ryan_Debug::error::xUnimplementedFunction());
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
							RDthrow(Ryan_Debug::error::xFileExists())
							<< Ryan_Debug::error::file_name(filename);
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
					std::cerr << "Error caught in netcdf_handle::open!\n"
						<< "\tFilename: " << filename << "\n"
						<< "\tIOtype: " << t << std::endl;
					boost::throw_exception(e);
					//RDthrow e;
				}
			}

			void netcdf_handle::handle_error(int status)
			{
				RDthrow(Ryan_Debug::error::xOtherError())
					<< Ryan_Debug::error::otherErrorCode(status)
					<< Ryan_Debug::error::otherErrorText("netcdf library error");
			}

			netcdf_handle::~netcdf_handle()
			{
				if (file >= 0)
				{
					nc_close(file);
				}
			}

			void netcdf_handle::openHeader()
			{
				// It's a bit useless for netcdf4, but occasionally a v3 file could be written...
				if (headerOpen) return;
				nc_redef(file);
			}

			void netcdf_handle::closeHeader()
			{
				// It's a bit useless for netcdf4, but occasionally a v3 file could be written...
				if (!headerOpen) return;
				nc_enddef(file);
			}

			template<> bool AttrMatches<double>(nc_type t) { if (t == NC_DOUBLE) return true; return false; }
			template<> bool AttrMatches<float>(nc_type t) { if (t == NC_FLOAT) return true; return false; }
			template<> bool AttrMatches<int>(nc_type t) { if (t == NC_INT) return true; return false; }
			template<> bool AttrMatches<short>(nc_type t) { if (t == NC_SHORT) return true; return false; }
			// char needs special handling
			//template<> bool AttrMatches<char>(nc_type t) { if (t == NC_CHAR) return true; return false; }
			//template<> bool AttrMatches<double>(nc_type t) { if (t == NC_BYTE) return true; return false; }

			std::pair<bool, int> AttrExists(int ncid, int varid, const char* varname)
			{
				int status = 0;
				int attrid = 0;
				status = nc_inq_attid(ncid, varid, varname, &attrid);
				if (status) return std::pair<bool, int>(false, -1);
				else return std::pair<bool, int>(true, attrid);
			}

			std::pair<bool, int> VarExists(int ncid, const char* varname)
			{
				int status = 0;
				int varid = 0;
				status = nc_inq_varid(ncid, varname, &varid);
				if (status) return std::pair<bool, int>(false, -1);
				else return std::pair<bool, int>(true, varid);
			}

			
			template<> int getNCvar<double>(int ncid, int varid, double* res) { return nc_get_var_double(ncid, varid, res); }
			template<> int getNCvar<float>(int ncid, int varid, float* res) { return nc_get_var_float(ncid, varid, res); }
			template<> int getNCvar<int>(int ncid, int varid, int* res) { return nc_get_var_int(ncid, varid, res); }
			template<> int getNCvar<short>(int ncid, int varid, short* res) { return nc_get_var_short(ncid, varid, res); }

			/*
			template<> int getNCvar<std::string>(int ncid, int varid, std::string *res)
			{
			//auto getAttrString = [&](const char* name, int varid) -> std::string
				int status = 0;
				size_t len = 0;
				status = nc_inq_attlen(ncid, varid, name, &len);
				if (status) return status;
				std::unique_ptr<char[]> txtbuf(new char[len + 1]);
				status = nc_get_att_text(h->file, varid, name, txtbuf.get());
				if (status) h->handle_error(status);
				std::string r(txtbuf.get(), len+1); // The netcdf storage of some of the arm attributes was flawed
				*res = r;
			}
			*/

		}

	}
}


D_Ryan_Debug_start()
{
	using namespace Ryan_Debug::registry;
	static const Ryan_Debug::registry::DLLpreamble id(
		"Plugin-netcdf",
		"Provides netcdf IO for reading and writing ARM data",
		PLUGINID);
	dllInitResult res = Ryan_Debug_registry_register_dll(id, (void*)dllStart);
	if (res != SUCCESS) return res;

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
	return SUCCESS;
}
