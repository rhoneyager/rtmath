/// \brief Provides ImageMagick file IO
#define _SCL_SECURE_NO_WARNINGS

#include <Eigen/Core>
#include <Eigen/Dense>
#include <string>
#include <boost/date_time/posix_time/posix_time.hpp>

#include "../../rtmath/rtmath/defs.h"
#include "../../rtmath/rtmath/data/arm_info.h"
#include "../../rtmath/rtmath/plugin.h"

#include "plugin-netcdf.h"
#include <netcdf.h>

void dllEntry();
rtmath_plugin_init(dllEntry);

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
				} catch (std::exception &e) {
					std::cerr << "Error caught in netcdf_handle::open!\n"
						<< "\tFilename: " << filename << "\n"
						<< "\tIOtype: " << t << std::endl;
					RTthrow e;
				}
			}

			void netcdf_handle::handle_error(int status)
			{
				std::cerr << "netcdf library error " << status << std::endl;
				RTthrow debug::xOtherError();
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


			template <class DataType>
			bool AttrMatches(nc_type) { return false; }
			template<> bool AttrMatches<double>(nc_type t) { if (t == NC_DOUBLE) return true; return false; }
			template<> bool AttrMatches<float>(nc_type t) { if (t == NC_FLOAT) return true; return false; }
			template<> bool AttrMatches<int>(nc_type t) { if (t == NC_INT) return true; return false; }
			template<> bool AttrMatches<short>(nc_type t) { if (t == NC_SHORT) return true; return false; }
			// char needs special handling
			//template<> bool AttrMatches<char>(nc_type t) { if (t == NC_CHAR) return true; return false; }
			//template<> bool AttrMatches<double>(nc_type t) { if (t == NC_BYTE) return true; return false; }

			template <class DataType>
			int getNCvar(int ncid, int varid, DataType *) { RTthrow debug::xUnimplementedFunction(); }
			template<> int getNCvar<double>(int ncid, int varid, double* res) { return nc_get_var_double(ncid, varid, res); }
			template<> int getNCvar<float>(int ncid, int varid, float* res) { return nc_get_var_float(ncid, varid, res); }
			template<> int getNCvar<int>(int ncid, int varid, int* res) { return nc_get_var_int(ncid, varid, res); }
			template<> int getNCvar<short>(int ncid, int varid, short* res) { return nc_get_var_short(ncid, varid, res); }

			template <class DataType>
			Eigen::Matrix<DataType, Eigen::Dynamic, Eigen::Dynamic>
				getMatrix(const char* name, std::shared_ptr<netcdf_handle> h)
			{
				int status = 0;
				int parentId = h->file;
				int varid = 0;
				status = nc_inq_varid(parentId, name, &varid);
				if (status) h->handle_error(status);
				nc_type vartype;
				status = nc_inq_vartype(parentId, varid, &vartype);
				if (status) h->handle_error(status);
				if (!AttrMatches<typename DataType>(vartype)) RTthrow debug::xBadInput(name);

				int ndims = 0;
				status = nc_inq_varndims(parentId, varid, &ndims);
				if (status) h->handle_error(status);
				if (ndims < 0 || ndims > 2) RTthrow debug::xArrayOutOfBounds();

				int dimids[2] = {0, 0};
				status = nc_inq_vardimid(parentId, varid, dimids);

				// Get dimension lengths
				size_t dimLens[2] = {1, 1};
				for (int i=0; i<ndims; ++i)
				{
					status = nc_inq_dimlen(parentId, dimids[i], &(dimLens[i]));
					if (status) h->handle_error(status);
				}

				// Define the matrix
				/// \note Variable is read with last dimension varying fastest. No idea how this converts to Eigen. Will 
				/// determine when reading actual data.
				Eigen::Matrix<typename DataType, Eigen::Dynamic, Eigen::Dynamic> res(dimLens[0], dimLens[1]);
				status = getNCvar<typename DataType>(parentId, varid, res.data());
				if (status) h->handle_error(status);
				return res;
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
}

namespace rtmath
{
	namespace registry
	{
		using std::shared_ptr;
		using namespace rtmath::plugins::netcdf;
		
		template<>
		shared_ptr<IOhandler>
			read_file_type_multi<::rtmath::data::arm::arm_info>
			(shared_ptr<IOhandler> sh, shared_ptr<IO_options> opts,
			::rtmath::data::arm::arm_info *s)
		{
			std::string filename = opts->filename();
			IOhandler::IOtype iotype = opts->getVal<IOhandler::IOtype>("iotype", IOhandler::IOtype::READONLY);
			//IOhandler::IOtype iotype = opts->iotype();
			std::string key = opts->getVal<std::string>("key");
			using std::shared_ptr;
			std::shared_ptr<netcdf_handle> h;
			if (!sh)
			{
				h = std::shared_ptr<netcdf_handle>(new netcdf_handle(filename.c_str(), iotype));
			} else {
				if (sh->getId() != PLUGINID) RTthrow debug::xDuplicateHook("Bad passed plugin");
				h = std::dynamic_pointer_cast<netcdf_handle>(sh);
			}

			
			s->hash = rtmath::HASHfile(filename);
			// Product id from filename
			boost::filesystem::path pfile(filename);
			s->filesize = static_cast<size_t>(boost::filesystem::file_size(pfile));
			auto pfilename = pfile.filename();
			std::string sfilename = pfilename.string();
			s->filename = sfilename;
			// The filename follows the pattern {nsa}{product}{subsite(2 chars)}. ...
			s->product = sfilename.substr(3, sfilename.find_first_of('.') - 5);
			
			auto getAttrString = [&](const char* name, int varid) -> std::string
			{
				int status = 0;
				size_t len = 0;
				status = nc_inq_attlen (h->file, varid, name, &len);
				if (status) h->handle_error(status);
				std::unique_ptr<char[]> txtbuf(new char[len+1]);
				status = nc_get_att_text(h->file, varid, name, txtbuf.get());
				if (status) h->handle_error(status);
				std::string res(txtbuf.get());
				return res;
			};

			// Read attributes site_id, facility_id, data_level
			s->site = getAttrString("site_id", NC_GLOBAL);
			s->subsiteFull = getAttrString("facility_id", NC_GLOBAL);
			s->subsite = s->subsiteFull.substr(0,2);
			s->datalevel = getAttrString("proc_level", NC_GLOBAL);

			// Read in datasets lat, lon, alt, base_time, time_offsets
			auto time_offsets = getMatrix<double>("time_offset", h);
			auto base_time = getMatrix<int>("base_time", h);
			using namespace boost::posix_time;
			using namespace boost::gregorian;
			date b(1970,Jan,1);
			s->startTime = ptime(b, seconds( static_cast<long>(base_time(0,0))));
			
			s->endTime = s->startTime + seconds( static_cast<long>(time_offsets.bottomRows(1)(0,0)));
			auto lat = getMatrix<float>("lat", h);
			s->lat = lat(0,0);
			auto lon = getMatrix<float>("lon", h);
			s->lon = lon(0,0);
			auto alt = getMatrix<float>("alt", h);
			s->alt = alt(0,0);

			//std::cerr << s->startTime << std::endl;
			//std::cerr << s->endTime << std::endl;

			return h;
		}

		template<>
		std::shared_ptr<IOhandler>
			read_file_type_vector<::rtmath::data::arm::arm_info>
			(std::shared_ptr<IOhandler> sh, std::shared_ptr<IO_options> opts,
			std::vector<boost::shared_ptr<::rtmath::data::arm::arm_info> > &s)
		{
			RTthrow debug::xUnimplementedFunction();
			return sh;
		}
	}
}
