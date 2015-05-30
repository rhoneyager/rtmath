/// \brief Provides ImageMagick file IO
#define _SCL_SECURE_NO_WARNINGS

#include <Eigen/Core>
#include <Eigen/Dense>
#include <algorithm>
#include <string>
#include <boost/algorithm/string/trim.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include <Ryan_Debug/error.h>
#include "../../rtmath/rtmath/defs.h"
#include "../../rtmath/rtmath/error/debug.h"
#include "../../rtmath/rtmath/error/error.h"
#include "../../rtmath/rtmath/data/arm_info.h"
#include "../../rtmath/rtmath/data/arm_scanning_radar_sacr.h"
#include "../../rtmath/rtmath/plugin.h"

#include "plugin-netcdf.h"
#include <netcdf.h>


namespace rtmath
{
	namespace registry
	{
		using std::shared_ptr;
		using namespace rtmath::plugins::netcdf;

		template<>
		shared_ptr<IOhandler>
			read_file_type_multi<::rtmath::data::arm::arm_scanning_radar_sacr>
			(shared_ptr<IOhandler> sh, shared_ptr<IO_options> opts,
			boost::shared_ptr<::rtmath::data::arm::arm_scanning_radar_sacr > s,
			std::shared_ptr<const rtmath::registry::collectionTyped<::rtmath::data::arm::arm_scanning_radar_sacr> > filter)
		{
			using namespace ::rtmath::data::arm;
				std::string filename = opts->filename();
				IOhandler::IOtype iotype = opts->getVal<IOhandler::IOtype>("iotype", IOhandler::IOtype::READONLY);
				//IOhandler::IOtype iotype = opts->iotype();
				std::string key = opts->getVal<std::string>("key");
				using std::shared_ptr;
				std::shared_ptr<netcdf_handle> h = registry::construct_handle
					<registry::IOhandler, netcdf_handle>(
					sh, PLUGINID, [&](){return std::shared_ptr<netcdf_handle>(
					new netcdf_handle(filename.c_str(), iotype)); });


				// Read base file information
				if (!s->info)
				{
					boost::shared_ptr<arm_info> info(new arm_info);
					read_file_type_multi<arm_info>(h, opts, info, nullptr); // filter needs to be null here to make the call bind
					s->info = info;
				}


				// Read time_offsets
				s->time_offsets = getMatrix<double>("time_offset", h);

				// Read range table, azimuths, elevations
				s->ranges = getMatrix<float>("range", h);
				s->azimuths = getMatrix<float>("azimuth", h);
				s->elevations = getMatrix<float>("elevation", h);

				// Read sweep_start_ray_index, sweep_stop_ray_index
				s->sweep_start_ray_index = getMatrix<int>("sweep_start_ray_index", h);
				s->sweep_end_ray_index = getMatrix<int>("sweep_end_ray_index", h);

				// Read reflectivity
				// Requires some conversion
				int status;
				float scaleFactor, scaleOffset;
				int reflId;
				status = nc_inq_varid(h->file, "reflectivity", &reflId);
				if (status) h->handle_error(status);
				status = nc_get_att_float(h->file, reflId, "scale_factor", &scaleFactor);
				if (status) h->handle_error(status);
				status = nc_get_att_float(h->file, reflId, "add_offset", &scaleOffset);
				if (status) h->handle_error(status);

				Eigen::Matrix<short, Eigen::Dynamic, Eigen::Dynamic> reflectivity_raw 
					= getMatrix<short>("reflectivity", h);

				//s->reflectivity.resize(reflectivity_raw.rows(), reflectivity_raw.cols());
				s->reflectivity = (reflectivity_raw.array().cast<float>() * scaleFactor) + scaleOffset;
				//s->reflectivity *= scaleFactor;
				//s->reflectivity += scaleOffset;

				/*
				auto getAttrString = [&](const char* name, int varid) -> std::string
				{
					int status = 0;
					size_t len = 0;
					status = nc_inq_attlen(h->file, varid, name, &len);
					if (status) h->handle_error(status);
					std::unique_ptr<char[]> txtbuf(new char[len + 1]);
					status = nc_get_att_text(h->file, varid, name, txtbuf.get());
					if (status) h->handle_error(status);
					std::string res(txtbuf.get(), len+1); // The netcdf storage of some of the arm attributes was flawed
					return res;
				};
				*/

				return h;
		}

		template<>
		std::shared_ptr<IOhandler>
			read_file_type_vector<::rtmath::data::arm::arm_scanning_radar_sacr>
			(std::shared_ptr<IOhandler> sh, std::shared_ptr<IO_options> opts,
			std::vector<boost::shared_ptr<::rtmath::data::arm::arm_scanning_radar_sacr> > &s,
			std::shared_ptr<const rtmath::registry::collectionTyped<::rtmath::data::arm::arm_scanning_radar_sacr> >)
		{
			RDthrow(debug::xUnimplementedFunction());
			return sh;
		}
	}
}
