/// \brief Provides ImageMagick file IO

#include "../../rtmath/rtmath/defs.h"
#include "../../rtmath/rtmath/data/arm_info.h"
#include "../../rtmath/rtmath/data/arm_scanning_radar_sacr.h"

/*
#define _SCL_SECURE_NO_WARNINGS
#define _CRT_SECURE_NO_WARNINGS
#pragma warning(push)
#pragma warning( disable : 4267 ) // conversion from 'size_t' to 'Ssiz_t', possible loss of data (ROOT)
#pragma warning( disable : 4800 ) // forcing value to bool 'true' or 'false' (performance warning) (ROOT)
#pragma warning( disable : 4018 ) // signed/unsigned mismatch (ROOT)
#pragma warning( disable : 4244 ) // conversion from 'float' to '__int64', possible loss of data (ROOT)

#include <Windows4Root.h>
#include <TCanvas.h>
#include <TGraph.h>
#include <TGraph2D.h>
#include <TGraphPolargram.h>
#include <TGraphPolar.h>

#pragma warning(pop)
*/

#include <Eigen/Core>
#include <Eigen/Dense>
#include <algorithm>
#include <string>
#include <boost/algorithm/string/trim.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/math/constants/constants.hpp>


#include "../../rtmath/rtmath/plugin.h"
#include "../../rtmath/rtmath/error/debug.h"
#include "../../rtmath/rtmath/error/error.h"

#include "plugin-tsv.h"
#ifdef min
#undef min
#endif
#ifdef max
#undef max
#endif


namespace rtmath
{
	using std::shared_ptr;
	using namespace rtmath::registry;
	namespace plugins
	{
		namespace tsv
		{
			struct tsv_sacr_reflectivity_handle : public rtmath::registry::IOhandler
			{
				tsv_sacr_reflectivity_handle(const char* filename, IOtype t) : IOhandler(PLUGINID_SACR_REFL) { open(filename, t); }
				virtual ~tsv_sacr_reflectivity_handle() {}
				void open(const char* filename, IOtype t)
				{
					using namespace boost::filesystem;
					switch (t)
					{
					case IOtype::EXCLUSIVE:
					case IOtype::DEBUG:
					case IOtype::READONLY:
						RTthrow(debug::xOtherError());
						break;
					case IOtype::CREATE:
						if (exists(path(filename))) RTthrow(debug::xFileExists());
					case IOtype::TRUNCATE:
						file = std::shared_ptr<std::ofstream>(new std::ofstream(filename, std::ios_base::trunc));
						writeHeader();
						break;
					case IOtype::READWRITE:
						{
							bool e = false;
							if (exists(path(filename))) e = true;
							file = std::shared_ptr<std::ofstream>(new std::ofstream(filename, std::ios_base::app));
							if (!e) writeHeader(); // If the file had to be created, give it a header
						}
						break;
					}
				}
				void writeHeader()
				{
					(*(file.get())) << "Filename\tPass\tTime\tElevation (degrees)\t"
						"Azimuth (degrees)\tRange (m)\tx (m)\ty (m)\tReflectivity (dBz)"
						<< std::endl;
					;
				}
				std::shared_ptr<std::ofstream> file;
			};


			/// \param scale indicates that the values should be scaled by elevation angle
			void write_sacr_reflectivity
				(shared_ptr<tsv_sacr_reflectivity_handle> h, shared_ptr<IO_options> opts,
				const boost::shared_ptr<const ::rtmath::data::arm::arm_scanning_radar_sacr > s)
			{
				size_t numPass = opts->getVal<size_t>("pass", 0);
				if (numPass > (size_t) s->sweep_start_ray_index.rows() && numPass > (size_t) s->sweep_start_ray_index.cols())
					RTthrow(debug::xArrayOutOfBounds());
				int startIndex = s->sweep_start_ray_index(numPass);
				int endIndex = s->sweep_end_ray_index(numPass);
				int span = endIndex - startIndex;

				auto &ranges = s->ranges;
				auto &AllAzimuths = s->azimuths;
				auto &AllElevations = s->elevations;
				auto &AllTime_offsets = s->time_offsets;

				Eigen::Array<float, Eigen::Dynamic, Eigen::Dynamic> refl_pass 
					= s->reflectivity.block(startIndex, 0, span, ranges.size());
				// refl rows are for each time index
				// cols are for the range bins

				Eigen::ArrayXf azimuths = AllAzimuths.block(startIndex, 0, span, 1),
					elevations = AllElevations.block(startIndex, 0, span, 1);
				Eigen::ArrayXd time_offsets = AllTime_offsets.block(startIndex, 0, span, 1);

				// Standard elevations field ranges from 0 (horizon) to 180 (horizon).
				// So, x = cos( d2rad( elevation ) ), y = sin( d2rad( elevation ) )
				const float pi = boost::math::constants::pi<float>();
				Eigen::ArrayXf elevationsRad = elevations * pi / 180.f;
				Eigen::ArrayXf xs, ys;
				xs = elevationsRad.cos();// * ranges;
				ys = elevationsRad.sin();// * ranges;

				// Start time
				using namespace boost::posix_time;
				using namespace boost::gregorian;
				boost::posix_time::ptime startTime = s->info->startTime + seconds(static_cast<long>(time_offsets(0))); 
				// End time
				boost::posix_time::ptime endTime = startTime + seconds(static_cast<long>(AllTime_offsets(endIndex)));


				//std::ostringstream otitle;
				//otitle << "Pass " << numPass << " for " << s->info->filename;
				//std::string stitle = otitle.str();
				for (int i=0; i<span; ++i)
					for (int rbin=0; rbin < ranges.size(); ++rbin)
				{
					boost::posix_time::ptime t = s->info->startTime + seconds(static_cast<long>(time_offsets(i))); 
					*(h->file.get()) << s->info->filename << "\t" << numPass << "\t"
						<< t << "\t" << elevations(i) << "\t" << azimuths(i) << "\t" << ranges(rbin) << "\t"
						<< xs(i) * ranges(rbin) << "\t" << ys(i) * ranges(rbin) << "\t" << refl_pass(i,rbin) << std::endl;
				}
			}
		}
	}

	namespace registry
	{
		using namespace rtmath::plugins::tsv;



		template<>
		std::shared_ptr<IOhandler>
			write_file_type_multi<::rtmath::data::arm::arm_scanning_radar_sacr>
			(std::shared_ptr<IOhandler> sh, std::shared_ptr<IO_options> opts,
			const boost::shared_ptr<const ::rtmath::data::arm::arm_scanning_radar_sacr > s)
		{
			using namespace ::rtmath::data::arm;
			std::string filename = opts->filename();
			IOhandler::IOtype iotype = opts->getVal<IOhandler::IOtype>("iotype", IOhandler::IOtype::TRUNCATE);
			//IOhandler::IOtype iotype = opts->iotype();
			std::string key = opts->getVal<std::string>("key");
			std::string exporttype = opts->exportType();

			using std::shared_ptr;
			std::shared_ptr<tsv_sacr_reflectivity_handle> h;
			if (!sh) h = std::shared_ptr<tsv_sacr_reflectivity_handle>(new tsv_sacr_reflectivity_handle(filename.c_str(), iotype));
			else {
				if (sh->getId() != PLUGINID_SACR_REFL) RTthrow(debug::xDuplicateHook());
				h = std::dynamic_pointer_cast<tsv_sacr_reflectivity_handle>(sh);
			}

			if (exporttype == "reflectivity_angscaled")
			{
				RTthrow(debug::xUnimplementedFunction());
				//write_sacr_reflectivity(h, opts, s);
			} else if (exporttype == "reflectivity")
			{
				write_sacr_reflectivity(h, opts, s);
			}
			
			return h;
		}

	}
}
