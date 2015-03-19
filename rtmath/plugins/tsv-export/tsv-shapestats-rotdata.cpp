/// \brief Provides tsv file IO
#define _SCL_SECURE_NO_WARNINGS

#pragma warning( disable : 4503 ) // decorated length exceeded

#include <array>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <fstream>
#include <string>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <tuple>

#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/kurtosis.hpp>
#include <boost/accumulators/statistics/max.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <boost/accumulators/statistics/min.hpp>
#include <boost/accumulators/statistics/skewness.hpp>
#include <boost/accumulators/statistics/variance.hpp>


#include "../../rtmath/rtmath/defs.h"
#include "../../rtmath/rtmath/ddscat/shapefile.h"
#include "../../rtmath/rtmath/ddscat/shapestats.h"
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

using namespace rtmath::registry;
namespace rtmath {
	namespace plugins {
		namespace tsv {
			using std::shared_ptr;
			using rtmath::ddscat::ddOutput;

			struct tsv_ar_handle : public rtmath::registry::IOhandler
			{
				tsv_ar_handle(const char* filename, IOtype t) : IOhandler(PLUGINID_ARS) { open(filename, t); }
				virtual ~tsv_ar_handle() {}
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
					(*(file.get())) << "Hash\tV_dipoles_const\t"
						"aeff_um\tNumber of Rotations\t"
						"min_as_abs_xy\tmin_as_abs_xz\tmin_as_abs_yz\t"
						"max_as_abs_xy\tmax_as_abs_xz\tmax_as_abs_yz\t"
						"mean_as_abs_xy\tmean_as_abs_xz\tmean_as_abs_yz\t"
						"skewness_as_abs_xy\tskewness_as_abs_xz\tskewness_as_abs_yz\t"
						"kurtosis_as_abs_xy\tkurtosis_as_abs_xz\tkurtosis_as_abs_yz\t"
						"variance_as_abs_xy\tvariance_as_abs_xz\tvariance_as_abs_yz"
						<< std::endl;
					;
				}
				std::shared_ptr<std::ofstream> file;
			};

			shared_ptr<IOhandler>
				export_tsv_ar_rot_data
				(shared_ptr<IOhandler> sh, shared_ptr<IO_options> opts,
				const boost::shared_ptr<const rtmath::ddscat::stats::shapeFileStats > s)
			{
				std::string filename = opts->filename();
				IOhandler::IOtype iotype = opts->iotype();

				using std::shared_ptr;
				std::shared_ptr<tsv_ar_handle> h;
				if (!sh)
				{
					h = std::shared_ptr<tsv_ar_handle>(new tsv_ar_handle(filename.c_str(), iotype));
				}
				else {
					if (sh->getId() != PLUGINID_ARS) RTthrow(debug::xDuplicateHook());
					h = std::dynamic_pointer_cast<tsv_ar_handle>(sh);
				}

				double dSpacing = opts->getVal<double>("dSpacing", 0);
				double aeff_um = s->Scircum_sphere.aeff_V * dSpacing;
				size_t nRots = s->rotstats.size();

				// Write the aspect ratio stat data
				// Go through the loaded rotations and calculate ar stats

				using namespace boost::accumulators;
				//using namespace boost::accumulators::tag;

				// Do two passes to be able to renormalize coordinates
				accumulator_set<double, boost::accumulators::stats<
					tag::mean, tag::min, tag::max,
					tag::skewness,
					tag::kurtosis,
					tag::variance
				> > m_xy, m_yz, m_xz;

				/// \todo Modify stats to imclude aspect ratios that are in the target reference frame

				for (const auto &rot : s->rotstats)
				{
					auto &tbl = rot.get<0>();
					auto &mat = rot.get<1>();
					auto &vec = rot.get<2>();
					using namespace rtmath::ddscat::stats;

					m_xy(mat[rotColDefs::AS_ABS](0, 1));
					m_yz(mat[rotColDefs::AS_ABS](1, 2));
					m_xz(mat[rotColDefs::AS_ABS](0, 2));
				}


				(*(h->file.get())) << s->_shp->hash().lower << "\t" << s->V_dipoles_const << "\t"
					<< aeff_um << "\t" << nRots << "\t"
					<< ::boost::accumulators::min(m_xy) << "\t" << ::boost::accumulators::min(m_yz) << "\t" << ::boost::accumulators::min(m_xz) << "\t"
					<< ::boost::accumulators::max(m_xy) << "\t" << ::boost::accumulators::max(m_yz) << "\t" << ::boost::accumulators::max(m_xz) << "\t"
					<< boost::accumulators::mean(m_xy) << "\t" << boost::accumulators::mean(m_yz) << "\t" << boost::accumulators::mean(m_xz) << "\t"
					<< boost::accumulators::skewness(m_xy) << "\t" << boost::accumulators::skewness(m_yz) << "\t" << boost::accumulators::skewness(m_xz) << "\t"
					<< boost::accumulators::kurtosis(m_xy) << "\t" << boost::accumulators::kurtosis(m_yz) << "\t" << boost::accumulators::kurtosis(m_xz) << "\t"
					<< boost::accumulators::variance(m_xy) << "\t" << boost::accumulators::variance(m_yz) << "\t" << boost::accumulators::variance(m_xz)
					<< std::endl;

				return h; // Pass back the handle
			}


		}
	}
}
