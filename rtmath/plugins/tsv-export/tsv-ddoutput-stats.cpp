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


#include <boost/math/constants/constants.hpp>
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>

#include "../../rtmath/rtmath/defs.h"
#include "../../rtmath/rtmath/ddscat/shapefile.h"
#include "../../rtmath/rtmath/ddscat/shapestats.h"
#include "../../rtmath/rtmath/common_templates.h"
#include "../../rtmath/rtmath/hash.h"
#include "../../rtmath/rtmath/splitSet.h"
#include "../../rtmath/rtmath/ddscat/ddOriData.h"
#include "../../rtmath/rtmath/ddscat/ddOutput.h"
#include "../../rtmath/rtmath/ddscat/ddUtil.h"
#include "../../rtmath/rtmath/ddscat/ddRunSet.h"
#include "../../rtmath/rtmath/ddscat/ddpar.h"
#include "../../rtmath/rtmath/ddscat/ddweights.h"
#include "../../rtmath/rtmath/ddscat/rotations.h"
#include "../../rtmath/rtmath/units.h"
#include "../../rtmath/rtmath/ddscat/ddweights.h"
#include "../../rtmath/rtmath/ddscat/ddavg.h"
#include "../../rtmath/rtmath/plugin.h"
#include "../../rtmath/rtmath/error/debug.h"
#include "../../rtmath/rtmath/error/error.h"

#include "plugin-tsv.h"


namespace rtmath {
	namespace plugins {
		namespace tsv {
			using namespace std;
			using namespace rtmath::ddscat;
			using namespace rtmath::ddscat::weights;
			using rtmath::ddscat::ddOutput;
			using rtmath::ddscat::rotations;
			using namespace rtmath::registry;

			struct tsv_ddoutput_stats_handle : public tsv_handle
			{
				tsv_ddoutput_stats_handle(const char* filename, IOtype t) : tsv_handle(filename, t, PLUGINID_DDSTATS) { open(filename, t); }
				virtual ~tsv_ddoutput_stats_handle() {}
				virtual void writeHeader() override
				{
					(*(file.get())) << "Shape Hash\t"
						"Frequency (GHz)\tWavelength (um)\tSize Parameter\tDipole Spacing (um)\t"
						"Temperature (K)\tAeff (um)\tV_Ice (um^3)\tV_Ice (dipoles^3)\t"
						"SA_Ice (um^2)\tSA_Ice (dipoles^2)\tSA_V_Ice (um^-1)\tSA_V_Ice (dipoles^-1)\t"
						"Number of Dipoles\tBetas\tThetas\tPhis"
						<< std::endl;
					;
				}
			};

			shared_ptr<::rtmath::registry::IOhandler>
				export_tsv_ddori_stats
				(shared_ptr<::rtmath::registry::IOhandler> sh, shared_ptr<::rtmath::registry::IO_options> opts,
				const ::rtmath::ddscat::ddOutput *ddOut)
			{
				std::string exporttype = opts->exportType();
				if (exporttype != "stats") RTthrow debug::xUnimplementedFunction();
				std::string filename = opts->filename();
				std::string sDescrip = opts->getVal<std::string>("description", "");
				IOhandler::IOtype iotype = opts->iotype();
				using std::shared_ptr;
				std::shared_ptr<tsv_ddoutput_stats_handle> h;
				if (!sh)
					h = std::shared_ptr<tsv_ddoutput_stats_handle>(new tsv_ddoutput_stats_handle(filename.c_str(), iotype));
				else {
					if (sh->getId() != PLUGINID_DDSTATS) RTthrow debug::xDuplicateHook("Bad passed plugin");
					h = std::dynamic_pointer_cast<tsv_ddoutput_stats_handle>(sh);
				}

				ddOutput* ddOutRW = const_cast<ddOutput*>(ddOut);
				ddOutRW->loadShape(true);

				rotations rots(*(ddOut->parfile));
				weights::ddWeightsDDSCAT wts(rots);
				boost::shared_ptr<weights::DDSCAT3dWeights> ow(new weights::DDSCAT3dWeights(wts));
				weights::ddOutputAvg averager(ow);

				boost::shared_ptr<ddOutput> ddOutWithAvg;
				Eigen::MatrixXf outwts;
				averager.doAvgAll(ddOut, ddOutWithAvg, outwts);


				//boost::shared_ptr<const ddOriData> fiso = boost::shared_ptr<const ddOriData>(new ddOriData(*ddOutWithAvg));

				//auto data = fiso->selectData();

				// Also pull in stats (should be loaded before write)
				auto stats = ddOut->stats;


				const double pi = boost::math::constants::pi<double>();
				double lambda = units::conv_spec("GHz", "um").convert(ddOut->freq);
				double sizep = 2. * pi * ddOut->aeff / lambda;
				double Vice_um = pow(ddOut->aeff, 3.) * 4. * pi / 3;

				// calculate interdipole spacing from aeff_um and V_ice_di
				double aeff_di = stats->aeff_dipoles_const;
				double d = ddOut->aeff / aeff_di;
				//double aeff_di = ddOut->aeff / data(ddOutput::stat_entries::D);
				double Vice_di = pow(aeff_di, 3.) * 4. * pi / 3;
				double SAice_um = 4. * pi * pow(ddOut->aeff, 2.);
				double SAice_di = 4. * pi * pow(aeff_di, 2.);
				double SA_V_ice_di = SAice_di / Vice_di;
				double SA_V_ice_um = SAice_um / Vice_um;

				/// \todo Write out the more relevant stats (AR, Voronoi stuff, fractal dimension.....)

				(*(h->file.get())) << ddOut->shapeHash.lower << "\t"
					<< ddOut->freq << "\t" << lambda << "\t" << sizep << "\t"
					<< d << "\t" << ddOut->temp << "\t"
					<< ddOut->aeff << "\t" << Vice_um << "\t" << Vice_di << "\t" << SAice_um << "\t" << SAice_di << "\t"
					<< SA_V_ice_um << "\t" << SA_V_ice_di << "\t"
					<< ddOut->shape->numPoints << "\t"
					<< rots.bN() << "\t" << rots.tN() << "\t" << rots.pN();


				(*(h->file.get())) << std::endl;

				return h; // Pass back the handle
			}

		}
	}
}

