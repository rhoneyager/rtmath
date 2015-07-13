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
#include <boost/math/constants/constants.hpp>

#include "../../rtmath/rtmath/defs.h"
#include "../../rtmath/rtmath/ddscat/ddavg.h"
#include "../../rtmath/rtmath/ddscat/shapefile.h"
#include "../../rtmath/rtmath/ddscat/shapestats.h"
#include "../../rtmath/rtmath/common_templates.h"
#include "../../rtmath/rtmath/ddscat/ddOriData.h"
#include "../../rtmath/rtmath/ddscat/ddOutput.h"
#include "../../rtmath/rtmath/ddscat/ddUtil.h"
#include "../../rtmath/rtmath/ddscat/ddRunSet.h"
#include "../../rtmath/rtmath/ddscat/ddpar.h"
#include "../../rtmath/rtmath/ddscat/rotations.h"
#include "../../rtmath/rtmath/ddscat/ddweights.h"
#include "../../rtmath/rtmath/units.h"
#include "../../rtmath/rtmath/plugin.h"
#include <Ryan_Debug/debug.h>
#include <Ryan_Debug/error.h>

#include "plugin-tsv.h"


namespace rtmath {
	namespace plugins {
		namespace tsv {
			using std::shared_ptr;
			using namespace rtmath::ddscat;
			using rtmath::ddscat::ddOutput;
			using rtmath::ddscat::rotations;
			using namespace Ryan_Debug::registry;

			struct tsv_ddoutput_iso_handle : public Ryan_Debug::registry::IOhandler
			{
				tsv_ddoutput_iso_handle(const char* filename, IOtype t) : IOhandler(PLUGINID_DDISO) { open(filename, t); }
				virtual ~tsv_ddoutput_iso_handle() {}
				void open(const char* filename, IOtype t)
				{
					using namespace boost::filesystem;
					switch (t)
					{
					case IOtype::EXCLUSIVE:
					case IOtype::DEBUG:
					case IOtype::READONLY:
						RDthrow(Ryan_Debug::error::xOtherError());
						break;
					case IOtype::CREATE:
						if (exists(path(filename))) RDthrow(Ryan_Debug::error::xFileExists());
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
					(*(file.get())) << "Shape Hash\tDescription\tDDSCAT Version Tag\t"
						"Frequency (GHz)\tWavelength (um)\tSize Parameter\tDipole Spacing (um)\t"
						"Temperature (K)\tAeff (um)\tV_Ice (um^3)\tV_Ice (dipoles^3)\tSA_Ice (um^2)\tSA_Ice (dipoles^2)\tSA_V_Ice (um^-1)\tSA_V_Ice (dipoles^-1)\t"
						"Number of Dipoles\tBetas\tThetas\tPhis\t"
						"Qsca_iso\tQbk_iso\tQbk_normalized\tQabs_iso\tQext_iso\tG_iso\t"
						"V_Voronoi\tSA_Voronoi\tSA_V_Voronoi\taeff_SA_Voronoi\taeff_V_Voronoi\tf_Voronoi\tRatio_Voronoi_Ice\t"
						"V_Circum_Sphere\tSA_Circum_Sphere\tSA_V_Circum_Sphere\taeff_SA_Circum_Sphere\taeff_V_Circum_Sphere\tf_Circum_Sphere\tRatio_Circum_Ice\tRatio_Voronoi_Circum\t"
						"V_Convex\tSA_Convex\tSA_V_Convex\taeff_SA_Convex\taeff_V_Convex\tf_Convex\tRatio_Convex_Ice\t"
						"V_Ellipsoid_Max\tSA_Ellipsoid_Max\tSA_V_Ellipsoid_Max\taeff_SA_Ellipsoid_Max\taeff_V_Ellipsoid_Max\tf_Ellipsoid_Max\tRatio_Ellipsoid_Max_Ice"
						<< std::endl;
					;
				}
				std::shared_ptr<std::ofstream> file;
			};

			shared_ptr<::Ryan_Debug::registry::IOhandler>
				export_tsv_ddori_iso_data
				(shared_ptr<::Ryan_Debug::registry::IOhandler> sh, shared_ptr<::Ryan_Debug::registry::IO_options> opts,
				const boost::shared_ptr<const  ::rtmath::ddscat::ddOutput > ddOut)
			{
				std::string exporttype = opts->exportType();
				if (exporttype != "isotropic_data") RDthrow(Ryan_Debug::error::xUnimplementedFunction());
				std::string filename = opts->filename();
				std::string sDescrip = opts->getVal<std::string>("description","");
				IOhandler::IOtype iotype = opts->iotype();
				using std::shared_ptr;
				std::shared_ptr<tsv_ddoutput_iso_handle> h;
				if (!sh)
					h = std::shared_ptr<tsv_ddoutput_iso_handle>(new tsv_ddoutput_iso_handle(filename.c_str(), iotype));
				else {
					if (std::string(sh->getId()) != std::string(PLUGINID_DDISO)) RDthrow(Ryan_Debug::error::xDuplicateHook());
					h = std::dynamic_pointer_cast<tsv_ddoutput_iso_handle>(sh);
				}

				// Initial file creation handles writing the initial header.
				// So, just write the data.

				boost::shared_ptr<const ddOutput> ddOutRW = ddOut;
				ddOutRW->loadShape(true);

				rotations rots(*(ddOut->parfile));
				weights::ddWeightsDDSCAT wts(rots);
				boost::shared_ptr<weights::DDSCAT3dWeights> ow(new weights::DDSCAT3dWeights(wts));
				weights::ddOutputAvg averager(ow);

				boost::shared_ptr<ddOutput> ddOutWithAvg;
				Eigen::MatrixXf outwts;
				averager.doAvgAll(ddOut.get(), ddOutWithAvg, outwts);


				boost::shared_ptr<const ddOriData> fori = ddOriData::generate(*ddOutWithAvg);

				auto data = fori->selectData();

				// Also pull in stats (should be loaded before write)
				auto stats = ddOut->stats;

				// Using a reference in object construction makes 
				// data accesses hard. Perhaps I should modify the structure a bit.
				//if (ddOut->avgdata.hasAvg) fori = boost::shared_ptr<const ddOriData>(new ddOriData(*ddOutRW));
				//else {
				//	fori = boost::shared_ptr<const ddOriData>(new ddOriData(*ddOutRW, 0));
				//}

				
				/*
				if (!ddOut->stats)
				{
				if (ddOut->shape)
				ddOut->stats = boost::shared_ptr<rtmath::ddscat::stats::shapeFileStats>(stats::shapeFileStats::genStats(ddOut->shape));
				}
				// Calculate scaled quantities
				double ds = ddOut->avg->dipoleSpacing();

				double Vvoro = ddOut->stats->SVoronoi_hull.V * pow(ds,3.);
				double Svoro = ddOut->stats->SVoronoi_hull.SA * pow(ds,2.);
				double fvoro = ddOut->stats->SVoronoi_hull.f;

				double Vconv = ddOut->stats->Sconvex_hull.V * pow(ds,3.);
				double Sconv = ddOut->stats->Sconvex_hull.SA * pow(ds,2.);
				double fconv = ddOut->stats->Sconvex_hull.f;

				double Vellm = ddOut->stats->Sellipsoid_max.V * pow(ds,3.);
				double Sellm = ddOut->stats->Sellipsoid_max.SA * pow(ds,2.);
				double fellm = ddOut->stats->Sellipsoid_max.f;

				double Vcirc = ddOut->stats->Scircum_sphere.V * pow(ds,3.);
				double Scirc = ddOut->stats->Scircum_sphere.SA * pow(ds,2.);
				double fcirc = ddOut->stats->Scircum_sphere.f;
				*/

				if (!sDescrip.size()) sDescrip = ddOut->description;
				const double pi = boost::math::constants::pi<double>();
				double lambda = units::conv_spec("GHz", "um").convert(ddOut->freq);
				double sizep = 2. * pi * ddOut->aeff / lambda;
				double Vice_um = pow(ddOut->aeff, 3.) * 4. * pi / 3;
				double aeff_di = ddOut->aeff / data(ddOutput::stat_entries::D);
				double Vice_di = pow(aeff_di, 3.) * 4. * pi / 3;
				double SAice_um = 4. * pi * pow(ddOut->aeff, 2.);
				double SAice_di = 4. * pi * pow(aeff_di, 2.);
				double SA_V_ice_di = SAice_di / Vice_di;
				double SA_V_ice_um = SAice_um / Vice_um;


				(*(h->file.get())) << ddOut->shapeHash.lower << "\t" << sDescrip << "\t"
					<< ddOut->ddvertag << "\t"
					<< ddOut->freq << "\t" << lambda << "\t" 
					<< sizep << "\t"
					<< data(ddOutput::stat_entries::D) << "\t"
					<< ddOut->temp << "\t"
					<< ddOut->aeff << "\t" << Vice_um << "\t" << Vice_di << "\t" << SAice_um << "\t" << SAice_di << "\t"
					<< SA_V_ice_um << "\t" << SA_V_ice_di << "\t"
					<< ddOut->s.num_dipoles << "\t"
					<< rots.bN() << "\t" << rots.tN() << "\t" << rots.pN() << "\t"
					<< data(ddOutput::stat_entries::QSCAM) << "\t"
					<< data(ddOutput::stat_entries::QBKM) << "\t"
					<< data(ddOutput::stat_entries::QBKM) * 4. * pi << "\t"
					<< data(ddOutput::stat_entries::QABSM) << "\t"
					<< data(ddOutput::stat_entries::QEXTM) << "\t"
					<< data(ddOutput::stat_entries::G1M);
				if (stats)
				{
					(*(h->file.get())) << "\t" << stats->SVoronoi_hull.V << "\t" << stats->SVoronoi_hull.SA << "\t" << stats->SVoronoi_hull.SA / stats->SVoronoi_hull.V
						<< "\t" << stats->SVoronoi_hull.aeff_SA << "\t" << stats->SVoronoi_hull.aeff_V << "\t" << stats->SVoronoi_hull.f
						<< "\t" << ((stats->SVoronoi_hull.SA / stats->SVoronoi_hull.V) / (SA_V_ice_di))
						<< "\t" << stats->Scircum_sphere.V << "\t" << stats->Scircum_sphere.SA << "\t" << stats->Scircum_sphere.SA / stats->Scircum_sphere.V
						<< "\t" << stats->Scircum_sphere.aeff_SA << "\t" << stats->Scircum_sphere.aeff_V << "\t" << stats->Scircum_sphere.f
						<< "\t" << ((stats->Scircum_sphere.SA / stats->Scircum_sphere.V) / (SA_V_ice_di))
						<< "\t" << ((stats->Scircum_sphere.SA / stats->Scircum_sphere.V) / (stats->Scircum_sphere.SA / stats->Scircum_sphere.V))
						<< "\t" << stats->Sconvex_hull.V << "\t" << stats->Sconvex_hull.SA << "\t" << stats->Sconvex_hull.SA / stats->Sconvex_hull.V
						<< "\t" << stats->Sconvex_hull.aeff_SA << "\t" << stats->Sconvex_hull.aeff_V << "\t" << stats->Sconvex_hull.f
						<< "\t" << ((stats->Sconvex_hull.SA / stats->Sconvex_hull.V) / (SA_V_ice_di))
						<< "\t" << stats->Sellipsoid_max.V << "\t" << stats->Sellipsoid_max.SA << "\t" << stats->Sellipsoid_max.SA / stats->Sellipsoid_max.V
						<< "\t" << stats->Sellipsoid_max.aeff_SA << "\t" << stats->Sellipsoid_max.aeff_V << "\t" << stats->Sellipsoid_max.f
						<< "\t" << ((stats->Sellipsoid_max.SA / stats->Sellipsoid_max.V) / (SA_V_ice_di))
						;
				}
				(*(h->file.get())) << std::endl;
				;

				return h; // Pass back the handle
			}

		}
	}
}
