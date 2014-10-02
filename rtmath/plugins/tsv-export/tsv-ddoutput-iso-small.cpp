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
#include "../../rtmath/rtmath/hash.h"
#include "../../rtmath/rtmath/splitSet.h"
#include "../../rtmath/rtmath/ddscat/ddOriData.h"
#include "../../rtmath/rtmath/ddscat/ddOutput.h"
#include "../../rtmath/rtmath/ddscat/ddUtil.h"
#include "../../rtmath/rtmath/ddscat/ddRunSet.h"
#include "../../rtmath/rtmath/ddscat/ddpar.h"
#include "../../rtmath/rtmath/ddscat/rotations.h"
#include "../../rtmath/rtmath/ddscat/ddweights.h"
#include "../../rtmath/rtmath/units.h"
#include "../../rtmath/rtmath/plugin.h"
#include "../../rtmath/rtmath/error/debug.h"
#include "../../rtmath/rtmath/error/error.h"

#include "plugin-tsv.h"


namespace rtmath {
	namespace plugins {
		namespace tsv {
			using std::shared_ptr;
			using namespace rtmath::ddscat;
			using rtmath::ddscat::ddOutput;
			using rtmath::ddscat::rotations;
			using namespace rtmath::registry;

			struct tsv_ddoutput_iso_small_handle : public rtmath::registry::IOhandler
			{
				tsv_ddoutput_iso_small_handle(const char* filename, IOtype t) : IOhandler(PLUGINID_DDISOSMALL) { open(filename, t); }
				virtual ~tsv_ddoutput_iso_small_handle() {}
				void open(const char* filename, IOtype t)
				{
					using namespace boost::filesystem;
					switch (t)
					{
					case IOtype::EXCLUSIVE:
					case IOtype::DEBUG:
					case IOtype::READONLY:
						RTthrow debug::xOtherError();
						break;
					case IOtype::CREATE:
						if (exists(path(filename))) RTthrow("File already exists");
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
						"Temperature (K)\tAeff (um)\t"
						"Number of Dipoles\tBetas\tThetas\tPhis\t"
						"Qsca_iso\tQbk_iso\tQbk_normalized\tQabs_iso\tQext_iso\tG_iso\t"
						<< std::endl;
					;
				}
				std::shared_ptr<std::ofstream> file;
			};

			shared_ptr<::rtmath::registry::IOhandler>
				export_tsv_ddori_iso_small_data
				(shared_ptr<::rtmath::registry::IOhandler> sh, shared_ptr<::rtmath::registry::IO_options> opts,
				const boost::shared_ptr<const ::rtmath::ddscat::ddOutput > ddOut)
			{
				std::string exporttype = opts->exportType();
				if (exporttype != "isotropic_data_small") RTthrow debug::xUnimplementedFunction();
				std::string filename = opts->filename();
				std::string sDescrip = opts->getVal<std::string>("description","");
				IOhandler::IOtype iotype = opts->iotype();
				using std::shared_ptr;
				std::shared_ptr<tsv_ddoutput_iso_small_handle> h;
				if (!sh)
					h = std::shared_ptr<tsv_ddoutput_iso_small_handle>(new tsv_ddoutput_iso_small_handle(filename.c_str(), iotype));
				else {
					if (sh->getId() != PLUGINID_DDISOSMALL) RTthrow debug::xDuplicateHook("Bad passed plugin");
					h = std::dynamic_pointer_cast<tsv_ddoutput_iso_small_handle>(sh);
				}

				// Initial file creation handles writing the initial header.
				// So, just write the data.

				boost::shared_ptr<const ddOutput > ddOutRW = (ddOut);
				ddOutRW->loadShape(false);

				rotations rots(*(ddOut->parfile));
				weights::ddWeightsDDSCAT wts(rots);
				boost::shared_ptr<weights::DDSCAT3dWeights> ow(new weights::DDSCAT3dWeights(wts));
				weights::ddOutputAvg averager(ow);

				boost::shared_ptr<ddOutput> ddOutWithAvg;
				Eigen::MatrixXf outwts;
				averager.doAvgAll(ddOut.get(), ddOutWithAvg, outwts);


				boost::shared_ptr<const ddOriData> fori = ddOriData::generate(*ddOutWithAvg);

				auto data = fori->selectData();

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
					<< ddOut->aeff << "\t" 
					<< ddOut->s.num_dipoles << "\t"
					<< rots.bN() << "\t" << rots.tN() << "\t" << rots.pN() << "\t"
					<< data(ddOutput::stat_entries::QSCAM) << "\t"
					<< data(ddOutput::stat_entries::QBKM) << "\t"
					<< data(ddOutput::stat_entries::QBKM) * 4. * pi << "\t"
					<< data(ddOutput::stat_entries::QABSM) << "\t"
					<< data(ddOutput::stat_entries::QEXTM) << "\t"
					<< data(ddOutput::stat_entries::G1M);
				(*(h->file.get())) << std::endl;
				;

				return h; // Pass back the handle
			}

		}
	}
}
