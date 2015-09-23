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
#include <Ryan_Debug/splitSet.h>

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

			struct tsv_ddoutput_iso_small_handle : public Ryan_Debug::registry::IOhandler
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
						"Temperature (K)\tAeff (um)\t"
						"Number of Dipoles\tisIso\tBetas\tThetas\tPhis\t"
						"Qsca_iso\tQbk_iso\tQbk_normalized\tQabs_iso\tQext_iso\tG_iso\t"
						<< std::endl;
					;
				}
				std::shared_ptr<std::ofstream> file;
			};

			shared_ptr<::Ryan_Debug::registry::IOhandler>
				export_tsv_ddori_iso_small_data
				(shared_ptr<::Ryan_Debug::registry::IOhandler> sh, shared_ptr<::Ryan_Debug::registry::IO_options> opts,
				const boost::shared_ptr<const ::rtmath::ddscat::ddOutput > ddOut)
			{
				std::string exporttype = opts->exportType();
				if (exporttype != "isotropic_data_small") RDthrow(Ryan_Debug::error::xUnimplementedFunction());
				std::string filename = opts->filename();
				std::string sDescrip = opts->getVal<std::string>("description","");
				IOhandler::IOtype iotype = opts->iotype();
				using std::shared_ptr;
				std::shared_ptr<tsv_ddoutput_iso_small_handle> h;
				if (!sh)
					h = std::shared_ptr<tsv_ddoutput_iso_small_handle>(new tsv_ddoutput_iso_small_handle(filename.c_str(), iotype));
				else {
					if (std::string(sh->getId()) != std::string(PLUGINID_DDISOSMALL)) RDthrow(Ryan_Debug::error::xDuplicateHook());
					h = std::dynamic_pointer_cast<tsv_ddoutput_iso_small_handle>(sh);
				}

				// Initial file creation handles writing the initial header.
				// So, just write the data.

				boost::shared_ptr<const ddOutput > ddOutRW = (ddOut);
				bool ignoreMissingShape = false;
				if (opts->hasVal("ignoreMissingShape"))
					ignoreMissingShape = opts->getVal<bool>("ignoreMissingShape");
				try {
					//ddOutRW->loadShape(false); // not used right now...
				} catch (Ryan_Debug::error::xMissingHash &e) {
					if (ignoreMissingShape) {
						std::cerr << e.what() << std::endl;
						return h;
					}
					else throw e;
				}

				rotations rots(*(ddOut->parfile));
				weights::ddWeightsDDSCAT wts(rots);
				boost::shared_ptr<weights::DDSCAT3dWeights> ow(new weights::DDSCAT3dWeights(wts));
				weights::ddOutputAvg averager(ow);

				boost::shared_ptr<ddOutput> ddOutWithAvg;
				Eigen::MatrixXf outwts;
				averager.doAvgAll(ddOut.get(), ddOutWithAvg, outwts);


				bool isAvg = true;
				std::string sbeta, stheta, sphi;
				std::set<double> betas, thetas, phis;
				if (opts->hasVal("beta")) {
					isAvg = false;
					sbeta = opts->getVal<std::string>("beta");
				}
				if (opts->hasVal("theta")) stheta = opts->getVal<std::string>("theta");
				if (opts->hasVal("phi")) sphi = opts->getVal<std::string>("phi");
				Ryan_Debug::splitSet::splitSet(sbeta, betas);
				Ryan_Debug::splitSet::splitSet(stheta, thetas);
				Ryan_Debug::splitSet::splitSet(sphi, phis);
				if (!betas.size()) betas.insert(-1);
				if (!thetas.size()) thetas.insert(-1);
				if (!phis.size()) phis.insert(-1);

				if (!sDescrip.size()) sDescrip = ddOut->description;

				for (const auto &beta : betas) {
					for (const auto &theta : thetas) {
						for (const auto &phi : phis) {
							boost::shared_ptr<const ddOriData> fori;
							if (isAvg) {
								fori = ddOriData::generate(*ddOutWithAvg);
							} else {
								size_t row = 0;
								bool hasRow = false;
								hasRow = ddOut->getRow(beta, theta, phi, row);
								if (!hasRow) {
									fori = ddOriData::generate(*ddOutWithAvg, row);
									auto data = fori->selectData();
									std::ostringstream emsg;
									emsg << "Cannot find rotation in ddOri that matches "
										"beta " << beta << " theta " << theta << " phi " << phi
										<< " for hash " << ddOut->shapeHash.lower
										<< ". Closest match is on row " << row
										<< ", which has beta " << data(0,rtmath::ddscat::ddOutput::stat_entries::BETA)
										<< " theta " << data(0,rtmath::ddscat::ddOutput::stat_entries::THETA)
										<< " phi " << data(0,rtmath::ddscat::ddOutput::stat_entries::PHI);
									RDthrow(Ryan_Debug::error::xArrayOutOfBounds())
										<< Ryan_Debug::error::otherErrorText(emsg.str());
								}
								fori = ddOriData::generate(*ddOutWithAvg, row);
							}

							auto data = fori->selectData();
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
								<< ddOut->s.num_dipoles << "\t";
							if (beta >= 0) (*(h->file.get())) 
								<< "0" << "\t" << beta << "\t" << theta << "\t" << phi << "\t";
							else (*(h->file.get())) 
								<< "1" << "\t" << rots.bN() << "\t" << rots.tN() << "\t" << rots.pN() << "\t";
							(*(h->file.get())) << data(ddOutput::stat_entries::QSCAM) << "\t"
								<< data(ddOutput::stat_entries::QBKM) << "\t"
								<< data(ddOutput::stat_entries::QBKM) * 4. * pi << "\t"
								<< data(ddOutput::stat_entries::QABSM) << "\t"
								<< data(ddOutput::stat_entries::QEXTM) << "\t"
								<< data(ddOutput::stat_entries::G1M);
							(*(h->file.get())) << std::endl;
							;
						}
					}
				}
				return h; // Pass back the handle
			}

		}
	}
}
