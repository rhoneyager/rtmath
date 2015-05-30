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

			struct tsv_ddoutput_ori_handle : public rtmath::registry::IOhandler
			{
				tsv_ddoutput_ori_handle(const char* filename, IOtype t) : IOhandler(PLUGINID_DDORI) { open(filename, t); }
				virtual ~tsv_ddoutput_ori_handle() {}
				void open(const char* filename, IOtype t)
				{
					using namespace boost::filesystem;
					switch (t)
					{
					case IOtype::EXCLUSIVE:
					case IOtype::DEBUG:
					case IOtype::READONLY:
						RDthrow(debug::xOtherError());
						break;
					case IOtype::CREATE:
						if (exists(path(filename))) RDthrow(debug::xFileExists());
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
						"Temperature (K)\tAeff (um)\tV_Ice (um^3)\tV_Ice (dipoles^3)\t"
						"SA_Ice (um^2)\tSA_Ice (dipoles^2)\tSA_V_Ice (um^-1)\tSA_V_Ice (dipoles^-1)\t"
						"Number of Dipoles\t"
						"Qsca_m_iso\tQbk_m_iso\ttQbk_m_iso_normalized\tQabs_m_iso\tQext_m_iso\tG1_m_iso\t"
						"Beta\tTheta\tPhi\tWeight\t"
						"Qsca_m_ori\tQbk_m_ori\ttQbk_m_ori_normalized\tQabs_m_ori\tQext_m_ori\tG1_m_ori"

						<< std::endl;
					;
				}
				std::shared_ptr<std::ofstream> file;
			};

			shared_ptr<::rtmath::registry::IOhandler>
				export_tsv_ddori_ori_data
				(shared_ptr<::rtmath::registry::IOhandler> sh, shared_ptr<::rtmath::registry::IO_options> opts,
				const boost::shared_ptr<const ::rtmath::ddscat::ddOutput > ddOut)
			{
				std::string exporttype = opts->exportType();
				if (exporttype != "orientation_data") RDthrow(debug::xUnimplementedFunction());
				std::string filename = opts->filename();
				std::string sDescrip = opts->getVal<std::string>("description", "");
				bool isoWts = opts->getVal<bool>("isoWts", false);
				IOhandler::IOtype iotype = opts->iotype();
				using std::shared_ptr;
				std::shared_ptr<tsv_ddoutput_ori_handle> h;
				if (!sh)
					h = std::shared_ptr<tsv_ddoutput_ori_handle>(new tsv_ddoutput_ori_handle(filename.c_str(), iotype));
				else {
					if (sh->getId() != PLUGINID_DDORI) RDthrow(debug::xDuplicateHook());
					h = std::dynamic_pointer_cast<tsv_ddoutput_ori_handle>(sh);
				}

				boost::shared_ptr<const ddOutput> ddOutRW = (ddOut);
				ddOutRW->loadShape(true);

				rotations rots(*(ddOut->parfile));
				weights::ddWeightsDDSCAT wts(rots);
				boost::shared_ptr<weights::DDSCAT3dWeights> ow(new weights::DDSCAT3dWeights(wts));
				weights::ddOutputAvg averager(ow);

				boost::shared_ptr<ddOutput> ddOutWithAvg;
				Eigen::MatrixXf outwts;
				averager.doAvgAll(ddOut.get(), ddOutWithAvg, outwts);


				boost::shared_ptr<const ddOriData> fiso = ddOriData::generate(*ddOutWithAvg);

				auto data = fiso->selectData();

				// Also pull in stats (should be loaded before write)
				auto stats = ddOut->stats;


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

				for (size_t i = 0; i < ddOut->numOriData; ++i)
				{
					boost::shared_ptr<const ddOriData> fori = ddOriData::generate(*ddOutWithAvg);

					auto dataori = fori->selectData();
					double beta = dataori(ddOutput::stat_entries::BETA);
					double theta = dataori(ddOutput::stat_entries::THETA);
					double phi = dataori(ddOutput::stat_entries::PHI);
					double wt = ow->getWeight(beta, theta, phi);
					/// \todo Add cross-sections and perimeters to orientations after they are implemented

					(*(h->file.get())) << ddOut->shapeHash.lower << "\t" << ddOut->description << "\t"
						<< ddOut->ddvertag << "\t"
						<< ddOut->freq << "\t" << lambda << "\t" << sizep << "\t"
						<< data(ddOutput::stat_entries::D) << "\t" << ddOut->temp << "\t"
						<< ddOut->aeff << "\t" << Vice_um << "\t" << Vice_di << "\t" << SAice_um << "\t" << SAice_di << "\t"
						<< SA_V_ice_um << "\t" << SA_V_ice_di << "\t"
						<< ddOut->s.num_dipoles << "\t"
						<< data(ddOutput::stat_entries::QSCAM) << "\t"
						<< data(ddOutput::stat_entries::QBKM) << "\t"
						<< data(ddOutput::stat_entries::QBKM) * 4. * pi << "\t"
						<< data(ddOutput::stat_entries::QABSM) << "\t"
						<< data(ddOutput::stat_entries::QEXTM) << "\t"
						<< data(ddOutput::stat_entries::G1M) << "\t"

						<< beta << "\t" << theta << "\t" << phi << "\t" << wt << "\t"

						<< dataori(ddOutput::stat_entries::QSCAM) << "\t"
						<< dataori(ddOutput::stat_entries::QBKM) << "\t"
						<< dataori(ddOutput::stat_entries::QBKM) * 4. * pi << "\t"
						<< dataori(ddOutput::stat_entries::QABSM) << "\t"
						<< dataori(ddOutput::stat_entries::QEXTM) << "\t"
						<< dataori(ddOutput::stat_entries::G1M) // no tab after this (last entry)
						;

					/*
					ddOutputSingle::scattMatricesContainer &fs = (*fml)->getScattMatrices();
					boost::shared_ptr<const ddScattMatrixF> t0p0, t180p0, t0p90, t180p90;
					for (const auto &s : fs)
					{
					if (s->id() != scattMatrixType::F) continue; // Should never happen
					if (hasSameRot(s->theta(), 0, 0) && hasSameRot(s->phi(), 0, 0) )
					t0p0 = boost::dynamic_pointer_cast<const ddScattMatrixF>(s);
					if (hasSameRot(s->theta(), 180, 180) && hasSameRot(s->phi(), 0, 0))
					t180p0 = boost::dynamic_pointer_cast<const ddScattMatrixF>(s);
					if (hasSameRot(s->theta(), 0, 0) && hasSameRot(s->phi(), 90, 90))
					t0p90 = boost::dynamic_pointer_cast<const ddScattMatrixF>(s);
					if (hasSameRot(s->theta(), 180, 180) && hasSameRot(s->phi(), 90, 90))
					t180p90 = boost::dynamic_pointer_cast<const ddScattMatrixF>(s);
					if (t0p0 && t180p0 && t0p90 && t180p90) break;
					}

					auto writeFType = [](std::ostream &out, const ddScattMatrix::FType &s)
					{
					for (size_t j = 0; j < 2; j++)
					for (size_t i = 0; i < 2; i++)
					{
					// Note the reversed coordinates. This matches ddscat.
					out << "\t" << s(i, j).real();
					out << "\t" << s(i, j).imag();
					}
					};

					writeFType((*(h->file.get())), t0p0->getF());
					writeFType((*(h->file.get())), t0p90->getF());
					writeFType((*(h->file.get())), t180p0->getF());
					writeFType((*(h->file.get())), t180p90->getF());

					writeFType((*(h->file.get())), t0p0->getS());
					writeFType((*(h->file.get())), t0p90->getS());
					writeFType((*(h->file.get())), t180p0->getS());
					writeFType((*(h->file.get())), t180p90->getS());

					auto writePType = [](std::ostream &out, const ddScattMatrix::PnnType &p, size_t i, size_t j)
					{
					out << "\t" << p(i, j);
					};

					auto writePTypeStd = [&writePType](std::ostream &out, const ddScattMatrix::PnnType &p)
					{
					// Write P11, P12, P21, P22, P31, P41
					writePType(out, p, 0, 0);
					writePType(out, p, 0, 1);
					writePType(out, p, 1, 0);
					writePType(out, p, 1, 1);
					writePType(out, p, 2, 0);
					writePType(out, p, 3, 0);
					};

					writePTypeStd((*(h->file.get())), t0p0->mueller());
					writePTypeStd((*(h->file.get())), t0p90->mueller());
					writePTypeStd((*(h->file.get())), t180p0->mueller());
					writePTypeStd((*(h->file.get())), t180p90->mueller());

					*/

					(*(h->file.get())) << std::endl;
				}

				return h; // Pass back the handle
			}

		}
	}
}

