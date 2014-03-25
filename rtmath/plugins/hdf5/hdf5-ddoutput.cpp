/// \brief Provides hdf5 file IO
#define _SCL_SECURE_NO_WARNINGS
#pragma warning( disable : 4251 ) // warning C4251: dll-interface

#include <array>
#include <iostream>
#include <fstream>
#include <string>
#include <memory>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <tuple>

#include <boost/filesystem.hpp>
#include <hdf5.h>
#include <H5Cpp.h>

#include "../../rtmath/rtmath/defs.h"
#include "../../rtmath/rtmath/ddscat/shapefile.h"
#include "../../rtmath/rtmath/ddscat/shapestats.h"
#include "../../rtmath/rtmath/ddscat/ddpar.h"
#include "../../rtmath/rtmath/ddscat/ddOutput.h"
#include "../../rtmath/rtmath/ddscat/ddOutputSingle.h"
#include "../../rtmath/rtmath/ddscat/ddScattMatrix.h"
#include "../../rtmath/rtmath/plugin.h"
#include "../../rtmath/rtmath/error/debug.h"
#include "../../rtmath/rtmath/error/error.h"

#include "plugin-hdf5.h"

#include "cmake-settings.h"

namespace rtmath {
	namespace plugins {
		namespace hdf5 {

			void write_hdf5_ddPar(std::shared_ptr<H5::Group> base, 
				const rtmath::ddscat::ddPar *r);
			bool read_hdf5_ddPar(std::shared_ptr<H5::Group> grpPar, 
				boost::shared_ptr<rtmath::ddscat::ddPar> &r);

			void write_hdf5_ddOutputSingle(
				const rtmath::ddscat::ddOutputSingle *r, size_t index = 0,
				Eigen::MatrixXf *tblOri = nullptr, Eigen::MatrixXf *tblSca = nullptr)
			{
				using std::shared_ptr;
				using namespace H5;
				using namespace ddscat;

				/*
				std::string rotid;
				{
					std::ostringstream o;
					o << r->beta() << "," << r->theta() << "," << r->phi();
					rotid = o.str();
				}
				shared_ptr<Group> grpRot;
				if (!inplace)
					grpRot = shared_ptr<Group> (new Group(base->createGroup(rotid)));
				else grpRot = base;

				addAttr<double, Group>(grpRot, "Beta", r->beta());
				addAttr<double, Group>(grpRot, "Theta", r->theta());
				addAttr<double, Group>(grpRot, "Phi", r->phi());

				addAttr<double, Group>(grpRot, "Wavelength", r->wave());
				addAttr<double, Group>(grpRot, "Frequency", r->freq());
				addAttr<double, Group>(grpRot, "aeff", r->aeff());
				addAttr<double, Group>(grpRot, "Dipole_Spacing", r->dipoleSpacing());
				addAttr<size_t, Group>(grpRot, "Num_Dipoles", r->numDipoles());

				addAttr<size_t, Group>(grpRot, "version", r->version());

				/// \todo add complex double support
				//addAttr<std::complex<double>, Group>(grpRot, "Refractive_Index", r->getM());

				// Header Entries
				//shared_ptr<Group> headers(new Group(grpRot->createGroup("Headers")));
				{
					// Create a dataset from strings

				}

				// Stat table
				*/
				// Scattering matrices
				//shared_ptr<Group> g(new Group(grpRot->createGroup("Scattering_Matrices")));
				ddOutputSingle::scattMatricesContainer c;
				r->getScattMatrices(c);
				size_t i=0;
				for (const auto &mat : c)
				{
					if (tblOri)
					{
						ddscat::ddOutputSingle::statTableType stats;
						r->getStatTable(stats);
						// The column ordering matches the stat_entries ordering, making reads easier.
						double blockds[43] = {
							r->freq(), r->aeff(),
							r->beta(), r->theta(), r->phi(),
							stats[stat_entries::QEXT1],
							stats[stat_entries::QABS1],
							stats[stat_entries::QSCA1],
							stats[stat_entries::G11],
							stats[stat_entries::G21],
							stats[stat_entries::QBK1],
							stats[stat_entries::QPHA1],
							stats[stat_entries::QEXT2],
							stats[stat_entries::QABS2],
							stats[stat_entries::QSCA2],
							stats[stat_entries::G12],
							stats[stat_entries::G22],
							stats[stat_entries::QBK2],
							stats[stat_entries::QPHA2],
							stats[stat_entries::QEXTM],
							stats[stat_entries::QABSM],
							stats[stat_entries::QSCAM],
							stats[stat_entries::G1M],
							stats[stat_entries::G2M],
							stats[stat_entries::QBKM],
							stats[stat_entries::QPHAM],
							stats[stat_entries::QPOL],
							stats[stat_entries::DQPHA],
							stats[stat_entries::QSCAG11],
							stats[stat_entries::QSCAG21],
							stats[stat_entries::QSCAG31],
							stats[stat_entries::ITER1],
							stats[stat_entries::MXITER1],
							stats[stat_entries::NSCA1],
							stats[stat_entries::QSCAG12],
							stats[stat_entries::QSCAG22],
							stats[stat_entries::QSCAG32],
							stats[stat_entries::ITER2],
							stats[stat_entries::MXITER2],
							stats[stat_entries::NSCA2],
							stats[stat_entries::QSCAG1M],
							stats[stat_entries::QSCAG2M],
							stats[stat_entries::QSCAG3M]
						};
						tblOri->block<1,43>(index,0) = 
							Eigen::Map<Eigen::VectorXd>(blockds,43).cast<float>();

					}
					if (tblSca)
					{
						// Only the FML files are stored. The rest can be regenerated from these.
						if (mat->id() == scattMatrixType::F)
						{
							const ddScattMatrixF *vv = dynamic_cast<const ddScattMatrixF*>(mat.get());
							auto F = vv->getF();
							//addDatasetEigenComplexMethodA<ddScattMatrix::FType, Group>(gv, "F", vv->getF());
							//addDatasetEigenComplexMethodA<ddScattMatrix::FType, Group>(gv, "S", vv->getS());
							double blockd[15] = {
								r->freq(), r->aeff(),
								r->beta(), r->theta(), r->phi(),
								vv->theta(), vv->phi(),
								F(0,0).real(), F(0,0).imag(),
								F(0,1).real(), F(0,1).imag(),
								F(1,0).real(), F(1,0).imag(),
								F(1,1).real(), F(1,1).imag()
							};
							tblSca->block<1,15>((c.size() * index)+i,0) = 
								Eigen::Map<Eigen::VectorXd>(blockd,15).cast<float>();
						}
					}
					++i;
				}

				//return grpRot;
			}

			/// \param base is the base (./Runs) to write the subgroups to.
			std::shared_ptr<H5::Group> write_hdf5_ddOutput(std::shared_ptr<H5::Group> base, 
				std::shared_ptr<rtmath::registry::IO_options> opts, 
				const rtmath::ddscat::ddOutput *s)
			{
				using std::string;
				using std::shared_ptr;
				using namespace H5;

				// Pick a unique name matching frequency, aeff and temperature (refractive index)
				/// \todo Pick a better naming function for hdf5-internal runs
				shared_ptr<Group> gRun(new Group(base->createGroup(s->genNameSmall())));


				addAttr<string, Group>(gRun, "Description", s->description);
				addAttr<double, Group>(gRun, "Frequency", s->freq);
				addAttr<double, Group>(gRun, "aeff", s->aeff);
				addAttr<double, Group>(gRun, "Temperature", s->temp);

				// Refractive indices table
				Eigen::MatrixXf refrs((int) s->ms.size(), 2);
				for (size_t i=0; i < s->ms.size(); ++i)
				{
					refrs(i,0) = (float) s->ms[i].real();
					refrs(i,1) = (float) s->ms[i].imag();
				}
				addDatasetEigen(gRun, "Refractive_Indices", refrs);

				// Source file paths

				// Tags

				// DDSCAT run verion tag
				addAttr<string, Group>(gRun, "DDSCAT_Version_Tag", s->ddvertag);

				// Ensemble average results
				{
					//shared_ptr<Group> gEns(new Group(gRun->createGroup("Ensemble")));
					//write_hdf5_ddOutputSingle(gEns, s->avg.get());
					//shared_ptr<Group> gEnso(new Group(gRun->createGroup("Ensemble_Original")));
					//write_hdf5_ddOutputSingle(gEnso, s->avg_original.get());
				}


				// Get number of orientations, and allocate a matrix to hold al of the cross-sectional results
				size_t numOris = s->scas.size();
				Eigen::MatrixXf tblOri(numOris, 43);
				// Get number of scattering angles from the avg table
				size_t numScaAngles = s->avg->getScattMatrices().size();
				Eigen::MatrixXf tblFML(numScaAngles * numOris, 15);
				//Eigen::MatrixXf tblSCA(numScaAngles * numOris, 22);



				auto writeGroup = [&](const char* grpname, 
					const std::set<boost::shared_ptr<rtmath::ddscat::ddOutputSingle>, 
					sharedComparator<boost::shared_ptr<const ddscat::ddOutputSingle> > > &o,
					Eigen::MatrixXf *oritable, Eigen::MatrixXf *stable)
				{
					//shared_ptr<Group> g(new Group(gRun->createGroup(grpname)));
					size_t i = 0; // Index for table writing
					for (const auto& f : o)
					{
						write_hdf5_ddOutputSingle(f.get(), i, oritable, stable);
						++i;
					}
				};
				//writeGroup("SCA_original", s->scas_original, nullptr, nullptr);
				writeGroup("FML", s->fmls, nullptr, &tblFML);
				writeGroup("SCA", s->scas, &tblOri, nullptr);

				// Enable compression
				hsize_t chunk_dimsOri[2] = { (hsize_t) numOris, 1 };
				hsize_t chunk_dimsFML[2] = { (hsize_t) numScaAngles, 15 };
				auto plistOri = std::shared_ptr<DSetCreatPropList>(new DSetCreatPropList);
				plistOri->setChunk(2, chunk_dimsOri);
				auto plistFML = std::shared_ptr<DSetCreatPropList>(new DSetCreatPropList);
				plistFML->setChunk(2, chunk_dimsFML);
				/* // SZIP compression disabled for now
#if COMPRESS_SZIP
				unsigned szip_options_mask = H5_SZIP_NN_OPTION_MASK;
				unsigned szip_pixels_per_block = 16;
				plistOri->setSzip(szip_options_mask, szip_pixels_per_block);
				plistFML->setSzip(szip_options_mask, szip_pixels_per_block);
				*/
#if COMPRESS_ZLIB
				plistOri->setDeflate(6);
				plistFML->setDeflate(6);
#endif

				bool writeFML = opts->getVal("writeFMLs", true);

				addDatasetEigen(gRun, "Cross_Sections", tblOri, plistOri);
				if (writeFML)
					addDatasetEigen(gRun, "FML_Data", tblFML, plistFML);
				//addDatasetEigen(gRun, "Scattering_Data", tblSCA);

				// Add a special ddOutputSingle entry for the avg file

				
				// Shapefile link
				//addAttr<string,Group>(gRun, "Shapehash_full", s->shapeHash.string());
				addAttr<uint64_t,Group>(gRun, "Shapehash_lower", s->shapeHash.lower);
				addAttr<uint64_t,Group>(gRun, "Shapehash_upper", s->shapeHash.upper);

				// If a shapefile is written to this file, make a symlink
				std::string pShape;
				{
					std::ostringstream o;
					o << "/Hashed/" << s->shapeHash.string() << "/Shape";
					pShape = o.str();
				}
				gRun->link(H5L_TYPE_SOFT, pShape, "Shape");


				// If stats are written to this file, make a symlink
				std::string pStats;
				{
					std::ostringstream o;
					o << "/Hashed/" << s->shapeHash.string() << "/Stats";
					pShape = o.str();
				}
				gRun->link(H5L_TYPE_SOFT, pShape, "Stats");
				// Insert stats information given known dipole spacing


				// ddscat.par file
				write_hdf5_ddPar(gRun, s->parfile.get());

				// Testing ddscat.par read...
				boost::shared_ptr<rtmath::ddscat::ddPar> stest(new rtmath::ddscat::ddPar);
				shared_ptr<Group> grpPar = openGroup(gRun, "par");
				read_hdf5_ddPar(grpPar, stest);
				stest->write(std::cout);


				return gRun;
			}

			/// \todo Add in writing handlers for ddoutputsingle and ddscatmatrix

		}
	}

	namespace registry
	{
		using std::shared_ptr;
		using namespace rtmath::plugins::hdf5;
		
		template<>
		shared_ptr<IOhandler> 
			write_file_type_multi<rtmath::ddscat::ddOutput>
			(shared_ptr<IOhandler> sh, shared_ptr<IO_options> opts, 
			const rtmath::ddscat::ddOutput *s)
		{
			std::string filename = opts->filename();
			IOhandler::IOtype iotype = opts->iotype();
			std::string key = opts->getVal<std::string>("key", "");
			using std::shared_ptr;
			using namespace H5;
			Exception::dontPrint();
			std::shared_ptr<hdf5_handle> h;
			if (!sh)
			{
				// Access the hdf5 file
				h = std::shared_ptr<hdf5_handle>(new hdf5_handle(filename.c_str(), iotype));
			} else {
				if (sh->getId() != PLUGINID) RTthrow debug::xDuplicateHook("Bad passed plugin");
				h = std::dynamic_pointer_cast<hdf5_handle>(sh);
			}

			// Check for the existence of the appropriate:
			// Group "Hashed"
			shared_ptr<Group> grpHashes = openOrCreateGroup(h->file, "Hashed");
			// Group "Hashed"/shp->hash
			shared_ptr<Group> grpHash = openOrCreateGroup(grpHashes, s->shape->hash().string().c_str());
			// Group "Hashed"/shp->hash/"Shape". If it exists, overwrite it. There should be no hard links here.
			/// \note The unlink operation does not really free the space..... Should warn the user.
			shared_ptr<Group> gRuns = openOrCreateGroup(grpHash, "Runs");

			//std::string aeffid("aeff-");
			//aeffid.append(boost::lexical_cast<std::string>( (float) s->aeff ));
			//shared_ptr<Group> gRunsAeff = openOrCreateGroup(gRuns, aeffid.c_str());

			//if (groupExists(grpRuns, "Stats")) return h; //grpHash->unlink("Stats");

			/// \todo Modify to also support external symlinks
			shared_ptr<Group> base = write_hdf5_ddOutput(gRuns, opts, s);

			bool writeShape = opts->getVal("writeShapes", true);
			if (writeShape)
				write_file_type_multi<rtmath::ddscat::shapefile::shapefile>
					(h, opts, s->shape.get());

			//shared_ptr<Group> newstatsbase = write_hdf5_statsrawdata(grpHash, s);
			//shared_ptr<Group> newshapebase = write_hdf5_shaperawdata(grpHash, s->_shp.get());

			return h; // Pass back the handle
		}

		template<>
		shared_ptr<IOhandler>
			read_file_type_multi<rtmath::ddscat::ddOutput>
			(shared_ptr<IOhandler> sh, shared_ptr<IO_options> opts,
			std::vector<boost::shared_ptr<rtmath::ddscat::ddOutput> > &s)
		{
			std::string filename = opts->filename();
			IOhandler::IOtype iotype = opts->getVal<IOhandler::IOtype>("iotype", IOhandler::IOtype::READONLY);
			//IOhandler::IOtype iotype = opts->iotype();
			std::string key = opts->getVal<std::string>("key", "");
			using std::shared_ptr;
			using namespace H5;
			Exception::dontPrint();
			std::shared_ptr<hdf5_handle> h;
			if (!sh)
			{
				// Access the hdf5 file
				h = std::shared_ptr<hdf5_handle>(new hdf5_handle(filename.c_str(), iotype));
			} else {
				if (sh->getId() != PLUGINID) RTthrow debug::xDuplicateHook("Bad passed plugin");
				h = std::dynamic_pointer_cast<hdf5_handle>(sh);
			}

			/* The reader function will attempt to load ddOutput files that match the opts 
			 * search descriptions.
			 * opts can search:
			 * - by hash
			 * - by effective radius
			 * - random particle
			 * - by temperature
			 * - by frequency
			 **/
			/// \todo Implement opts searching features


			return h;
		}
	}
}
