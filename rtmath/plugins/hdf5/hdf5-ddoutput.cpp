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
#include "../../rtmath/rtmath/ddscat/ddOutput.h"
#include "../../rtmath/rtmath/ddscat/ddOutputSingle.h"
#include "../../rtmath/rtmath/ddscat/ddScattMatrix.h"
#include "../../rtmath/rtmath/plugin.h"
#include "../../rtmath/rtmath/error/debug.h"
#include "../../rtmath/rtmath/error/error.h"

#include "plugin-hdf5.h"

namespace rtmath {
	namespace plugins {
		namespace hdf5 {

			/// \deprecated Writing now goes through the ddOutputSingle write function
			std::shared_ptr<H5::Group> write_hdf5_ddScattMatrix(std::shared_ptr<H5::Group> base, 
				const rtmath::ddscat::ddScattMatrix *v)
			{
				using std::shared_ptr;
				using namespace H5;
				using namespace rtmath::ddscat;

				std::string rotid;
				{
					std::ostringstream o;
					o << v->theta() << "," << v->phi() << "," << v->thetan() << "," << v->phin();
					rotid = o.str();
				}

				shared_ptr<Group> gv(new Group(base->createGroup(rotid)));

				addAttr<double, Group>(gv, "Pol", v->pol());
				addAttr<double, Group>(gv, "Pol_Linear", v->polLin());
				addAttr<double, Group>(gv, "Frequency", v->freq());
				addAttr<double, Group>(gv, "theta", v->theta());
				addAttr<double, Group>(gv, "thetan", v->thetan());
				addAttr<double, Group>(gv, "phi", v->phi());
				addAttr<double, Group>(gv, "phin", v->phin());

				addDatasetEigen<ddScattMatrix::PnnType, Group>(gv, "Mueller", v->mueller());

				if (v->id() == scattMatrixType::F)
				{
					const ddScattMatrixF *vv = dynamic_cast<const ddScattMatrixF*>(v);
					/// \todo Add complex double support
					addDatasetEigenComplexMethodA<ddScattMatrix::FType, Group>(gv, "F", vv->getF());
					addDatasetEigenComplexMethodA<ddScattMatrix::FType, Group>(gv, "S", vv->getS());
				}

				return gv;
			}

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
						double blockds[20] = {
							r->freq(), r->aeff(),
							r->beta(), r->theta(), r->phi(),
							stats[stat_entries::QSCAM],
							stats[stat_entries::QSCA1],
							stats[stat_entries::QSCA2],
							stats[stat_entries::QBKM],
							stats[stat_entries::QBK1],
							stats[stat_entries::QBK2],
							stats[stat_entries::QABSM],
							stats[stat_entries::QABS1],
							stats[stat_entries::QABS2],
							stats[stat_entries::QEXTM],
							stats[stat_entries::QEXT1],
							stats[stat_entries::QEXT2],
							stats[stat_entries::G1M],
							stats[stat_entries::G11],
							stats[stat_entries::G12]
						};
						tblOri->block<1,20>(index,0) = 
							Eigen::Map<Eigen::VectorXd>(blockds,20).cast<float>();

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
				const rtmath::ddscat::ddOutput *s)
			{
				using std::string;
				using std::shared_ptr;
				using namespace H5;

				// Pick a unique name matching frequency, aeff and temperature (refractive index)
				shared_ptr<Group> gRun(new Group(base->createGroup(s->genName())));


				addAttr<string, Group>(gRun, "Description", s->description);
				addAttr<double, Group>(gRun, "Frequency", s->freq);
				addAttr<double, Group>(gRun, "aeff", s->aeff);
				addAttr<double, Group>(gRun, "Temperature", s->temp);

				// Refractive indices table

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
				Eigen::MatrixXf tblOri(numOris, 20);
				// Get number of scattering angles from the avg table
				size_t numScaAngles = s->avg->getScattMatrices().size();
				Eigen::MatrixXf tblFML(numScaAngles * numOris, 15);
				//Eigen::MatrixXf tblSCA(numScaAngles * numOris, 22);



				auto writeGroup = [&](const char* grpname, 
					const std::set<boost::shared_ptr<rtmath::ddscat::ddOutputSingle> > &o,
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

				addDatasetEigen(gRun, "Cross_Sections", tblOri);
				addDatasetEigen(gRun, "FML_Data", tblFML);
				//addDatasetEigen(gRun, "Scattering_Data", tblSCA);

				// Add a special ddOutputSingle entry for the avg file

				// Stats link

				// Insert stats information given known dipole spacing

				// Shapefile link

				// ddscat.par file



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
			shared_ptr<Group> base = write_hdf5_ddOutput(gRuns, s);
			//shared_ptr<Group> newstatsbase = write_hdf5_statsrawdata(grpHash, s);
			//shared_ptr<Group> newshapebase = write_hdf5_shaperawdata(grpHash, s->_shp.get());

			return h; // Pass back the handle
		}

	}
}
