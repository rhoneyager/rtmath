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
#include "../../rtmath/rtmath/ddscat/rotations.h"
#include "../../rtmath/rtmath/plugin.h"
#include "../../rtmath/rtmath/error/debug.h"
#include "../../rtmath/rtmath/error/error.h"

#include "plugin-hdf5.h"

namespace rtmath {
	namespace plugins {
		namespace hdf5 {

			template<class T, class Container>
			void readAttrSet(std::shared_ptr<H5::Group> grpPar, const char* name,
				boost::shared_ptr<rtmath::ddscat::ddPar> r, void (rtmath::ddscat::ddOutput::* f) (const T&))
			{
				T val;
				readAttr<T, Container>(grpPar, name, val);
				(*r.*f)(val);
			}

			template<class T, class Container>
			void readAttrSet(std::shared_ptr<H5::Group> grpPar, const char* name,
				boost::shared_ptr<rtmath::ddscat::ddPar> r, void (rtmath::ddscat::ddOutput::* f) (T))
			{
				T val;
				readAttr<T, Container>(grpPar, name, val);
				(*r.*f)(val);
			}

			bool read_hdf5_ddOutput(std::shared_ptr<H5::Group> base, 
				boost::shared_ptr<rtmath::ddscat::ddOutput> &r)
			{
				using std::shared_ptr;
				using std::string;
				using namespace H5;
				using namespace ddscat;
				
				readAttr<string, Group>(base, "Description", r->description);
				readAttr<double, Group>(base, "Frequency", r->freq);
				readAttr<double, Group>(base, "aeff", r->aeff);
				readAttr<double, Group>(base, "Temperature", r->temp);

				Eigen::MatrixXf refrs;
				readDatasetEigen<Eigen::MatrixXf, Group>(base, "Refractive_Indices", refrs);
				for (size_t i=0; i< (size_t) refrs.rows(); ++i)
					r->ms.push_back(std::complex<double>( refrs(i,0), refrs(i,1) ));

				// Source file paths

				// Tags

				// DDSCAT run version tag
				readAttr<string, Group>(base, "DDSCAT_Version_Tag", s->ddvertag);

				// Ensemble average results
				{
					//shared_ptr<Group> gEns(new Group(gRun->createGroup("Ensemble")));
					//write_hdf5_ddOutputSingle(gEns, s->avg.get());
					//shared_ptr<Group> gEnso(new Group(gRun->createGroup("Ensemble_Original")));
					//write_hdf5_ddOutputSingle(gEnso, s->avg_original.get());
				}

				/// \todo Change the write operation to include sca headers!

				// Get number of orientations, and allocate a matrix to hold al of the cross-sectional results
				size_t numOris = s->scas.size();
				Eigen::MatrixXf tblOri(numOris, 43);
				// Get number of scattering angles from the avg table
				size_t numScaAngles = s->avg->getScattMatrices().size();
				Eigen::MatrixXf tblFML(numScaAngles * numOris, 15);
				//Eigen::MatrixXf tblSCA(numScaAngles * numOris, 22);





				readAttrSet<size_t, Group>(grpPar, "Version", r,&rtmath::ddscat::ddPar::version);

				int ival;
				readAttr<int, Group>(grpPar, "CMTORQ", ival);
				if (ival) r->doTorques(true);
				else r->doTorques(false);

				readAttrSet<string, Group>(grpPar, "CMDSOL", r,&rtmath::ddscat::ddPar::setSolnMeth);

				readAttrSet<string, Group>(grpPar, "CMDFFT", r,&rtmath::ddscat::ddPar::setFFTsolver);
				readAttrSet<string, Group>(grpPar, "CALPHA", r,&rtmath::ddscat::ddPar::setCalpha);
				readAttrSet<string, Group>(grpPar, "CBINFLAG", r,&rtmath::ddscat::ddPar::setBinning);

				Eigen::Vector3i Imem;
				readAttrEigen<Eigen::Vector3i, Group>(grpPar, "dimension", Imem);
				r->Imem(0,Imem(0));
				r->Imem(1, Imem(1));
				r->Imem(2, Imem(2));
				readAttrSet<string, Group>(grpPar, "CSHAPE", r,&rtmath::ddscat::ddPar::setShape);
				Eigen::Vector3d shparams;
				readAttrEigen<Eigen::Vector3d, Group>(grpPar, "shape_parameters", shparams);
				r->shpar(0, shparams(0));
				r->shpar(1, shparams(1));
				r->shpar(2, shparams(2));

				// Dielectrics go here
				// Write both number of dielectrics (attr) and the list (dataset)
				size_t nDiels;
				readAttr<size_t, Group>(grpPar, "NCOMP", nDiels);
				std::vector<string> diels(nDiels);
				std::vector<const char*> cdiels(diels.size());
				
				readDatasetArray<const char*, Group>(grpPar, "Dielectrics", cdiels.data());
				for (size_t i=0; i<cdiels.size(); ++i)
					diels[i] = string(cdiels[i]);
				r->setDiels(diels);

				Eigen::Matrix<uint64_t, Eigen::Dynamic, Eigen::Dynamic> dielhashes;
				
				readDatasetEigen<Eigen::Matrix<uint64_t, Eigen::Dynamic, Eigen::Dynamic>, Group>
					(grpPar, "Dielectric_Hashes", dielhashes);
				std::vector<HASH_t> vdielhashes((size_t) dielhashes.rows());
				for (size_t i=0; i<(size_t) dielhashes.rows(); ++i)
				{
					vdielhashes[i].lower = dielhashes(i,0);
					vdielhashes[i].upper = dielhashes(i,1);
				}
				r->setDielHashes(vdielhashes);

				
				for (size_t i=0; i<diels.size(); ++i)
					cdiels[i] = diels[i].c_str();
				r->setDiels(diels);

				

				readAttr<int, Group>(grpPar, "NRFLD", ival);
				if (ival) r->doNearField(true);
				else r->doNearField(false);

				Eigen::VectorXd extent(6);
				readAttrEigen<Eigen::VectorXd, Group>(grpPar, "near_extent", extent);
#undef near // No idea where this gets defined...
				r->near(0,extent(0));
				r->near(1,extent(1));
				r->near(2,extent(2));
				r->near(3,extent(3));
				r->near(4,extent(4));
				r->near(5,extent(5));


				readAttrSet<double, Group>(grpPar, "TOL", r,&rtmath::ddscat::ddPar::maxTol);
				readAttrSet<size_t, Group>(grpPar, "MXITER", r,&rtmath::ddscat::ddPar::maxIter);
				readAttrSet<double, Group>(grpPar, "GAMMA", r,&rtmath::ddscat::ddPar::gamma);
				readAttrSet<double, Group>(grpPar, "ETASCA", r,&rtmath::ddscat::ddPar::etasca);
				
				// Wavelengths
				string sWaves, sSpecialized;
				readAttr<string, Group>(grpPar, "Wavelengths_str", sWaves);
				double wMin, wMax, wJunk;
				size_t wN;
				rtmath::config::extractInterval(sWaves, wMin, wMax, wJunk, wN, sSpecialized);
				r->setWavelengths(wMin, wMax, wN, sSpecialized);

				/// \todo Add tab file support for wavelengths
				/*
				std::set<double> wavelengths;
				r->getWavelengths(wavelengths);
				std::vector<double> vwvs(wavelengths.begin(), wavelengths.end());
				addDatasetArray<double, Group>(grpPar, "Wavelengths", vwvs.size(), 1, vwvs.data());
				*/

				readAttrSet<double, Group>(grpPar, "NAMBIENT", r,&rtmath::ddscat::ddPar::nAmbient);

				// Effective radii
				readAttr<string, Group>(grpPar, "Effective_Radii_str", sWaves);
				rtmath::config::extractInterval(sWaves, wMin, wMax, wJunk, wN, sSpecialized);
				r->setAeff(wMin, wMax, wN, sSpecialized);

				/// \todo Add tab file support for effective radii
				/*
				std::set<double> aeffs;
				r->getWavelengths(aeffs);
				std::vector<double> va(aeffs.begin(), aeffs.end());
				addDatasetArray<double, Group>(grpPar, "Effective_Radii", va.size(), 1, va.data());
				*/

				// Incident polarizations (needed for fml conversions!)
				Eigen::VectorXd ipol(6);
				readAttrEigen<Eigen::VectorXd, Group>(grpPar, "PolState", ipol);
				r->PolState(0,ipol(0));
				r->PolState(1,ipol(1));
				r->PolState(2,ipol(2));
				r->PolState(3,ipol(3));
				r->PolState(4,ipol(4));
				r->PolState(5,ipol(5));
				readAttrSet<size_t, Group>(grpPar, "IORTH", r,&rtmath::ddscat::ddPar::OrthPolState);
				
				// IWRPOL is retired. It was dropped in 7.2.
				//addAttr<bool, Group>(grpPar, "IWRPOL", r->writePol());
				readAttr<int, Group>(grpPar, "IWRKSC", ival);
				if (ival) r->writeSca(true);
				else r->writeSca(false);

				// Rotations
				// Beta, Theta, Phi
				string betas, thetas, phis, sjunk;
				readAttr<string, Group>(grpPar, "Betas", betas);
				readAttr<string, Group>(grpPar, "Thetas", thetas);
				readAttr<string, Group>(grpPar, "Phis", phis);
				double bMin, bMax, tMin, tMax, pMin, pMax, junk;
				size_t bN, tN, pN;
				rtmath::config::extractInterval(betas, bMin, bMax, junk, bN, sjunk);
				rtmath::config::extractInterval(thetas, tMin, tMax, junk, tN, sjunk);
				rtmath::config::extractInterval(phis, pMin, pMax, junk, pN, sjunk);

				rotations rot(bMin,bMax,bN,tMin,tMax,tN,pMin,pMax,pN);
				r->setRots(rot);
				

				Eigen::Vector3i iwav;
				readAttrEigen<Eigen::Vector3i, Group>(grpPar, "firstOri", iwav);
				r->firstOri(0,iwav(0));
				r->firstOri(1,iwav(1));
				r->firstOri(2,iwav(2));

				// Get SIJ matrix
				Eigen::Matrix<size_t, Eigen::Dynamic, Eigen::Dynamic> sijs;
				std::set<size_t> ssij;
				readDatasetEigen<Eigen::Matrix<size_t, Eigen::Dynamic, Eigen::Dynamic>, Group>
					(grpPar, "SIJs", sijs);
				for (size_t i=0; i<(size_t)sijs.rows(); ++i)
					ssij.insert(sijs(i,0));
				r->setSIJ(ssij);
				

				readAttrSet<string, Group>(grpPar, "CMDFRM", r,&rtmath::ddscat::ddPar::setCMDFRM);

				//readAttr<size_t, Group>(grpPar, "NPLANES", r->numPlanes());
				Eigen::MatrixXd planes; //(r->numPlanes(), 4);
				readDatasetEigen<Eigen::MatrixXd, Group>(grpPar, "Planes", planes);
				// Write out scattering plane information
				for (size_t i=0; i < (size_t) planes.rows(); ++i)
				{
					double phi = planes(i,0), 
						thetan_min = planes(i,1), 
						thetan_max = planes(i,2), 
						dtheta = planes(i,3);
					r->setPlane(i+1,phi,thetan_min,thetan_max,dtheta);
				}

				//return grpRot;
				return true;
			}
		}
	}

	namespace registry
	{
		using std::shared_ptr;
		using namespace rtmath::plugins::hdf5;
		
		template<>
		shared_ptr<IOhandler>
			read_file_type_multi<rtmath::ddscat::ddPar>
			(shared_ptr<IOhandler> sh, shared_ptr<IO_options> opts,
			std::vector<boost::shared_ptr<rtmath::ddscat::ddPar> > &s)
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

			shared_ptr<Group> grpHashes = openGroup(h->file, "Hashed");
			if (!grpHashes) return h;
			// Iterate over hash entries
			hsize_t nObjs = grpHashes->getNumObjs();
			for (hsize_t i=0; i<nObjs; ++i)
			{
				std::string hashname = grpHashes->getObjnameByIdx(i);
				H5G_obj_t t = grpHashes->getObjTypeByIdx(i);
				if (t != H5G_obj_t::H5G_GROUP) continue;

				shared_ptr<Group> grpHash = openGroup(grpHashes, hashname.c_str());

				shared_ptr<Group> grpShape = openGroup(grpHash, "Shape");
				if (!grpShape) continue;

				boost::shared_ptr<rtmath::ddscat::ddPar> res(new rtmath::ddscat::ddPar);
				if (read_hdf5_ddPar(grpShape, res))
					s.push_back(res);
			}

			/// \todo Implement opts searching features

			return h;
		}
	}
}
