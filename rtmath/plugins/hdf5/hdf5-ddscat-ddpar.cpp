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

#include "../../rtmath/rtmath/defs.h"
//#include "../../rtmath/rtmath/ddscat/shapefile.h"
//#include "../../rtmath/rtmath/ddscat/shapestats.h"
#include "../../rtmath/rtmath/ddscat/ddpar.h"
#include "../../rtmath/rtmath/ddscat/rotations.h"
#include "../../rtmath/rtmath/plugin.h"
#include "../../rtmath/rtmath/error/debug.h"
#include "../../rtmath/rtmath/error/error.h"

#include "../../related/rtmath_hdf5_cpp/export-hdf5.h"
#include "plugin-hdf5.h"

#include <hdf5.h>
#include <H5Cpp.h>
namespace rtmath {
	namespace plugins {
		namespace hdf5 {

			void write_hdf5_ddPar(std::shared_ptr<H5::Group> grpPar,
				const boost::shared_ptr<const rtmath::ddscat::ddPar > r)
			{
				using std::shared_ptr;
				using std::string;
				using namespace H5;
				using namespace ddscat;

				addAttr<size_t, Group>(grpPar, "Version", r->version());

				addAttr<int, Group>(grpPar, "CMTORQ", (int) r->doTorques());
				addAttr<string, Group>(grpPar, "CMDSOL", r->getSolnMeth());
				addAttr<string, Group>(grpPar, "CMDFFT", r->getFFTsolver());
				addAttr<string, Group>(grpPar, "CALPHA", r->getCalpha());
				addAttr<string, Group>(grpPar, "CBINFLAG", r->getBinning());
				Eigen::Vector3i Imem;
				Imem(0) = (int) r->Imem(0);
				Imem(1) = (int) r->Imem(1);
				Imem(2) = (int) r->Imem(2);
				addAttrEigen<Eigen::Vector3i, Group>(grpPar, "dimension", Imem);
				addAttr<string, Group>(grpPar, "CSHAPE", r->getShape());
				Eigen::Vector3d shparams;
				shparams(0) = r->shpar(0);
				shparams(1) = r->shpar(1);
				shparams(2) = r->shpar(2);
				addAttrEigen<Eigen::Vector3d, Group>(grpPar, "shape_parameters", shparams);

				// Dielectrics go here
				// Write both number of dielectrics (attr) and the list (dataset)
				std::vector<string> diels;
				r->getDiels(diels);
				std::vector<const char*> cdiels(diels.size());
				for (size_t i=0; i<diels.size(); ++i)
					cdiels[i] = diels[i].c_str();
				addAttr<size_t, Group>(grpPar, "NCOMP", diels.size());
				addDatasetArray<const char*, Group>(grpPar, "Dielectrics", cdiels.size(), 1, cdiels.data());
				std::vector<HASH_t> dielhashes;
				r->getDielHashes(dielhashes);
				std::vector<uint64_t> cdielhashes(dielhashes.size()*2);
				for (size_t i=0; i<dielhashes.size(); ++i)
				{
					cdielhashes[2*i] = dielhashes[i].lower;
					cdielhashes[2*i+1] = dielhashes[i].upper;
				}
				addDatasetArray<uint64_t, Group>(grpPar, "Dielectric_Hashes", cdielhashes.size() / 2, 2, cdielhashes.data());
				/*
				hsize_t dieldims[1] = {(hsize_t) diels.size()};
				DataSpace dfspace(1, dieldims);
				std::shared_ptr<H5::AtomType> dt(new H5::StrType(0, H5T_VARIABLE));
				std::shared_ptr<DataSet> dielsdata(new DataSet(grpPar->createDataSet("Dielectrics", *(dt.get()), dfspace)));
				dielsdata->write(cdiels.data(), *(dt.get()));
				*/

				addAttr<int, Group>(grpPar, "NRFLD", (int) r->doNearField());
				Eigen::VectorXd extent(6);
#undef near // No idea where this gets defined...
				extent(0) = r->near(0);
				extent(1) = r->near(1);
				extent(2) = r->near(2);
				extent(3) = r->near(3);
				extent(4) = r->near(4);
				extent(5) = r->near(5);
				addAttrEigen<Eigen::VectorXd, Group>(grpPar, "near_extent", extent);
				addAttr<double, Group>(grpPar, "TOL", r->maxTol());
				addAttr<size_t, Group>(grpPar, "MXITER", r->maxIter());
				addAttr<double, Group>(grpPar, "GAMMA", r->gamma());
				addAttr<double, Group>(grpPar, "ETASCA", r->etasca());
				
				// Wavelengths
				addAttr<string, Group>(grpPar, "Wavelengths_str", r->getWavelengths());
				std::set<double> wavelengths;
				r->getWavelengths(wavelengths);
				std::vector<double> vwvs(wavelengths.begin(), wavelengths.end());
				addDatasetArray<double, Group>(grpPar, "Wavelengths", vwvs.size(), 1, vwvs.data());
				
				addAttr<double, Group>(grpPar, "NAMBIENT", r->nAmbient());

				// Effective radii
				addAttr<string, Group>(grpPar, "Effective_Radii_str", r->getAeff());
				std::set<double> aeffs;
				r->getWavelengths(aeffs);
				std::vector<double> va(aeffs.begin(), aeffs.end());
				addDatasetArray<double, Group>(grpPar, "Effective_Radii", va.size(), 1, va.data());

				// Incident polarizations (needed for fml conversions!)
				Eigen::VectorXd ipol(6);
				ipol(0) = r->PolState(0);
				ipol(1) = r->PolState(1);
				ipol(2) = r->PolState(2);
				ipol(3) = r->PolState(3);
				ipol(4) = r->PolState(4);
				ipol(5) = r->PolState(5);
				addAttrEigen<Eigen::VectorXd, Group>(grpPar, "PolState", ipol);
				addAttr<size_t, Group>(grpPar, "IORTH", r->OrthPolState());
				
				// IWRPOL is retired. It was dropped in 7.2.
				//addAttr<bool, Group>(grpPar, "IWRPOL", r->writePol());
				addAttr<int, Group>(grpPar, "IWRKSC", (int) r->writeSca());

				// Rotations
				// Beta, Theta, Phi
				rotations rot;
				r->getRots(rot);
				addAttr<string, Group>(grpPar, "Betas", rot.betas());
				addAttr<string, Group>(grpPar, "Thetas", rot.thetas());
				addAttr<string, Group>(grpPar, "Phis", rot.phis());
				std::set<double> srot;
				rot.betas(srot);
				std::vector<double> vrot(srot.begin(), srot.end());
				addDatasetArray<double, Group>(grpPar, "Betas", vrot.size(), 1, vrot.data());
				rot.thetas(srot);
				vrot = std::vector<double>(srot.begin(), srot.end());
				addDatasetArray<double, Group>(grpPar, "Thetas", vrot.size(), 1, vrot.data());
				rot.phis(srot);
				vrot = std::vector<double>(srot.begin(), srot.end());
				addDatasetArray<double, Group>(grpPar, "Phis", vrot.size(), 1, vrot.data());

				Eigen::Vector3i iwav;
				iwav(0) = r->firstOri(0);
				iwav(1) = r->firstOri(1);
				iwav(2) = r->firstOri(2);
				addAttrEigen<Eigen::VectorXi, Group>(grpPar, "firstOri", iwav);

				// Get SIJ matrix
				std::set<size_t> sij;
				r->getSIJ(sij);
				std::vector<size_t> vsij(sij.begin(), sij.end());
				addAttr<size_t, Group>(grpPar, "NSMELTS", sij.size());
				addDatasetArray<size_t, Group>(grpPar, "SIJs", vsij.size(), 1, vsij.data());


				addAttr<string, Group>(grpPar, "CMDFRM", r->getCMDFRM());
				addAttr<size_t, Group>(grpPar, "NPLANES", r->numPlanes());
				// Write out scattering plane information
				Eigen::MatrixXd planes(r->numPlanes(), 4);
				for (size_t i=0; i < r->numPlanes(); ++i)
				{
					double phi, thetan_min, thetan_max, dtheta;
					r->getPlane(i+1,phi,thetan_min,thetan_max,dtheta);
					planes(i,0) = phi;
					planes(i,1) = thetan_min;
					planes(i,2) = thetan_max;
					planes(i,3) = dtheta;
				}
				addDatasetEigen<Eigen::MatrixXd, Group>(grpPar, "Planes", planes);

				//return grpRot;
			}
		}
	}
}
