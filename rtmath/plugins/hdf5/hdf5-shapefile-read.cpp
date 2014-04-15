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
#include "../../rtmath/rtmath/ddscat/shapefile.h"
#include "../../rtmath/rtmath/plugin.h"
#include "../../rtmath/rtmath/error/debug.h"
#include "../../rtmath/rtmath/error/error.h"

#include "plugin-hdf5.h"
#include "../../related/rtmath_hdf5_cpp/export-hdf5.h"
#include <hdf5.h>
#include <H5Cpp.h>
#include "cmake-settings.h"

namespace rtmath {
	namespace plugins {
		namespace hdf5 {

			bool read_hdf5_shaperawdata(std::shared_ptr<H5::Group> base, 
				rtmath::ddscat::shapefile::shapefile *shp)
			{
				using std::shared_ptr;
				using namespace H5;

				// Read in attributes:

				// Description
				readAttr<std::string, Group>(base, "Description", shp->desc);
				// Shape hash
				HASH_t hash;
				readAttr<uint64_t, Group>(base, "Hash_Lower", hash.lower);
				readAttr<uint64_t, Group>(base, "Hash_Upper", hash.upper);
				shp->setHash(hash);
				// Number of points
				size_t numPoints;
				readAttr<size_t, Group>(base, "Number_of_points", numPoints);
				shp->resize(numPoints);
				// Source_filename
				readAttr<std::string, Group>(base, "Source_Filename", shp->filename);
				// mins, maxs, means
				readAttrEigen<Eigen::Array3f, Group>(base, "mins", shp->mins);
				readAttrEigen<Eigen::Array3f, Group>(base, "maxs", shp->maxs);
				readAttrEigen<Eigen::Array3f, Group>(base, "means", shp->means);
				// a1, a2, a3, d, x0, xd
				readAttrEigen<Eigen::Array3f, Group>(base, "a1", shp->a1);
				readAttrEigen<Eigen::Array3f, Group>(base, "a2", shp->a2);
				readAttrEigen<Eigen::Array3f, Group>(base, "a3", shp->a3);
				readAttrEigen<Eigen::Array3f, Group>(base, "d", shp->d);
				readAttrEigen<Eigen::Array3f, Group>(base, "x0", shp->x0);
				readAttrEigen<Eigen::Array3f, Group>(base, "xd", shp->xd);

				
				// Read in tables:
				Eigen::Matrix<int, Eigen::Dynamic, 1> lptsi;
				// Dielectrics
				readDatasetEigen<Eigen::Matrix<int, Eigen::Dynamic, 1>, Group>
					(base, "Dielectrics", lptsi);
				for (size_t i=0; i< (size_t) lptsi.rows(); ++i)
					shp->Dielectrics.insert((size_t) lptsi(i));

				// latticePts
				Eigen::Matrix<int, Eigen::Dynamic, 3> lpts;
				Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> lptsf;
				readDatasetEigen<Eigen::Matrix<int, Eigen::Dynamic, 3>, Group>
					(base, "latticePts", lpts);
				shp->latticePts = lpts.cast<float>();
				// latticePtsIndex
				readDatasetEigen<Eigen::Matrix<int, Eigen::Dynamic, 1>, Group>
					(base, "latticePtsIndex", lptsi);
				shp->latticeIndex = lptsi;
				// latticePtsNorm
				readDatasetEigen<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>, Group>
					(base, "latticePtsNorm", lptsf);
				shp->latticePtsNorm = lptsf;
				// latticePtsRi
				readDatasetEigen<Eigen::Matrix<int, Eigen::Dynamic, 3>, Group>
					(base, "latticePtsRi", lpts);
				shp->latticePtsRi = lpts.cast<float>();
				// latticePtsStd
				readDatasetEigen<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>, Group>
					(base, "latticePtsStd", lptsf);
				shp->latticePtsStd = lptsf;

				// Extras group
				// Read in all extras tables
				shared_ptr<Group> grpExtras = openGroup(base, "Extras");
				// Iterate over tables
				hsize_t nObjs = grpExtras->getNumObjs();
				for (hsize_t i=0; i<nObjs; ++i)
				{
					std::string name = grpExtras->getObjnameByIdx(i);
					H5G_obj_t t = grpExtras->getObjTypeByIdx(i);
					if (t != H5G_obj_t::H5G_GROUP) continue;

					boost::shared_ptr<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> > 
						mextra(new Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>);
					readDatasetEigen<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>, Group>
						(grpExtras, name.c_str(), *(mextra));
					shp->latticeExtras[name] = mextra;
				}

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
			read_file_type_multi<rtmath::ddscat::shapefile::shapefile>
			(shared_ptr<IOhandler> sh, shared_ptr<IO_options> opts,
			rtmath::ddscat::shapefile::shapefile *s)
		{
			std::string filename = opts->filename();
			IOhandler::IOtype iotype = opts->getVal<IOhandler::IOtype>("iotype", IOhandler::IOtype::READONLY);
			//IOhandler::IOtype iotype = opts->iotype();
			std::string key = opts->getVal<std::string>("key");
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
			if (!grpHashes) RTthrow debug::xMissingFile(key.c_str());

			shared_ptr<Group> grpHash = openGroup(grpHashes, key.c_str());
			shared_ptr<Group> grpShape = openGroup(grpHash, "Shape");
			if (!grpShape) RTthrow debug::xMissingFile(key.c_str());
			read_hdf5_shaperawdata(grpShape, s);

			// Iterate over hash entries
			/*
			hsize_t nObjs = grpHashes->getNumObjs();
			for (hsize_t i=0; i<nObjs; ++i)
			{
				std::string hashname = grpHashes->getObjnameByIdx(i);
				H5G_obj_t t = grpHashes->getObjTypeByIdx(i);
				if (t != H5G_obj_t::H5G_GROUP) continue;

				shared_ptr<Group> grpHash = openGroup(grpHashes, hashname.c_str());

				shared_ptr<Group> grpShape = openGroup(grpHash, "Shape");
				if (!grpShape) continue;

				//boost::shared_ptr<rtmath::ddscat::shapefile::shapefile> res(new rtmath::ddscat::shapefile::shapefile);
				//if (read_hdf5_shaperawdata(grpShape, res))
				//	s.push_back(res);
				read_hdf5_shaperawdata(grpShape, s);
			}
			*/
			/// \todo Implement opts searching features

			return h;
		}

		template<>
		std::shared_ptr<IOhandler>
			read_file_type_vector<rtmath::ddscat::shapefile::shapefile>
			(std::shared_ptr<IOhandler> sh, std::shared_ptr<IO_options> opts,
			std::vector<boost::shared_ptr<rtmath::ddscat::shapefile::shapefile> > &s)
		{
			RTthrow debug::xUnimplementedFunction();
			return sh;
		}
	}
}
