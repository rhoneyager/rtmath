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
#include "../../rtmath/rtmath/plugin.h"
#include "../../rtmath/rtmath/error/debug.h"
#include "../../rtmath/rtmath/error/error.h"

#include "plugin-hdf5.h"

namespace rtmath {
	namespace plugins {
		namespace hdf5 {

			std::shared_ptr<H5::Group> openOrCreateGroup(std::shared_ptr<H5::CommonFG> base, const char* name)
			{
				std::shared_ptr<H5::Group> res;
				try {
					res = std::shared_ptr<H5::Group>(new H5::Group( base->openGroup( name )));
				} catch( H5::GroupIException not_found_error ) {
					res = std::shared_ptr<H5::Group>(new H5::Group( base->createGroup( name )));
				} catch( H5::FileIException not_found_error ) {
					res = std::shared_ptr<H5::Group>(new H5::Group( base->createGroup( name )));
				}
				return res;
			}

			bool groupExists(std::shared_ptr<H5::CommonFG> base, const char* name)
			{
				try {
					H5::Group( base->openGroup( name ));
					return true;
				} catch( H5::GroupIException not_found_error ) {
					return false;
				}
			}

			bool match_hdf5_shapefile(const char* filename, const char* type)
			{
				using namespace boost::filesystem;
				using std::string;
				using std::ofstream;

				string stype(type);
				path pPrefix(filename);
				if (stype == "hdf5" || stype == ".hdf5") return true;
				else if (pPrefix.extension() == ".hdf5") return true;
				return false;
			}
			
			bool match_hdf5_multi(const char* filename, const char* type, std::shared_ptr<rtmath::registry::IOhandler> h)
			{
				if (h)
				{
					if (h->getId() != PLUGINID) return false;
					return true;
				} else {
					return match_hdf5_shapefile(filename, type);
				}
			}


			/// \param base is the base to write the subgroups to. From here, "./Shape" is the root of the routine's output.
			std::shared_ptr<H5::Group> write_hdf5_shaperawdata(std::shared_ptr<H5::Group> base, 
				const rtmath::ddscat::shapefile::shapefile *shp)
			{
				using std::shared_ptr;
				using namespace H5;

				shared_ptr<Group> shpraw(new Group(base->createGroup("Shape")));
				
				int fillvalue = -1;   /* Fill value for the dataset */
				DSetCreatPropList plist;
				plist.setFillValue(PredType::NATIVE_INT, &fillvalue);

				// Create a dataspace describing the dataset size
				hsize_t fDimBasic[] = {shp->numPoints, 3};
				DataSpace fspacePts( 2, fDimBasic );
				// Write the entire dataset (no hyperslabs necessary)
				shared_ptr<DataSet> latticePts(new DataSet(shpraw->createDataSet("latticePts", PredType::NATIVE_INT, 
					fspacePts, plist)));
				Eigen::Matrix<int, Eigen::Dynamic, 3, Eigen::RowMajor> latticePtsInt = shp->latticePts.cast<int>();
				latticePts->write(latticePtsInt.data(), PredType::NATIVE_INT);

				shared_ptr<DataSet> latticePtsRi(new DataSet(shpraw->createDataSet("latticePtsRi", PredType::NATIVE_INT, 
					fspacePts, plist)));
				Eigen::Matrix<int, Eigen::Dynamic, 3, Eigen::RowMajor> latticePtsRiInt = shp->latticePtsRi.cast<int>();
				latticePtsRi->write(latticePtsRiInt.data(), PredType::NATIVE_INT);

				shared_ptr<DataSet> latticePtsStd(new DataSet(shpraw->createDataSet("latticePtsStd", PredType::NATIVE_FLOAT, 
					fspacePts, plist)));
				Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> latticePtsStdRow(shp->latticePtsStd);
				latticePtsStd->write(latticePtsStdRow.data(), PredType::NATIVE_FLOAT);

				shared_ptr<DataSet> latticePtsNorm(new DataSet(shpraw->createDataSet("latticePtsNorm", PredType::NATIVE_FLOAT, 
					fspacePts, plist)));
				Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> latticePtsNormRow(shp->latticePtsNorm);
				latticePtsNorm->write(latticePtsNormRow.data(), PredType::NATIVE_FLOAT);

				// Write the "extra" arrays
				shared_ptr<Group> shpextras(new Group(shpraw->createGroup("Extras")));
				for (const auto& e : shp->latticeExtras)
				{
					hsize_t fDimExtra[] = {e.second->rows(), e.second->cols()};
					//if (fDimExtra[0] == 1)
					//{
					//	fDimExtra[0] = fDimExtra[1];
					//	fDimExtra[1] = 1;
					//}
					//int dimensionality = (fDimExtra[1] == 1) ? 1 : 2;
					int dimensionality = 2;
					DataSpace fDimExtraSpace( dimensionality, fDimExtra );

					shared_ptr<DataSet> data(new DataSet(shpextras->createDataSet(e.first.c_str(), PredType::NATIVE_FLOAT, 
					fDimExtraSpace, plist)));
					// Store in row-major form temporarily for proper output to hdf5
					Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> outmatrix(*(e.second));

					//data->write(e.second->data(), PredType::NATIVE_FLOAT);
					data->write(outmatrix.data(), PredType::NATIVE_FLOAT);
				}

				// Write the dielectric information
				{
					hsize_t fDielsSize[] = {shp->Dielectrics.size()};
					DataSpace fDielsSpace( 1, fDielsSize );
					shared_ptr<DataSet> diels(new DataSet(shpraw->createDataSet("Dielectrics", PredType::NATIVE_ULLONG, 
						fDielsSpace, plist)));
					std::vector<size_t> vd(shp->Dielectrics.begin(), shp->Dielectrics.end());
					diels->write(vd.data(), PredType::NATIVE_ULLONG);
				}


				// Write the basic information
				
				// Source hostname/filename pairs
				addAttr<const char*, Group>(shpraw, "Source_Filename", shp->filename.c_str());
				// Description
				addAttr<std::string, Group>(shpraw, "Description", shp->desc);
				// Number of points
				addAttr<size_t, Group>(shpraw, "Number_of_points", shp->numPoints);
				// Dielectrics
				// The full hash
				addAttr<uint64_t, Group>(shpraw, "Hash_Lower", shp->hash().lower);
				addAttr<uint64_t, Group>(shpraw, "Hash_Upper", shp->hash().upper);
				
				// mins, maxs, means
				addAttrEigen<Eigen::Array3f, Group>(shpraw, "mins", shp->mins);
				addAttrEigen<Eigen::Array3f, Group>(shpraw, "maxs", shp->maxs);
				addAttrEigen<Eigen::Array3f, Group>(shpraw, "means", shp->means);
				// a1, a2, a3, d, x0, xd
				addAttrEigen<Eigen::Array3f, Group>(shpraw, "a1", shp->a1);
				addAttrEigen<Eigen::Array3f, Group>(shpraw, "a2", shp->a2);
				addAttrEigen<Eigen::Array3f, Group>(shpraw, "a3", shp->a3);
				addAttrEigen<Eigen::Array3f, Group>(shpraw, "d", shp->d);
				addAttrEigen<Eigen::Array3f, Group>(shpraw, "x0", shp->x0);
				addAttrEigen<Eigen::Array3f, Group>(shpraw, "xd", shp->xd);

				return shpraw;
			}


			std::shared_ptr<rtmath::registry::IOhandler> write_hdf5_multi_shapefile
				(std::shared_ptr<rtmath::registry::IOhandler> sh, 
				const char* filename, 
				const rtmath::ddscat::shapefile::shapefile *shp, 
				const char* key, // Unused for this type of write
				rtmath::registry::IOhandler::IOtype iotype)
			{
				using std::shared_ptr;
				using namespace H5;
				Exception::dontPrint();
				std::shared_ptr<hdf5_handle> h;
				if (!sh)
				{
					// Access the hdf5 file
					h = std::shared_ptr<hdf5_handle>(new hdf5_handle(filename, iotype));
				} else {
					if (sh->getId() != PLUGINID) RTthrow debug::xDuplicateHook("Bad passed plugin");
					h = std::dynamic_pointer_cast<hdf5_handle>(sh);
				}

				// Check for the existence of the appropriate:
				// Group "Hashed"
				shared_ptr<Group> grpHashes = openOrCreateGroup(h->file, "Hashed");
				// Group "Hashed"/shp->hash
				shared_ptr<Group> grpShape = openOrCreateGroup(grpHashes, shp->hash().string().c_str());
				// Group "Hashed"/shp->hash/"Shape". If it exists, overwrite it. There should be no hard links here.
				/// \note The unlink operation does not really free the space..... Should warn the user.
				if (groupExists(grpShape, "Shape")) return h; //grpShape->unlink("Shape");
				shared_ptr<Group> newbase = write_hdf5_shaperawdata(grpShape, shp);

				return h; // Pass back the handle
			}

			/// Routine writes a full, isolated shapefile entry
			void write_hdf5_shapefile(const char* filename, const rtmath::ddscat::shapefile::shapefile *shp)
			{
				try {
					using std::string;
					using std::ofstream;
					using std::shared_ptr;
					using namespace H5;

					// Turn off the auto-printing when failure occurs so that we can
					// handle the errors appropriately
					Exception::dontPrint();

					shared_ptr<H5File> file(new H5File(filename, H5F_ACC_TRUNC ));
					shared_ptr<Group> grpHashes(new Group(file->createGroup("Hashed")));
					shared_ptr<Group> shpgroup(new Group(grpHashes->createGroup(shp->hash().string().c_str())));
					shared_ptr<Group> newbase = write_hdf5_shaperawdata(shpgroup, shp);

					newbase->link(H5L_TYPE_HARD, ".", "/Shape");
					//file->link(H5L_TYPE_SOFT, newbase->, "Shape");
				} catch (std::exception &e)
				{
					std::cerr << e.what() << "\n";
					throw e;
				}
			}

		}
	}
}
