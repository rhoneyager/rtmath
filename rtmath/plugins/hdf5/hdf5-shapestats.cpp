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

#include "../../related/rtmath_hdf5_cpp/export-hdf5.h"
#include "../../rtmath/rtmath/defs.h"
#include "../../rtmath/rtmath/ddscat/shapefile.h"
#include "../../rtmath/rtmath/ddscat/shapestats.h"
#include "../../rtmath/rtmath/plugin.h"
#include "../../rtmath/rtmath/error/debug.h"
#include "../../rtmath/rtmath/error/error.h"

#include "plugin-hdf5.h"

namespace rtmath {
	namespace plugins {
		namespace hdf5 {

			template<class T, class U>
			void vectorize(const T& src, U &arr, size_t r)
			{
				// Passing in an _array_ of Eigen matrices. Take each matrix and copy the internal data into 
				// the output array.
				size_t i = 0;
				for (const auto &mat : src)
				{
					for (size_t row = 0; row < (size_t)mat.rows(); ++row)
						for (size_t col = 0; col < (size_t)mat.cols(); ++col)
						{
						arr(r, i) = mat(row, col);
						++i;
						}
				}
			}



			/// \param base is the base to write the subgroups to. From here, "./Stats" is the root of the routine's output.
			std::shared_ptr<H5::Group> write_hdf5_statsrawdata(std::shared_ptr<H5::Group> base, 
				const rtmath::ddscat::stats::shapeFileStats *s)
			{
				using std::shared_ptr;
				using namespace H5;

				shared_ptr<Group> statsraw(new Group(base->createGroup("Stats")));

				auto make_plist = [](size_t rows, size_t cols)
				{
					hsize_t chunk[2] = { (hsize_t)rows, (hsize_t)cols };
					auto plist = std::shared_ptr<DSetCreatPropList>(new DSetCreatPropList);
					plist->setChunk(2, chunk);
#if COMPRESS_ZLIB
					plist->setDeflate(6);
#endif
					return plist;
				};

				int fillvalue = -1;   /* Fill value for the dataset */
				DSetCreatPropList plist;
				plist.setFillValue(PredType::NATIVE_INT, &fillvalue);

				using std::string;
				addAttr<string, Group>(statsraw, "ingest_timestamp", s->ingest_timestamp);
				addAttr<string, Group>(statsraw, "ingest_hostname", s->ingest_hostname);
				addAttr<string, Group>(statsraw, "ingest_username", s->ingest_username); // Not all ingests have this...
				addAttr<int, Group>(statsraw, "ingest_rtmath_version", s->ingest_rtmath_version);

				// The full hash
				addAttr<uint64_t, Group>(statsraw, "Hash_Lower", s->_shp->hash().lower);
				addAttr<uint64_t, Group>(statsraw, "Hash_Upper", s->_shp->hash().upper);

				addAttrEigen<Eigen::Vector3f, Group>(statsraw, "b_min", s->b_min);
				addAttrEigen<Eigen::Vector3f, Group>(statsraw, "b_max", s->b_max);
				addAttrEigen<Eigen::Vector3f, Group>(statsraw, "b_mean", s->b_mean);

				addAttr<float, Group>(statsraw, "V_cell_const", s->V_cell_const);
				addAttr<float, Group>(statsraw, "V_dipoles_const", s->V_dipoles_const);
				addAttr<float, Group>(statsraw, "aeff_dipoles_const", s->aeff_dipoles_const);
				addAttr<float, Group>(statsraw, "max_distance", s->max_distance);

				// Rotation and inverse rotation matrix are written out as Eigen datasets
				addDatasetEigen<Eigen::Matrix3f, Group>(statsraw, "rot", s->rot);
				addDatasetEigen<Eigen::Matrix3f, Group>(statsraw, "invrot", s->invrot);
				addAttr<double, Group>(statsraw, "beta", s->beta);
				addAttr<double, Group>(statsraw, "theta", s->theta);
				addAttr<double, Group>(statsraw, "phi", s->phi);

				// Volumetric data
				{
					struct vdata {
						const char* name;
						float V, SA, aeff_SA, aeff_V, f;
					};

					const char* names[4] = { "Circum_Sphere", "Convex_Hull", "Voronoi_Hull", "Ellipsoid_Max" };

					std::array<vdata, 4> data;
					data[0].name = names[0]; data[0].V = s->Scircum_sphere.V; data[0].SA = s->Scircum_sphere.SA; 
					data[0].aeff_V = s->Scircum_sphere.aeff_V; data[0].aeff_SA = s->Scircum_sphere.aeff_SA; data[0].f = s->Scircum_sphere.f;

					data[1].name = names[1]; data[1].V = s->Sconvex_hull.V; data[1].SA = s->Sconvex_hull.SA;
					data[1].aeff_V = s->Sconvex_hull.aeff_V; data[1].aeff_SA = s->Sconvex_hull.aeff_SA; data[1].f = s->Sconvex_hull.f;

					data[2].name = names[2]; data[2].V = s->SVoronoi_hull.V; data[2].SA = s->SVoronoi_hull.SA;
					data[2].aeff_V = s->SVoronoi_hull.aeff_V; data[2].aeff_SA = s->SVoronoi_hull.aeff_SA; data[2].f = s->SVoronoi_hull.f;

					data[3].name = names[3]; data[3].V = s->Sellipsoid_max.V; data[3].SA = s->Sellipsoid_max.SA;
					data[3].aeff_V = s->Sellipsoid_max.aeff_V; data[3].aeff_SA = s->Sellipsoid_max.aeff_SA; data[3].f = s->Sellipsoid_max.f;

					//data[4].name = names[4]; data[4].V = s->Sellipsoid_rms.V; data[4].SA = s->Sellipsoid_rms.SA;
					//data[4].aeff_V = s->Sellipsoid_rms.aeff_V; data[4].aeff_SA = s->Sellipsoid_rms.aeff_SA; data[4].f = s->Sellipsoid_rms.f;

					hsize_t dim[1] = { data.size() };
					DataSpace space(1, dim);
					CompType sType(sizeof(vdata));
					H5::StrType strtype(0, H5T_VARIABLE);

					sType.insertMember("Method", HOFFSET(vdata, name), strtype);
					sType.insertMember("V", HOFFSET(vdata, V), PredType::NATIVE_FLOAT);
					sType.insertMember("SA", HOFFSET(vdata, SA), PredType::NATIVE_FLOAT);
					sType.insertMember("aeff_V", HOFFSET(vdata, aeff_V), PredType::NATIVE_FLOAT);
					sType.insertMember("aeff_SA", HOFFSET(vdata, aeff_SA), PredType::NATIVE_FLOAT);
					sType.insertMember("f", HOFFSET(vdata, f), PredType::NATIVE_FLOAT);


					std::shared_ptr<DataSet> gV(new DataSet(statsraw->createDataSet("Volumetric", sType, space)));
					gV->write(data.data(), sType);
				}

				// Rotations
				{
					shared_ptr<Group> grpRotations(new Group(statsraw->createGroup("Rotations")));

					using namespace std;
					using namespace rtmath::ddscat::stats;
					Eigen::Matrix<float, Eigen::Dynamic, rotColDefs::NUM_ROTDEFS_FLOAT> tblBasic;
					// Matrix and vector tables get written out also as arrays
					const size_t matSize = 9 * rotColDefs::NUM_MATRIXDEFS;
					const size_t vecSize = 4 * rotColDefs::NUM_VECTORDEFS;
					Eigen::Matrix<float, Eigen::Dynamic, matSize> tblMatrices;
					Eigen::Matrix<float, Eigen::Dynamic, vecSize> tblVectors;

					const size_t rows = s->rotstats.size();
					tblBasic.resize(rows, rotColDefs::NUM_ROTDEFS_FLOAT);
					tblMatrices.resize(rows, matSize);
					tblVectors.resize(rows, vecSize);

					size_t row = 0;
					for (const auto &rot : s->rotstats)
					{
						//write_hdf5_statsrotatedrawdata(grpRotations, rot.get());
						const basicTable &tbl = rot.get<0>();
						const matrixTable &mat = rot.get<1>();
						const vectorTable &vec = rot.get<2>();

						for (size_t i = 0; i < rotColDefs::NUM_ROTDEFS_FLOAT; ++i)
							tblBasic(row, i) = tbl[i];
						vectorize(mat, tblMatrices, row);
						vectorize(vec, tblVectors, row);

						++row;
					}

					auto dBasic = addDatasetEigen(grpRotations, "Basic", tblBasic, make_plist(rows, rotColDefs::NUM_ROTDEFS_FLOAT));
					auto dMats = addDatasetEigen(grpRotations, "Matrices", tblMatrices, make_plist(rows, matSize));
					auto dVecs = addDatasetEigen(grpRotations, "Vectors", tblVectors, make_plist(rows, vecSize));
					
					addColNames(dBasic, rotColDefs::NUM_ROTDEFS_FLOAT, rotColDefs::stringifyBasic);
					addColNames(dMats, 9 * rotColDefs::NUM_MATRIXDEFS, rotColDefs::stringifyMatrix, 9, 3);
					addColNames(dVecs, 4 * rotColDefs::NUM_VECTORDEFS, rotColDefs::stringifyVector, 4);
				}

				return statsraw;
			}

		}
	}

	namespace registry
	{
		using std::shared_ptr;
		using namespace rtmath::plugins::hdf5;
		
		template<>
		shared_ptr<IOhandler> 
			write_file_type_multi<rtmath::ddscat::stats::shapeFileStats>
			(shared_ptr<IOhandler> sh, shared_ptr<IO_options> opts,
			const rtmath::ddscat::stats::shapeFileStats *s)
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
			shared_ptr<Group> grpHash = openOrCreateGroup(grpHashes, s->_shp->hash().string().c_str());
			// Group "Hashed"/shp->hash/"Shape". If it exists, overwrite it. There should be no hard links here.
			/// \note The unlink operation does not really free the space..... Should warn the user.
			if (groupExists(grpHash, "Stats")) return h; //grpHash->unlink("Stats");

			/// \todo Modify to also support external symlinks
			shared_ptr<Group> newstatsbase = write_hdf5_statsrawdata(grpHash, s);

			return h; // Pass back the handle
		}

	}
}
