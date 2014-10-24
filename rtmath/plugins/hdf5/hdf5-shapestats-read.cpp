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
#include "../../rtmath/rtmath/ddscat/shapestats.h"
#include "../../related/rtmath_hdf5_cpp/export-hdf5.h"
#include "../../rtmath/rtmath/plugin.h"
#include "../../rtmath/rtmath/error/debug.h"
#include "../../rtmath/rtmath/error/error.h"

#include "plugin-hdf5.h"
// These go last, as they do odd stuff to definitions.
#include <hdf5.h>
#include <H5Cpp.h>
namespace rtmath {
	namespace plugins {
		namespace hdf5 {

			template<class T, class U>
			void devectorize(const T& src, U &mats, size_t irow)
			{
				// Reading in one large matrix, at the specified row
				size_t i = 0;
				for (auto &mat : mats)
				{
					for (size_t row = 0; row < (size_t)mat.rows(); ++row)
						for (size_t col = 0; col < (size_t)mat.cols(); ++col)
						{
						mat(row, col) = src(irow, i);
						++i;
						}
				}
			}

			bool read_hdf5_statsrawdata(std::shared_ptr<H5::Group> base, std::shared_ptr<registry::IO_options> opts,
				boost::shared_ptr<rtmath::ddscat::stats::shapeFileStats > r)
			{
				using std::shared_ptr;
				using std::string;
				using namespace H5;
				using namespace ddscat;
				using namespace rtmath::ddscat::stats;

				readAttr<string, Group>(base, "ingest_timestamp", r->ingest_timestamp);
				readAttr<string, Group>(base, "ingest_hostname", r->ingest_hostname);
				readAttr<string, Group>(base, "ingest_username", r->ingest_username);
				readAttr<int, Group>(base, "ingest_rtmath_version", r->ingest_rtmath_version);

				if (attrExists(base, "shapeFileStats_version"))
					addAttr<int, Group>(base, "shapeFileStats_version", r->_currVersion);
				else r->_currVersion = 4; // The version where this was added.

				// Shape hash
				HASH_t hash;
				readAttr<uint64_t, Group>(base, "Hash_Lower", hash.lower);
				readAttr<uint64_t, Group>(base, "Hash_Upper", hash.upper);
				boost::shared_ptr<::rtmath::ddscat::shapefile::shapefile> shp = shapefile::shapefile::generate();
				shp->setHash(hash);
				r->_shp = shp;

				readAttrEigen<Eigen::Vector3f, Group>(base, "b_min", r->b_min);
				readAttrEigen<Eigen::Vector3f, Group>(base, "b_max", r->b_max);
				readAttrEigen<Eigen::Vector3f, Group>(base, "b_mean", r->b_mean);

				readAttr<float, Group>(base, "V_cell_const", r->V_cell_const);
				readAttr<float, Group>(base, "V_dipoles_const", r->V_dipoles_const);
				readAttr<float, Group>(base, "aeff_dipoles_const", r->aeff_dipoles_const);
				readAttr<float, Group>(base, "max_distance", r->max_distance);

				// Rotation and inverse rotation matrix are written out as Eigen datasets
				readDatasetEigen<Eigen::Matrix3f, Group>(base, "rot", r->rot);
				readDatasetEigen<Eigen::Matrix3f, Group>(base, "invrot", r->invrot);
				readAttr<float, Group>(base, "beta", r->beta);
				readAttr<float, Group>(base, "theta", r->theta);
				readAttr<float, Group>(base, "phi", r->phi);

				// Volumetric data
				{
					/// \note Has to match the definition in the writing function
					/// \todo Merge this with the writing function definition elsewhere
					/// \todo Add a name field for the volumetric class
					/// \see shapeFileStatsBase::volumetric
					/// \todo change volumetric to be contained in a map
					struct vdata { 
						const char* name;
						float V, SA, aeff_SA, aeff_V, f;
					};

					const size_t nMeths = 6;
					// Names must match writer
					const char* names[nMeths] = { "Circum_Sphere", "Convex_Hull", 
						"Voronoi_Hull", "Ellipsoid_Max",
						"RMS_Sphere", "Gyration_Sphere" };

					std::shared_ptr<DataSet> sdataset(new DataSet(base->openDataSet("Volumetric")));
					std::vector<size_t> dims;
					readDatasetDimensions(base, "Volumetric", dims);

					/// \note If another data member is added, this has to be updated!
					std::array<vdata, 4> data;
					CompType sType(sizeof(vdata));
					H5::StrType strtype(0, H5T_VARIABLE);

					sType.insertMember("Method", HOFFSET(vdata, name), strtype);
					sType.insertMember("V", HOFFSET(vdata, V), PredType::NATIVE_FLOAT);
					sType.insertMember("SA", HOFFSET(vdata, SA), PredType::NATIVE_FLOAT);
					sType.insertMember("aeff_V", HOFFSET(vdata, aeff_V), PredType::NATIVE_FLOAT);
					sType.insertMember("aeff_SA", HOFFSET(vdata, aeff_SA), PredType::NATIVE_FLOAT);
					sType.insertMember("f", HOFFSET(vdata, f), PredType::NATIVE_FLOAT);

					sdataset->read(data.data(), sType);

					for (const auto &d : data)
					{
						shapeFileStats::volumetric* v = nullptr;
						string name(d.name);
						if (name == string(names[0])) v = &(r->Scircum_sphere);
						else if (name == string(names[1])) v = &(r->Sconvex_hull);
						else if(name == string(names[2])) v = &(r->SVoronoi_hull);
						else if(name == string(names[3])) v = &(r->Sellipsoid_max);
						else if (name == string(names[4])) v = &(r->Srms_sphere);
						else if (name == string(names[5])) v = &(r->Sgyration);
						else RTthrow debug::xUnimplementedFunction();

						v->V = d.V;
						v->SA = d.SA;
						v->aeff_SA = d.aeff_SA;
						v->aeff_V = d.aeff_V;
						v->f = d.f;
					}
					
				}

				// Rotations
				{
					shared_ptr<Group> grpRotations = openGroup(base, "Rotations");

					using namespace std;
					using namespace rtmath::ddscat::stats;

					Eigen::MatrixXf tblBasic, tblMatrices, tblVectors;

					readDatasetEigen(grpRotations, "Basic", tblBasic);
					readDatasetEigen(grpRotations, "Matrices", tblMatrices);
					readDatasetEigen(grpRotations, "Vectors", tblVectors);

					
					// Matrix and vector tables get written out also as arrays
					const size_t matSize = 9 * rotColDefs::NUM_MATRIXDEFS;
					const size_t vecSize = 4 * rotColDefs::NUM_VECTORDEFS;

					for (size_t i = 0; i < (size_t)tblBasic.rows(); ++i)
					{
						rotData rot;
						basicTable &tbl = rot.get<0>();
						matrixTable &mat = rot.get<1>();
						vectorTable &vec = rot.get<2>();

						//  rotColDefs::NUM_ROTDEFS_FLOAT;
						for (size_t j = 0; j < (size_t)tblBasic.cols(); ++j)
							tbl[j] = tblBasic(i, j);
						if (tblBasic.cols() < 4) tbl[3] = 4; // Version tag added in version 5, 
						// so this is version 4 (when the hdf5 plugin was first implemented).
						
						devectorize(tblMatrices, mat, i);
						devectorize(tblVectors, vec, i);

						r->rotstats.insert(std::move(rot));
					}
				}

				r->upgrade(); // Will upgrade to the latest version if necessary.

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
			read_file_type_multi<rtmath::ddscat::stats::shapeFileStats>
			(shared_ptr<IOhandler> sh, shared_ptr<IO_options> opts,
			boost::shared_ptr<rtmath::ddscat::stats::shapeFileStats > s,
			std::shared_ptr<const rtmath::registry::collectionTyped<rtmath::ddscat::stats::shapeFileStats> >)
		{
			std::string filename = opts->filename();
			IOhandler::IOtype iotype = opts->getVal<IOhandler::IOtype>("iotype", IOhandler::IOtype::READONLY);
			//IOhandler::IOtype iotype = opts->iotype();
			std::string key;
			if (opts->hasVal("hash")) key = opts->getVal<std::string>("hash");
			else key = opts->getVal<std::string>("key");
			using std::shared_ptr;
			using namespace H5;
			Exception::dontPrint();
			std::shared_ptr<hdf5_handle> h = registry::construct_handle
				<registry::IOhandler, hdf5_handle>(
				sh, PLUGINID, [&](){return std::shared_ptr<hdf5_handle>(
				new hdf5_handle(filename.c_str(), iotype)); });

			shared_ptr<Group> grpHashes = openGroup(h->file, "Hashed");
			if (!grpHashes) RTthrow debug::xMissingFile(key.c_str());

			shared_ptr<Group> grpHash = openGroup(grpHashes, key.c_str());
			shared_ptr<Group> grpStats = openGroup(grpHash, "Stats");
			if (!grpStats) RTthrow debug::xMissingFile(key.c_str());
			read_hdf5_statsrawdata(grpStats, opts, s);

			return h;
		}

		template<>
		std::shared_ptr<IOhandler>
			read_file_type_vector<rtmath::ddscat::stats::shapeFileStats>
			(std::shared_ptr<IOhandler> sh, std::shared_ptr<IO_options> opts,
			std::vector<boost::shared_ptr<rtmath::ddscat::stats::shapeFileStats> > &s,
			std::shared_ptr<const rtmath::registry::collectionTyped<rtmath::ddscat::stats::shapeFileStats> > filter)
		{
			std::string filename = opts->filename();
			IOhandler::IOtype iotype = opts->getVal<IOhandler::IOtype>("iotype", IOhandler::IOtype::READONLY);
			//IOhandler::IOtype iotype = opts->iotype();
			std::string key = opts->getVal<std::string>("key", "");
			using std::shared_ptr;
			using namespace H5;
			Exception::dontPrint();
			std::shared_ptr<hdf5_handle> h = registry::construct_handle
				<registry::IOhandler, hdf5_handle>(
				sh, PLUGINID, [&](){return std::shared_ptr<hdf5_handle>(
				new hdf5_handle(filename.c_str(), iotype)); });

			shared_ptr<Group> grpHashes = openGroup(h->file, "Hashed");
			if (grpHashes)
			{
				hsize_t sz = grpHashes->getNumObjs();
				//s.reserve(s.size() + sz);
				for (hsize_t i = 0; i < sz; ++i)
				{
					std::string hname = grpHashes->getObjnameByIdx(i);
					H5G_obj_t t = grpHashes->getObjTypeByIdx(i);
					if (t != H5G_obj_t::H5G_GROUP) continue;
					if (key.size() && key != hname) continue;

					shared_ptr<Group> grpHash = openGroup(grpHashes, hname.c_str());
					if (!grpHash) continue; // Should never happen
					shared_ptr<Group> grpStats = openGroup(grpHash, "Stats");
					if (grpStats)
					{
						boost::shared_ptr<rtmath::ddscat::stats::shapeFileStats>
							stats(new rtmath::ddscat::stats::shapeFileStats);
						read_hdf5_statsrawdata(grpStats, opts, stats);
						if (filter) {
							if (filter->filter(stats.get()))
								s.push_back(stats);
						}
						else s.push_back(stats);
					}
				}
			}

			return h;
		}
	}
}
