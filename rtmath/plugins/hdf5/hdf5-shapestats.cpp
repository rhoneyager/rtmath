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

			std::shared_ptr<H5::Group> write_hdf5_statsvolumetric(std::shared_ptr<H5::Group> base, 
				const char* name,
				const rtmath::ddscat::stats::shapeFileStats::volumetric *v)
			{
				using std::shared_ptr;
				using namespace H5;

				shared_ptr<Group> gv(new Group(base->createGroup(name)));

				addAttr<float, Group>(gv, "V", v->V);
				addAttr<float, Group>(gv, "aeff_V", v->aeff_V);
				addAttr<float, Group>(gv, "SA", v->SA);
				addAttr<float, Group>(gv, "aeff_SA", v->aeff_SA);
				addAttr<float, Group>(gv, "f", v->f);
				return gv;
			}

			std::shared_ptr<H5::Group> write_hdf5_statsrotatedrawdata(std::shared_ptr<H5::Group> base, 
				const rtmath::ddscat::stats::shapeFileStatsRotated *r)
			{
				using std::shared_ptr;
				using namespace H5;

				std::string rotid;
				{
					std::ostringstream o;
					o << r->beta << "," << r->theta << "," << r->phi;
					rotid = o.str();
				}
				shared_ptr<Group> grpRot(new Group(base->createGroup(rotid)));

				addAttr<double, Group>(grpRot, "beta", r->beta);
				addAttr<double, Group>(grpRot, "theta", r->theta);
				addAttr<double, Group>(grpRot, "phi", r->phi);

				using namespace rtmath::ddscat::stats;
				addDatasetEigen<shapeFileStatsRotated::dir_mat_t, Group>(grpRot, "min", r->min);
				addDatasetEigen<shapeFileStatsRotated::dir_mat_t, Group>(grpRot, "max", r->max);
				addDatasetEigen<shapeFileStatsRotated::dir_mat_t, Group>(grpRot, "sum", r->sum);
				addDatasetEigen<shapeFileStatsRotated::dir_mat_t, Group>(grpRot, "skewness", r->skewness);
				addDatasetEigen<shapeFileStatsRotated::dir_mat_t, Group>(grpRot, "kurtosis", r->kurtosis);
				addDatasetEigen<shapeFileStatsRotated::dir_mat_t, Group>(grpRot, "mom1", r->mom1);
				addDatasetEigen<shapeFileStatsRotated::dir_mat_t, Group>(grpRot, "mom2", r->mom2);

				auto addVectorEigen = [&](std::shared_ptr<H5::Group> base, const char* name,
					const shapeFileStatsRotated::vec_mat_t &val)
				{
					shared_ptr<Group> grp(new Group(base->createGroup(name)));
					for (size_t i=0; i < val.size(); ++i)
					{
						std::ostringstream o;
						o << i;
						std::string so = o.str();
						addDatasetEigen<shapeFileStatsRotated::mat_3_t, Group>(grp, so.c_str(), val[i]);
					}
				};

				//addDatasetEigen<shapeFileStatsRotated::vec_mat_t, Group>(grpRot, "mominert", r->mominert);
				//addDatasetEigen<shapeFileStatsRotated::vec_mat_t, Group>(grpRot, "covariance", r->covariance);
				addVectorEigen(grpRot, "mominert", r->mominert);
				addVectorEigen(grpRot, "covariance", r->covariance);


				addDatasetEigen<shapeFileStatsRotated::dir_mat_t, Group>(grpRot, "PE", r->PE);

				addDatasetEigen<shapeFileStatsRotated::vec_3_t, Group>(grpRot, "abs_min", r->abs_min);
				addDatasetEigen<shapeFileStatsRotated::vec_3_t, Group>(grpRot, "abs_max", r->abs_max);
				addDatasetEigen<shapeFileStatsRotated::vec_3_t, Group>(grpRot, "abs_mean", r->abs_mean);
				addDatasetEigen<shapeFileStatsRotated::vec_4_t, Group>(grpRot, "rms_mean", r->rms_mean);

				addDatasetEigen<shapeFileStatsRotated::mat_3_t, Group>(grpRot, "as_abs", r->as_abs);
				addDatasetEigen<shapeFileStatsRotated::mat_3_t, Group>(grpRot, "as_abs_mean", r->as_abs_mean);
				addDatasetEigen<shapeFileStatsRotated::mat_3_t, Group>(grpRot, "as_rms", r->as_rms);

				addDatasetEigen<Eigen::Vector3f, Group>(grpRot, "areas", r->areas);
				return grpRot;
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

					//std::shared_ptr<H5::AtomType> strtype(new H5::StrType(0, H5T_VARIABLE));
					
					/*
					shared_ptr<Group> gV(new Group(statsraw->createGroup("Volumetric")));
					write_hdf5_statsvolumetric(gV, "Circum_sphere", &(s->Scircum_sphere));
					write_hdf5_statsvolumetric(gV, "Convex_hull", &(s->Sconvex_hull));
					write_hdf5_statsvolumetric(gV, "Voronoi_hull", &(s->SVoronoi_hull));
					write_hdf5_statsvolumetric(gV, "Ellipsoid_max", &(s->Sellipsoid_max));
					write_hdf5_statsvolumetric(gV, "Ellipsoid_rms", &(s->Sellipsoid_rms));
					*/
				}

				// Rotations
				shared_ptr<Group> grpRotations(new Group(statsraw->createGroup("Rotations")));
				for (const auto &rot : s->rotations)
				{
					write_hdf5_statsrotatedrawdata(grpRotations, rot.get());
				}

				// Extract PE and moment of inertia for ease of plotting
				Eigen::MatrixXf ptsRots(s->rotations.size(), 3);
				Eigen::MatrixXf rotPE(s->rotations.size(), 1);
				Eigen::MatrixXf rotMI(s->rotations.size(), 3);
				{
					size_t i=0;
					for (auto rot = s->rotations.begin(); rot != s->rotations.end(); ++rot, ++i)
					{
						ptsRots(i,0) = (float) (*rot)->beta;
						ptsRots(i,1) = (float) (*rot)->theta;
						ptsRots(i,2) = (float) (*rot)->phi;
						rotPE(i,0) = (float) (*rot)->PE(0, 0);
						rotMI.block(i,0,1,3) = (*rot)->mominert.at(0).block<1,3>(0,0);
					}
				}
				Eigen::MatrixXf pe(s->rotations.size(), 4);
				pe.block(0,0,s->rotations.size(),3) = ptsRots;
				pe.col(3) = rotPE;
				addDatasetEigen<Eigen::MatrixXf, Group>(grpRotations, "PE", pe);
				Eigen::MatrixXf mi(s->rotations.size(), 6);
				mi.block(0,0,s->rotations.size(),3) = ptsRots;
				mi.block(0,3,s->rotations.size(),3) = rotMI;
				addDatasetEigen<Eigen::MatrixXf, Group>(grpRotations, "Moment_Inertia", mi);

				// And make a soft link to the relevent raw shape information
				/*
				std::string pShape;
				{
					std::ostringstream o;
					o << "/Hashed/" << s->_shp->hash().string() << "/Shape";
					pShape = o.str();
				}
				statsraw->link(H5L_TYPE_SOFT, pShape, "Shape");
				*/

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
