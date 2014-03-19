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

				int fillvalue = -1;   /* Fill value for the dataset */
				DSetCreatPropList plist;
				plist.setFillValue(PredType::NATIVE_INT, &fillvalue);

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
				shared_ptr<Group> gV(new Group(statsraw->createGroup("Volumetric")));
				write_hdf5_statsvolumetric(gV, "Circum_sphere", &(s->Scircum_sphere));
				write_hdf5_statsvolumetric(gV, "Convex_hull", &(s->Sconvex_hull));
				write_hdf5_statsvolumetric(gV, "Voronoi_hull", &(s->SVoronoi_hull));
				write_hdf5_statsvolumetric(gV, "Ellipsoid_max", &(s->Sellipsoid_max));
				write_hdf5_statsvolumetric(gV, "Ellipsoid_rms", &(s->Sellipsoid_rms));

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
				std::string pShape;
				{
					std::ostringstream o;
					o << "/Hashed/" << s->_shp->hash().string() << "/Shape";
					pShape = o.str();
				}
				statsraw->link(H5L_TYPE_SOFT, pShape, "Shape");

				return statsraw;
			}


			/*
			/// Routine writes a full, isolated shapefile entry
			void write_hdf5_shapestats(const char* filename,
			const rtmath::ddscat::stats::shapeFileStats *s)
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
			shared_ptr<Group> shpgroup(new Group(grpHashes->createGroup(s->_shp->hash().string().c_str())));
			shared_ptr<Group> statsbase = write_hdf5_statsrawdata(shpgroup, s);
			shared_ptr<Group> shapebase = write_hdf5_shaperawdata(shpgroup, s->_shp.get());


			statsbase->link(H5L_TYPE_HARD, ".", "/Stats");
			shapebase->link(H5L_TYPE_HARD, ".", "/Shape");
			//file->link(H5L_TYPE_SOFT, newbase->, "Shape");
			} catch (std::exception &e)
			{
			std::cerr << e.what() << "\n";
			throw e;
			}
			}
			*/
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
			shared_ptr<Group> newshapebase = write_hdf5_shaperawdata(grpHash, s->_shp.get());

			return h; // Pass back the handle
		}

	}
}
