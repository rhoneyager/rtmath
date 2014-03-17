/// \brief Provides silo file IO
#define _SCL_SECURE_NO_WARNINGS

#include <array>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <fstream>
#include <string>
#include <boost/filesystem/path.hpp>
#include <boost/math/constants/constants.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <tuple>

#define DB_USE_MODERN_DTPTR
#include <silo.h>

#include "../../rtmath/rtmath/defs.h"
#include "../../rtmath/rtmath/ddscat/shapefile.h"
#include "../../rtmath/rtmath/ddscat/rotations.h"
#include "../../rtmath/rtmath/ddscat/ddOutput.h"
#include "../../rtmath/rtmath/ddscat/ddOutputSingle.h"
#include "../../rtmath/rtmath/ddscat/ddweights.h"
#include "../../rtmath/rtmath/plugin.h"
#include "../../rtmath/rtmath/error/debug.h"
#include "../../rtmath/rtmath/error/error.h"

#include "RectilinearMesh3d.h"
#include "MaterialList.h"
#include "QuadMesh3d.h"
#include "WritePoints.h"

#include "plugin-silo.h"


namespace rtmath {
	namespace plugins {
		namespace silo {

			/// \brief Function to construct a Spherical mesh corresponding to the beta and theta rotations
			/// \todo Update to use new system
			void CreateMeshSpherical(DBfile *df, const rtmath::ddscat::ddOutput *ddo)
			{

				// Given the given radius and the rotations, construct vertices at the given locations
				using namespace ddscat::weights;
				using namespace std;
				ddscat::rotations rot(*(ddo->parfile.get()));
				ddscat::weights::ddWeightsDDSCAT dw(rot);
				ddscat::weights::IntervalTable3d wts;
				dw.getIntervalTable(wts);
				// wts are ordered beta, theta, phi
				/// \todo Move wts into an ordered set, in case the ordering changes
				// In addition to the vertices, the neighbors are needed.
				std::vector<int> zones;
				Eigen::Matrix<double, Eigen::Dynamic, 3> centers, edges;
				Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> vals;

				centers.resize(wts.size(), 3);
				edges.resize(wts.size() * 4, 3);
				vals.resize(wts.size(), 1); // Weight (1), Qbk (3), Qsca (3), Qabs(3), Qext(3)

				zones.reserve(wts.size());
				size_t i = 0;
				auto aToRad = [](size_t n, double *vals, double *res)
				{
					const float pi = boost::math::constants::pi<float>();
					for (size_t i=0;i<n;++i)
						res[i] = vals[i] * pi / 180.f;
				};
				const double pinit = wts.begin()->at(IntervalTable3dDefs::PHI_PIVOT);
				for (const auto &it : wts)
				{
					if (it.at(IntervalTable3dDefs::PHI_PIVOT) != pinit) continue;
					// Convert the angles into coordinates on the sphere
					// We have the lower angle bound, the upper angle bound, and the midpoint.
					// The weight is also known, as are the tables for Qbksc, ...

					/// \todo Convert here and in ddweights.cpp to template functions.
					const size_t degree = 3;
					double start_deg[degree-1] = {
						it[IntervalTable3dDefs::THETA_MIN],
						it[IntervalTable3dDefs::PHI_MIN] };

					double end_deg[degree-1] = {
						it[IntervalTable3dDefs::THETA_MAX],
						it[IntervalTable3dDefs::PHI_MAX] };

					double mid_deg[degree-1] = {
						it[IntervalTable3dDefs::THETA_PIVOT],
						it[IntervalTable3dDefs::PHI_PIVOT] };


					double start_rad[degree], end_rad[degree], mid_rad[degree];
					start_rad[0] = 1; end_rad[0] = 1; mid_rad[0] = 1;
					aToRad(2, start_deg, start_rad+1);
					aToRad(2, end_deg, end_rad+1);
					aToRad(2, mid_deg, mid_rad+1);

					double start_pol[degree], end_pol[degree], mid_pol[degree];
					radSphToCrt(degree, start_rad, start_pol);
					radSphToCrt(degree, end_rad, end_pol);
					radSphToCrt(degree, mid_rad, mid_pol);

					// We now have cartesian coordinates.
					// Can append to the growing zonelist as a set of tetrahedrons.
					centers(i, 0) = mid_pol[0];
					centers(i, 1) = mid_pol[1];
					centers(i, 2) = mid_pol[2];

					//edges;

					vals(i, 0) = it[IntervalTable3dDefs::WEIGHT];
					/// \todo Insert cross-sections here

					++i;
				}

				DBoptlist *meshOptList = DBMakeOptlist(16);
				//DBAddOption(meshOptList, DBOPT_COORDSYS, (void*) DB_SPHERICAL);
				DBAddOption(meshOptList, DBOPT_FACETYPE, (void*) DB_CURVILINEAR);
				//DBAddOption(meshOptList, 


				//const char* axisLabels[] = { "x", "y", "z" };
				const double *pcoords[3] = {centers.col(0).data(), centers.col(1).data(), centers.col(2).data() };
				int ndims[] = {(int) dw.numBetas(), (int) dw.numThetas(), 2};
				DBPutQuadmesh(df, "Rotation_Surface", 
					NULL, // Unused. Can be NULL.
					pcoords, // Pointer to arrays of coordinates
					ndims, // Array of length ndims describing mesh dimensionality
					3, // Number of dimensions
					DB_DOUBLE, // Datatype of the coordinate arrays
					DB_NONCOLLINEAR, // Coordinate array type
					meshOptList // Optional option list information
					);

				DBFreeOptlist(meshOptList);



				// Add materials

				// Finally add objects that go on the mesh


			}

			/// \brief Creates a rectilinear mesh to hold beta, theta, phi paired values
			/// \todo Update to use new system
			void CreateMeshRectilinear(DBfile *df, const char* meshname, const rtmath::ddscat::ddOutput *ddo, const size_t dimension = 3)
			{
				using namespace ddscat::weights;
				using namespace std;
				ddscat::rotations rot(*(ddo->parfile.get()));
				ddscat::weights::ddWeightsDDSCAT dw(rot);
				ddscat::weights::IntervalTable3d wts;
				dw.getIntervalTable(wts);

				size_t nBetas = dw.numBetas(), nThetas = dw.numThetas(), nPhis = dw.numPhis();
				int dims[] = { (int) nBetas, (int) nThetas, (int) nPhis};
				// Construct the coordinate matrices
				IntervalTable1d betas, thetas, phis;
				dw.wBetas.getIntervals(betas);
				dw.wThetas.getIntervals(thetas);
				dw.wPhis.getIntervals(phis);

				vector<double> abetas, athetas, aphis;
				for (auto &b : betas)
					abetas.push_back(b.at(IntervalTable1dDefs::PIVOT));
				for (auto &b : thetas)
					athetas.push_back(b.at(IntervalTable1dDefs::PIVOT));
				for (auto &b : phis)
					aphis.push_back(b.at(IntervalTable1dDefs::PIVOT));

				double *coords[] = {abetas.data(), athetas.data(), aphis.data() };

				DBoptlist *optlist = DBMakeOptlist(6);
				DBAddOption(optlist, DBOPT_XLABEL, (void*)"Beta");
				DBAddOption(optlist, DBOPT_YLABEL, (void*)"Theta");
				DBAddOption(optlist, DBOPT_ZLABEL, (void*)"Phi");
				DBAddOption(optlist, DBOPT_XUNITS, (void *)"Degrees");
				DBAddOption(optlist, DBOPT_YUNITS, (void *)"Degrees");
				DBAddOption(optlist, DBOPT_ZUNITS, (void *)"Degrees");

				DBPutQuadmesh(df, meshname,
					NULL, coords, dims, (int) dimension, DB_DOUBLE, DB_COLLINEAR, optlist);

				DBFreeOptlist(optlist);
			}

			/// \brief Write a node-centerd variable to the mesh
			/// \todo Update to use new system
			void AddToMesh(DBfile *df, const char* meshname, const rtmath::ddscat::ddOutput *ddo,
				rtmath::ddscat::stat_entries entry, const size_t dimension = 3, const char* varname = "")
			{
				// sca tables are stored as a plain set (no ordering)
				ddscat::rotations rot(*(ddo->parfile.get()));
				ddscat::weights::ddWeightsDDSCAT dw(rot);
				ddscat::weights::IntervalTable3d wts;
				dw.getIntervalTable(wts);
				size_t nBetas = dw.numBetas(), nThetas = dw.numThetas(), nPhis = dw.numPhis();
				size_t nRots = nBetas * nThetas * nPhis;
				int dims[] = {(int) nBetas, (int) nThetas, (int) nPhis};

				std::vector<double> output(nRots);
				auto getIndex = [&](boost::shared_ptr<ddscat::ddOutputSingle> os) -> size_t
				{
					size_t iBeta = dw.wBetas.getIndex(os->beta());
					size_t iTheta = dw.wBetas.getIndex(os->theta());
					size_t iPhi = dw.wBetas.getIndex(os->phi());
					size_t index = (dimension == 3) 
						? iPhi + (nPhis * iTheta) + (nThetas * nPhis * iBeta) // dimension == 3
						: iBeta + (nBetas * iTheta); // dimension == 2
					return index;
				};

				for (const auto &sca : ddo->scas)
				{
					if (dimension == 2)
						if (0 != dw.wPhis.getIndex(sca->phi()))
							continue;
					size_t index = getIndex(sca);
					double val = sca->getStatEntry(entry);
					output[index] = val;
				}

				std::string varName(varname);
				if (!varName.size()) varName = rtmath::ddscat::getStatNameFromId(entry);

				DBPutQuadvar1(df, varName.c_str(), meshname,
					(void*) output.data(), dims, (int) dimension, NULL, 0, DB_DOUBLE, DB_NODECENT, NULL);
			}


		}
	}
	
	namespace registry
	{
		using std::shared_ptr;
		using rtmath::ddscat::ddOutput;
		using namespace rtmath::plugins::silo;

		shared_ptr<IOhandler> 
			write_file_type_multi
			(shared_ptr<IOhandler> sh, shared_ptr<IO_options> opts,
			const ddOutput *ddo)
		{
				std::string filename = opts->filename();
				IOhandler::IOtype iotype = opts->iotype();

			shared_ptr<silo_handle> h;
			if (!sh)
			{
				// Access the hdf5 file
				h = shared_ptr<silo_handle>(new silo_handle(filename.c_str(), iotype));
			} else {
				if (sh->getId() != PLUGINID) RTthrow debug::xDuplicateHook("Bad passed plugin");
				h = std::dynamic_pointer_cast<silo_handle>(sh);
			}


			CreateMeshRectilinear(h->file->df, "CoordsMesh_rect_3d", ddo);
			AddToMesh(h->file->df, "CoordsMesh_rect_3d", ddo, ddscat::stat_entries::QABSM);
			AddToMesh(h->file->df, "CoordsMesh_rect_3d", ddo, ddscat::stat_entries::QBK1);
			AddToMesh(h->file->df, "CoordsMesh_rect_3d", ddo, ddscat::stat_entries::QBK2);
			AddToMesh(h->file->df, "CoordsMesh_rect_3d", ddo, ddscat::stat_entries::QBKM);
			AddToMesh(h->file->df, "CoordsMesh_rect_3d", ddo, ddscat::stat_entries::QEXTM);
			AddToMesh(h->file->df, "CoordsMesh_rect_3d", ddo, ddscat::stat_entries::QSCA1);
			AddToMesh(h->file->df, "CoordsMesh_rect_3d", ddo, ddscat::stat_entries::QSCA2);
			AddToMesh(h->file->df, "CoordsMesh_rect_3d", ddo, ddscat::stat_entries::QSCAM);

			CreateMeshRectilinear(h->file->df, "CoordsMesh_rect_2d", ddo, 2);
			AddToMesh(h->file->df, "CoordsMesh_rect_2d", ddo, ddscat::stat_entries::QABSM, 2, "QABSM_2d");
			AddToMesh(h->file->df, "CoordsMesh_rect_2d", ddo, ddscat::stat_entries::QBK1, 2, "QBK1_2d");
			AddToMesh(h->file->df, "CoordsMesh_rect_2d", ddo, ddscat::stat_entries::QBK2, 2, "QBK2_2d");
			AddToMesh(h->file->df, "CoordsMesh_rect_2d", ddo, ddscat::stat_entries::QBKM, 2, "QBKM_2d");
			AddToMesh(h->file->df, "CoordsMesh_rect_2d", ddo, ddscat::stat_entries::QEXTM, 2, "QEXTM_2d");
			AddToMesh(h->file->df, "CoordsMesh_rect_2d", ddo, ddscat::stat_entries::QSCA1, 2, "QSCA1_2d");
			AddToMesh(h->file->df, "CoordsMesh_rect_2d", ddo, ddscat::stat_entries::QSCA2, 2, "QSCA2_2d");
			AddToMesh(h->file->df, "CoordsMesh_rect_2d", ddo, ddscat::stat_entries::QSCAM, 2, "QSCAM_2d");


			//writeShape(f, "PointMesh", ddo->shape.get());
			write_file_type_multi(h, opts, ddo->shape.get());
			//write_file_type_multi(h, filename, ddo->shape.get(), key, iotype);

			return h;
		}

	}
}
