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
#include "../../rtmath/rtmath/ddscat/ddweights.h"
#include "../../rtmath/rtmath/plugin.h"
#include "../../rtmath/rtmath/error/debug.h"
#include "../../rtmath/rtmath/error/error.h"

#include "RectilinearMesh3d.h"
#include "MaterialList.h"
#include "QuadMesh3d.h"
#include "WritePoints.h"


namespace rtmath {
	namespace plugins {
		namespace silo {

			/// Function to construct a Cartesian mesh corresponding to the beta and theta rotations
			void CreateMesh(DBfile *df, const rtmath::ddscat::ddOutput *ddo)
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

			bool match_silo_ddOutput(const char* fsilo, const char* type)
			{
				using namespace boost::filesystem;
				using std::string;
				using std::ofstream;

				string stype(type);
				path pPrefix(fsilo);
				if (stype == "silo" || stype == ".silo") return true;
				else if (pPrefix.extension() == ".silo") return true;
				return false;
			}

			void write_silo_ddOutput(const char* fsilo, const rtmath::ddscat::ddOutput *ddo)
			{
				using std::string;
				using std::ofstream;
				using namespace boost::filesystem;

				DBfile *f = DBCreate(fsilo, DB_CLOBBER, DB_LOCAL,
					ddo->description.c_str(), // Optional string describing file
					DB_PDB);
				TASSERT(f);

				//CreateMesh(f, ddo);
				/*
				Eigen::Matrix3f steps = shp->maxs - shp->mins + 1;
				RectilinearMesh3D B((int)steps(0), (int)steps(1), (int)steps(2));
				B.SetXValues(shp->mins(0), shp->maxs(0));
				B.SetYValues(shp->mins(1), shp->maxs(1));
				B.SetZValues(shp->mins(2), shp->maxs(2));

				for (const auto &diel : shp->Dielectrics)
				B.AddMaterial(boost::lexical_cast<std::string>(diel).c_str());

				B.WriteFile(f);
				*/

				auto shp = ddo->shape;
				std::array<std::string, 3> axislabels = { "x", "y", "z" };
				std::array<std::string, 3> axisunits = { "dipoles", "dipoles", "dipoles" };
				

				std::vector<std::tuple<std::string, std::string, 
					const Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> > > vals;
				vals.push_back(std::tuple<std::string, std::string,
					const Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> >
					(std::string("Dielectric_x"), std::string("Dimensionless"),
					shp->latticePtsRi.col(0)));
				vals.push_back(std::tuple<std::string, std::string,
					const Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> >
					(std::string("Dielectric_y"), std::string("Dimensionless"),
					shp->latticePtsRi.col(1)));
				vals.push_back(std::tuple<std::string, std::string,
					const Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> >
					(std::string("Dielectric_z"), std::string("Dimensionless"),
					shp->latticePtsRi.col(2)));

				for (const auto &extras : shp->latticeExtras)
				{
					vals.push_back(std::tuple<std::string, std::string,
						const Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> >
						(extras.first, std::string("Unknown"),
						extras.second));
				}
					
				WritePoints(f, axislabels, axisunits, shp->latticePtsStd, vals);

				DBClose(f);

			}

		}
	}
}
