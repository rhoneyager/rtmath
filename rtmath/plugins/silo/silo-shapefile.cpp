/// \brief Provides silo file IO
#define _SCL_SECURE_NO_WARNINGS

#include <array>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <fstream>
#include <string>
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <tuple>

#define DB_USE_MODERN_DTPTR
#include <silo.h>

#include "../../rtmath/rtmath/defs.h"
#include "../../rtmath/rtmath/ddscat/shapefile.h"
#include "../../rtmath/rtmath/plugin.h"
#include "../../rtmath/rtmath/error/debug.h"
#include "../../rtmath/rtmath/error/error.h"

#include "RectilinearMesh3d.h"
#include "MaterialList.h"
#include "QuadMesh3d.h"



namespace rtmath {
	namespace plugins {
		namespace silo {

			void WritePoints(DBfile *db, const std::array<std::string, 3> &axislabels,
				const std::array<std::string, 3> &axisunits, 
				const Eigen::Matrix<float, Eigen::Dynamic, 3> &pts,
				std::vector<std::tuple<std::string, std::string, 
				const Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> > > &vals)
			{
				// Write the point mesh
				DBoptlist *optlist = DBMakeOptlist(6);
				DBAddOption(optlist, DBOPT_XLABEL, (void*)axislabels[0].c_str());
				DBAddOption(optlist, DBOPT_YLABEL, (void*)axislabels[1].c_str());
				DBAddOption(optlist, DBOPT_ZLABEL, (void*)axislabels[2].c_str());
				DBAddOption(optlist, DBOPT_XUNITS, (void *)axisunits[0].c_str());
				DBAddOption(optlist, DBOPT_YUNITS, (void *)axisunits[0].c_str());
				DBAddOption(optlist, DBOPT_ZUNITS, (void *)axisunits[0].c_str());
				const int numPoints = (int)pts.rows();

				// Convert the points, by coordinate, into arrays
				/*
				float *xs = new float[numPoints];
				float *ys = new float[numPoints];
				float *zs = new float[numPoints];
				std::copy_n(pts.col(0).data(), numPoints, xs);
				std::copy_n(pts.col(1).data(), numPoints, xs);
				std::copy_n(pts.col(2).data(), numPoints, xs);
				*/
				/*
				std::cout << "xs\n";
				for (size_t i=0; i<numPoints;i++)
					std::cout << i << " - " << pts.col(0).data()[i] << std::endl;
				std::cout << "ys\n";
				for (size_t i=0; i<numPoints;i++)
					std::cout << i << " - " << pts.col(1).data()[i] << std::endl;
				std::cout << "zs\n";
				for (size_t i=0; i<numPoints;i++)
					std::cout << i << " - " << pts.col(2).data()[i] << std::endl;
				*/
				const float *pcoords[3] = { pts.col(0).data(), 
					pts.col(1).data(), pts.col(2).data() };
				DBPutPointmesh(db, "PointMesh", 3, pcoords, numPoints, DB_FLOAT, optlist);
				DBFreeOptlist(optlist);
				/*
				delete[] xs;
				delete[] ys;
				delete[] zs;
				*/

				for (const auto &a : vals)
				{
					// Write the point var
					optlist = DBMakeOptlist(1);
					DBAddOption(optlist, DBOPT_UNITS, (void *)std::get<1>(a).c_str());
					
					const float **vals = new const float*[std::get<2>(a).cols()];
					for (size_t i = 0; i < (size_t) std::get<2>(a).cols(); ++i)
						vals[i] = std::get<2>(a).col(i).data();
					//float *vals[1] = { a.second.col(0).data() };
					DBPutPointvar(db, std::get<0>(a).c_str(), "PointMesh", 
						(int) std::get<2>(a).cols(), 
						vals, 
						numPoints, 
						DB_FLOAT, 
						optlist);
					DBFreeOptlist(optlist);
					delete[] vals;
				}
			}

			bool match_silo_shapefile(const char* fsilo)
			{
				using namespace boost::filesystem;
				using std::string;
				using std::ofstream;

				path pPrefix(fsilo);
				if (pPrefix.extension() == ".silo") return true;
				return false;
			}

			void write_silo_shapefile(const char* fsilo, const rtmath::ddscat::shapefile::shapefile *shp)
			{
				using std::string;
				using std::ofstream;
				using namespace boost::filesystem;

				DBfile *f = DBCreate(fsilo, DB_CLOBBER, DB_LOCAL,
					shp->desc.c_str(), // Optional string describing file
					DB_PDB);
				TASSERT(f);

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
