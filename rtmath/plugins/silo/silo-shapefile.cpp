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
#include "WritePoints.h"


namespace rtmath {
	namespace plugins {
		namespace silo {

			bool match_silo_shapefile(const char* fsilo, const char* type)
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
