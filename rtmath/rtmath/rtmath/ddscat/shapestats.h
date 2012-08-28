#pragma once
#include <vector>
#include <map>
#include <set>

#include <boost/shared_ptr.hpp>
#include "../matrixop.h"
#include "shapefile.h"
#include "shapestatsRotated.h"

namespace rtmath {
	namespace ddscat {

		class shapeFileStatsBase
		{
		public:
			// Function that, if the shapefile referenced is not loaded, reloads the shapefile
			// Required for hulling or stats adding operations
			bool load();

			// Set rotation matrix, with each value in degrees
			//void setRot(double beta, double theta, double phi);
			void calcStatsBase();
			// calcStatsRot calculates the stats RELATIVE to the shapefile default rot.
			void calcStatsRot(double beta, double theta, double phi);

			// rot is the effective rotation designated by the choice of a1 and a2
			matrixop rot, invrot;
			double beta, theta, phi;

			// The constant multipliers! d is unknown!
			double V_cell_const, V_dipoles_const;
			double aeff_dipoles_const;

			// These require convex hull calculations
			double max_distance;
			double a_circum_sphere;
			double V_circum_sphere;
			double SA_circum_sphere;
			
			double V_convex_hull;
			double aeff_V_convex_hull;
			double SA_convex_hull;
			double aeff_SA_convex_hull;

			// Special stats calculated only in default orientation
			//
			double V_ellipsoid_max;
			double aeff_ellipsoid_max;
			double V_ellipsoid_rms;
			double aeff_ellipsoid_rms;

			// Extend to get volume fractions
			double f_circum_sphere;
			double f_convex_hull;
			double f_ellipsoid_max;
			double f_ellipsoid_rms;

			static const unsigned int _maxVersion;
			unsigned int _currVersion;

			// Before normalization and rotation
			matrixop b_min, b_max, b_mean;

			std::set<shapeFileStatsRotated> rotations;

			// The object
			boost::shared_ptr<shapefile> _shp;
		protected:
			shapeFileStatsBase();
			virtual ~shapeFileStatsBase();
			
			bool _valid;
		private:
			friend class boost::serialization::access;
		};

		class shapeFileStats : public shapeFileStatsBase
		{
		public:
			shapeFileStats();
			shapeFileStats(const shapefile &shp);
			shapeFileStats(const boost::shared_ptr<const shapefile> &shp);
		private:
			friend class boost::serialization::access;
		public:
			static boost::shared_ptr<shapeFileStats> genStats(
				const std::string &shpfile, const std::string &statsfile = "");
		};

	}
}
