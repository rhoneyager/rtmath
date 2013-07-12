#pragma once
#include <vector>
#include <map>
#include <set>
#include <Eigen/Core>
//#include <Eigen/Dense>
#include <Eigen/StdVector>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <boost/shared_ptr.hpp>
#include "shapefile.h"
#include "shapestatsRotated.h"

// Forward declarations
namespace rtmath {
	namespace ddscat {
		class shapeFileStatsBase;
		class shapeFileStats;
	}
}

namespace rtmath {
	namespace ddscat {
		struct rotComp
		{
			bool operator()(const boost::shared_ptr<const shapeFileStatsRotated> &lhs,
				const boost::shared_ptr<const shapeFileStatsRotated> &rhs) const;
		};

		class shapeFileStatsBase
		{
		public:
			virtual ~shapeFileStatsBase();
			// Function that, if the shapefile referenced is not loaded, reloads the shapefile
			// Required for hulling or stats adding operations
			bool load();

			// Set rotation matrix, with each value in degrees
			//void setRot(double beta, double theta, double phi);
			void calcStatsBase();
			// calcStatsRot calculates the stats RELATIVE to the shapefile default rot.
			boost::shared_ptr<const shapeFileStatsRotated> calcStatsRot(double beta, double theta, double phi);

			// rot is the effective rotation designated by the choice of a1 and a2
			Eigen::Matrix3f rot, invrot;
			double beta, theta, phi;

			// The constant multipliers! d is unknown!
			float V_cell_const, V_dipoles_const;
			float aeff_dipoles_const;

			// These require convex hull calculations
			float max_distance;
			float a_circum_sphere;
			float V_circum_sphere;
			float SA_circum_sphere;
			
			float V_convex_hull;
			float aeff_V_convex_hull;
			float SA_convex_hull;
			float aeff_SA_convex_hull;

			// Special stats calculated only in default orientation
			float V_ellipsoid_max;
			float aeff_ellipsoid_max;
			float V_ellipsoid_rms;
			float aeff_ellipsoid_rms;

			// Extend to get volume fractions
			float f_circum_sphere;
			float f_convex_hull;
			float f_ellipsoid_max;
			float f_ellipsoid_rms;

			static const unsigned int _maxVersion;
			unsigned int _currVersion;
			bool qhull_enabled;

			// Before normalization and rotation
			Eigen::Vector3f b_min, b_max, b_mean;

			std::set<boost::shared_ptr<const shapeFileStatsRotated>, rotComp > rotations;

			// The object
			boost::shared_ptr<shapefile> _shp;
			EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		protected:
			shapeFileStatsBase();
			bool _valid;

			friend class ::boost::serialization::access;
			template<class Archive>
			void serialize(Archive & ar, const unsigned int version);
			
		};

		class shapeFileStats : public shapeFileStatsBase
		{
		public:
			shapeFileStats();
			shapeFileStats(const shapefile &shp);
			shapeFileStats(const boost::shared_ptr<const shapefile> &shp);
		public:
			static boost::shared_ptr<shapeFileStats> genStats(
				const std::string &shpfile, const std::string &statsfile = "");
			static void doQhull(bool);
			static bool doQhull();

			friend class ::boost::serialization::access;
			template<class Archive>
			void serialize(Archive & ar, const unsigned int version);
			EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		};

	}
}
