#pragma once
#include <vector>
#include <map>
#include <set>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <boost/shared_ptr.hpp>
#include "shapefile.h"
#include "shapestatsRotated.h"

// Forward declaration for boost::serialization below
namespace rtmath {
	namespace ddscat {
		class shapeFileStatsBase;
		class shapeFileStats;
	}
}

// Need these so the template friends can work
namespace boost
{
	namespace serialization
	{
		template <class Archive>
		void serialize(Archive &, rtmath::ddscat::shapeFileStatsBase &, const unsigned int);
		template <class Archive>
		void serialize(Archive &, rtmath::ddscat::shapeFileStats &, const unsigned int);
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
			bool qhull_enabled;

			// Before normalization and rotation
			Eigen::Vector3f b_min, b_max, b_mean;

			std::set<boost::shared_ptr<const shapeFileStatsRotated>, rotComp > rotations;

			// The object
			boost::shared_ptr<shapefile> _shp;
		protected:
			shapeFileStatsBase();
			
			bool _valid;
			template<class Archive> 
			friend void ::boost::serialization::serialize(
				Archive &, shapeFileStatsBase &, const unsigned int);
			EIGEN_MAKE_ALIGNED_OPERATOR_NEW
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
		};

	}
}
