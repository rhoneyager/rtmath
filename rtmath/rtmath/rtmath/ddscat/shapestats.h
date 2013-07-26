#pragma once
#include <vector>
#include <map>
#include <set>
#include <Eigen/Core>
#include <Eigen/StdVector>
#include <Eigen/Dense>
#include <boost/shared_ptr.hpp>
#include <boost/serialization/version.hpp>
//#include <boost/program_options.hpp>

#include "shapefile.h"
#include "shapestatsRotated.h"

// Forward declarations
namespace rtmath {
	namespace ddscat {
		class shapeFileStatsBase;
		class shapeFileStats;
	}
}

namespace boost { namespace program_options { 
	class options_description; class variables_map; } }
namespace boost { namespace filesystem { class path; } }

namespace rtmath {
	namespace ddscat {
		struct rotComp
		{
			bool operator()(const boost::shared_ptr<const shapeFileStatsRotated> &lhs,
				const boost::shared_ptr<const shapeFileStatsRotated> &rhs) const;
		};

		/*
		 * Version history:
		 * 2 - removed qhull_enabled flag, as vtk convex hulls are now used
		 * 1 - added qhull enabled / disabled flag when recording calculations
		 * 0 - 
		 */
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

			// Before normalization and rotation
			Eigen::Vector3f b_min, b_max, b_mean;

			std::set<boost::shared_ptr<const shapeFileStatsRotated>, rotComp > rotations;

			// The object
			boost::shared_ptr<shapefile> _shp;
			EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		protected:
			shapeFileStatsBase();

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
			/// Should the stats file be recalculated in the newest version?
			bool needsUpgrade() const;
			/// Recalculate all stats, using the newest version of the code
			void upgrade();
			/// Write stats to file (convenience function to mask the Ryan_Serialization call)
			void write(const std::string &filename) const;
			/// Write stats to the hash file (convenience function)
			void writeToHash() const;
			/// Load stats from serialized file.
			/// A convenience function that calls Ryan_Serialization
			void read(const std::string &src);
		private:
			/// Gets some initial path information from rtmath.conf
			static void initPaths();
		public:
			/// \brief Generate shapefile stats for the given shape. Optionally write them to statsfile.
			///
			/// \note Reads and writes to hash database for precomputed stats
			static boost::shared_ptr<shapeFileStats> genStats(
				const std::string &shpfile, const std::string &statsfile = "");
			/// \brief Generate / load shapefile stats for the given shape.
			///
			/// \note Reads and writes to hash database for precomputed stats
			static boost::shared_ptr<shapeFileStats> genStats(
				const boost::shared_ptr<shapefile> &shp);

			/**
			 * \brief Adds passes shapestats options to a program
			 *
			 * \item cmdline provides options only allowed on the command line
			 * \item config provides options available on the command line and in a config file
			 * \item hidden provides options allowed anywhere, but are not displayed to the user
			 **/
			static void add_options(
				boost::program_options::options_description &cmdline,
				boost::program_options::options_description &config,
				boost::program_options::options_description &hidden);
			/// Processes static options defined in add_options
			/// \todo Add processor for non-static options
			static void process_static_options(
				boost::program_options::variables_map &vm);
			/**
			 * \brief Retrieve the base hash paths
			 *
			 * \item pHashShapes is the base shape hash directory
			 * \item pHashStats is the base stats hash directory
			 **/
			static void getHashPaths(
				boost::filesystem::path &pHashShapes,
				boost::filesystem::path &pHashStats);

			/// Load stats based on hash
			/// \throws rtmath::debug::xMissingFile if the hashed stats not found
			static boost::shared_ptr<shapeFileStats> loadHash(
				const HASH_t &hash);
			/// Load stats based on hash
			/// \throws rtmath::debug::xMissingFile if the hashed stats not found
			static boost::shared_ptr<shapeFileStats> loadHash(
				const std::string &hash);
		private:
			friend class ::boost::serialization::access;
			template<class Archive>
			void serialize(Archive & ar, const unsigned int version);
		public:
			EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
		};

	}
}

#define SHAPESTATS_VERSION 2
BOOST_CLASS_VERSION(rtmath::ddscat::shapeFileStatsBase, SHAPESTATS_VERSION);

