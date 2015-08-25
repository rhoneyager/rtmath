#pragma once
#include "../defs.h"
#include <vector>
#include <map>
#include <set>
#include <boost/tuple/tuple.hpp>
#include <Eigen/Core>
#include <Eigen/StdVector>
#include <Eigen/Dense>
#include <boost/shared_ptr.hpp>
//#include <boost/program_options.hpp>

#include "shapefile.h"
#include <Ryan_Debug/io.h>

// Forward declarations
namespace rtmath {
	namespace ddscat {
		namespace stats {
			class shapeFileStatsBase;
			class shapeFileStats;
			class shapeFileStats_IO_input_registry {};
			class shapeFileStats_IO_output_registry {};
			class DLEXPORT_rtmath_ddscat rotColDefs
			{
			public:
				enum rotDefs {
					// VERSION TAG ADDED IN VERSION 5
					BETA, THETA, PHI, VERSION,
					NUM_ROTDEFS_FLOAT
				};
				enum vectorDefs {
					MIN, MAX, SUM, SKEWNESS, KURTOSIS, MOM1, MOM2,
					PE, AREA_CONVEX, PERIMETER_CONVEX,
					ABS_MIN, ABS_MAX, ABS_MEAN, RMS_MEAN,
					NUM_VECTORDEFS
				};
				enum matrixDefs {
					AS_ABS, AS_ABS_MEAN, AS_RMS, MOMINERT, COVARIANCE,
					NUM_MATRIXDEFS
				};
				static std::string stringifyBasic(int);
				static std::string stringifyVector(int);
				static std::string stringifyMatrix(int);
			};
			typedef std::array<float, rotColDefs::NUM_ROTDEFS_FLOAT> basicTable;
			typedef Eigen::Matrix3f matType;
			typedef Eigen::Vector4f vecType;
			typedef std::array<matType, rotColDefs::NUM_MATRIXDEFS> matrixTable;
			typedef std::array<vecType, rotColDefs::NUM_VECTORDEFS> vectorTable;
			// coordinates, basic table, moments of inertia, covariances
			typedef boost::tuple<basicTable, matrixTable, vectorTable> rotData;


		}
	}
	namespace Voronoi { class VoronoiDiagram; }
}

namespace std {
	/// \note specializing a STRUCT, so afaik this has to occur in this header - cannot move to a .cpp file 
	template<>
	struct less<::rtmath::ddscat::stats::rotData>
	{
		bool operator()(const ::rtmath::ddscat::stats::rotData& lhs, const ::rtmath::ddscat::stats::rotData& rhs) const
		{
			using namespace ::rtmath::ddscat::stats;
			if (lhs.get<0>()[rotColDefs::BETA] != rhs.get<0>()[rotColDefs::BETA]) return lhs.get<0>()[rotColDefs::BETA] < rhs.get<0>()[rotColDefs::BETA];
			if (lhs.get<0>()[1] != rhs.get<0>()[rotColDefs::THETA]) return lhs.get<0>()[rotColDefs::THETA] < rhs.get<0>()[rotColDefs::THETA];
			if (lhs.get<0>()[2] != rhs.get<0>()[rotColDefs::PHI]) return lhs.get<0>()[rotColDefs::PHI] < rhs.get<0>()[rotColDefs::PHI];

			return false;
		}
	};
}

namespace Ryan_Debug {
	namespace registry {
		
		extern template struct IO_class_registry_writer<
			::rtmath::ddscat::stats::shapeFileStats>;

		extern template struct IO_class_registry_reader<
			::rtmath::ddscat::stats::shapeFileStats>;

		extern template class usesDLLregistry<
			::rtmath::ddscat::stats::shapeFileStats_IO_input_registry,
			IO_class_registry_reader<::rtmath::ddscat::stats::shapeFileStats> >;

		extern template class usesDLLregistry<
			::rtmath::ddscat::stats::shapeFileStats_IO_output_registry,
			IO_class_registry_writer<::rtmath::ddscat::stats::shapeFileStats> >;
		
	}
}

namespace boost { namespace program_options { 
	class options_description; class variables_map; } }
namespace boost { namespace filesystem { class path; } }

namespace rtmath {
	namespace ddscat {
		class rotations;
		namespace stats {


			
			/**
			* Version history for shapeFileStats, shapeFileStatsRotated and base classes:
			*
			* 7 - Fixed max_diameter calculation. Error caused by a bug in VTK. Affects Scircum.
			* 6 - Ellipsoid_max adds SA. Adding Holly AR ellipsoid_max.
			* 5 - Adding rms sphere and radius of gyration to stats
			* 4 - Adding Voronoi hulls, and subclassing V,SA and f values.
			* 3 - Bug fixes
			* 2 - removed qhull_enabled flag, as vtk convex hulls are now used
			* 1 - added qhull enabled / disabled flag when recording calculations
			* 0 - 
			**/
			class DLEXPORT_rtmath_ddscat shapeFileStatsBase
			{
			public:
				virtual ~shapeFileStatsBase();
				/// \brief Function that, if the shapefile referenced is not loaded, reloads the 
				/// shapefile. Required for hulling or stats adding operations.
				bool load();
				bool load(boost::shared_ptr<const shapefile::shapefile>);

				/// When the run was imported
				std::string ingest_timestamp;
				/// The system that the run was imported on
				std::string ingest_hostname;
				/// The user account that imported the run
				std::string ingest_username;
				/// Revision of the rtmath code for ingest
				int ingest_rtmath_version;

				/// rot is the effective rotation designated by the choice of a1 and a2
				Eigen::Matrix3f rot, invrot;
				float beta, theta, phi;

				// The constant multipliers! d is unknown!
				float V_cell_const, V_dipoles_const;
				float aeff_dipoles_const;

				/// Hold related data together
				/// \todo Rename aeff_* to rad_*.
				class volumetric
				{
				public:
					volumetric();
					// TODO: Fix names. aeff_* are really equivalent radii.
					float V, aeff_V, SA, aeff_SA, f;
					//float &f_V;
					//float f_SA;
					void calc(const shapeFileStatsBase*,
						std::function<std::pair<float,float>()>);
					void calc(const shapeFileStatsBase*);
				};

				/// Scircum_sphere is for a circumscribing sphere
				/// Srms_sphere is for a RMS sphere (P&H 2010 def.)
				/// SVoronoi_hull is for a voronoi hull
				/// Sellipsoid_max is for a max ellipsoid
				/// Sconvex_hull is for a convex hull
				/// Sgyration is for Westbrook 2006's radius of gyration
				/// Ssolid is for solid ice
				/// SVoronoi_internal_2 is for the internal voronoi hull
				volumetric Scircum_sphere, Sconvex_hull, Sgyration,
					SVoronoi_hull, Sellipsoid_max, Srms_sphere,
					Ssolid, SVoronoi_internal_2, Sellipsoid_max_Holly;
					//SCircum_circle_proj_x, SCircum_circle_proj_y,
					//SCircum_circle_proj_z, SCircum_ellipse_proj_x,
					//SCircum_ellipse_proj_y, SCircum_ellipse_proj_z,
					//Smean_circle_proj_x, Smean_circle_proj_y,
					//Smean_circle_proj_z, Sarea_circle_proj_x,
					//Sarea_circle_proj_y, Sarea_circle_proj_z; // , Sellipsoid_rms;

				/// Maximum distance between any two points
				float max_distance;
				/// Max distance 1st point
				//Eigen::Array3f md1;
				/// Max distance 2nd point

				/// Tracks the current stats version to see if recalculation is required.
				static const int _maxVersion;
				/// Tracks the loaded stats version to see if reaclculation is required.
				int _currVersion;

				// Before normalization and rotation
				Eigen::Vector3f b_min, b_max, b_mean;


				
				mutable std::set<rotData> rotstats;
				typedef std::set<rotData>::const_iterator rotPtr;
				//typedef boost::shared_ptr<const shapeFileStatsRotated> rotPtr;

				// Set rotation matrix, with each value in degrees
				//void setRot(double beta, double theta, double phi);
				void calcStatsBase();
				/// calcStatsRot calculates the stats RELATIVE to the shapefile default rot.
				rotPtr calcStatsRot(double beta, double theta, double phi) const;

				/** \brief Calculates the orientation with the minimum potential energy function.
				*
				* The orientations being considered are constrained on the 2d plane of beta and theta.
				* Stats are first calculated on a grid (orientations following defaults, modifiable by the
				* command-line) in an attempt to find multiple minima. Then, a minimization library is
				* called to find the true minimum. The resultant shared pointer is stored in a special
				* entry in the shapeFileStatsBase class.
				**/
				rotPtr calcOriMinPE() const;

				/** \brief Calculates the orientation with the greatest possible horizontal aspect ratio.
				*
				* This works by first considering the max distance between any two points, found during the
				* base stats calculation. The particle is rotated such that the max distance occurs along
				* the y axis (so that the particle major axis is perpendicular to incoming radiation in DDSCAT.
				* Additionally, the particle points are projected within this axis, and the particle is
				* furthermore rotated to ensure that the next greatest aspect ratio (following the projection
				* operation) lies along the z axis. The resultant shared pointer is stored in a special entry
				* in the shapeFileStatsBase class to avoid recalculation.
				**/
				rotPtr calcOriMaxAR() const;

				/// Calculate stats for each rotation to match a ddscat run's rotations
				void calcStatsRot(const rtmath::ddscat::rotations&) const;


				/// The shape
				boost::shared_ptr<const ::rtmath::ddscat::shapefile::shapefile> _shp;
				EIGEN_MAKE_ALIGNED_OPERATOR_NEW
			protected:
				shapeFileStatsBase();
				void calcStatsBaseRotMatrix();
				void calcVoroCvx();
				void calcBs();
				void calcSsolid();
				void calcScircum();
				void calcSellmax();
				void calcSellmaxHolly();
				void calcSrms_sphere();
				void calcSgyration();
				mutable rotPtr _rotMinPE, _rotMaxAR;

				/// Holds the Voronoi diagram (now used when calculating per-orientation area and perimeter)
				// Currently disabled because it makes stats storage huge! Per-ori vd use is currently
				// disabled, so it can be turned off. Otherwise, a cacheing shared pointer should be established, 
				// or the use of weak pointers with a parent that can drop the object.
				//boost::shared_ptr<rtmath::Voronoi::VoronoiDiagram> vd;


			};

			class DLEXPORT_rtmath_ddscat shapeFileStats 
				: public shapeFileStatsBase,
				virtual public ::Ryan_Debug::registry::usesDLLregistry<
				    shapeFileStats_IO_input_registry, 
					::Ryan_Debug::registry::IO_class_registry_reader<shapeFileStats> >,
					virtual public ::Ryan_Debug::registry::usesDLLregistry<
				    shapeFileStats_IO_output_registry, 
					::Ryan_Debug::registry::IO_class_registry_writer<shapeFileStats> >,
					virtual public ::Ryan_Debug::io::implementsStandardWriter<shapeFileStats, shapeFileStats_IO_output_registry>,
					virtual public ::Ryan_Debug::io::implementsStandardReader<shapeFileStats, shapeFileStats_IO_input_registry>//,
				//virtual public ::rtmath::io::Serialization::implementsSerialization<
				//	shapeFileStats, shapeFileStats_IO_output_registry, 
                //	shapeFileStats_IO_input_registry, shapeFileStats_serialization>
			{
			public:
				shapeFileStats();
				shapeFileStats(const ::rtmath::ddscat::shapefile::shapefile &shp);
				shapeFileStats(const boost::shared_ptr<const ::rtmath::ddscat::shapefile::shapefile> &shp);
				//static boost::shared_ptr<shapeFileStats> generate();
				/// Should the stats file be recalculated in the newest version?
				bool needsUpgrade() const;
				/// Recalculate all stats, using the newest version of the code
				void upgrade();
				// Write stats to file (convenience function to mask the Ryan_Serialization call)
				//void write(const std::string &filename, const std::string &type = "") const;
				// Write stats to the hash file (convenience function)
				void writeToHash() const;
				// Write or export to a complex, multiple storage object
				//std::shared_ptr<registry::IOhandler> writeMulti(
				//	std::shared_ptr<rtmath::registry::IOhandler> handle,
				//	std::shared_ptr<rtmath::registry::IO_options> opts) const;
				
				// Load stats from serialized file.
				// A convenience function that calls Ryan_Serialization
				//void read(const std::string &filename);
			private:
				void _init();
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
					const boost::shared_ptr<const shapefile::shapefile> &shp);

				/// Alias to loadHash
				static boost::shared_ptr<shapeFileStats> genStats(
					const Ryan_Debug::hash::HASH_t &hash);

				/**
				* \brief Adds shapestats options to a program
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
				/// Load stats based on hash
				/// \throws rtmath::debug::xMissingFile if the hashed stats not found
				static boost::shared_ptr<shapeFileStats> loadHash(
					const Ryan_Debug::hash::HASH_t &hash);
				/// Load stats based on hash
				/// \throws rtmath::debug::xMissingFile if the hashed stats not found
				static boost::shared_ptr<shapeFileStats> loadHash(
					const std::string &hash);
			public:
				EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
			};
		}
	}
}

