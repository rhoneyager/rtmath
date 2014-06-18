#pragma once

#include <functional>
#include <map>
#include <string>
#include <boost/shared_ptr.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>

#include "../hash.h"
#include "../registry.h"
#include "../io.h"


namespace voro
{
	class container;
	class voronicell_neighbor;
	class c_loop_all;
}

namespace rtmath {
	namespace Voronoi {
		class VoronoiDiagram;
		class Voronoi_IO_output_registry {};
		class Voronoi_IO_input_registry {};
	}
	namespace registry {
		extern template struct IO_class_registry_writer<::rtmath::Voronoi::VoronoiDiagram>;
		extern template struct IO_class_registry_reader<::rtmath::Voronoi::VoronoiDiagram>;
		extern template class usesDLLregistry<
			::rtmath::Voronoi::Voronoi_IO_input_registry,
			IO_class_registry_reader<::rtmath::Voronoi::VoronoiDiagram> >;
		extern template class usesDLLregistry<
			::rtmath::Voronoi::Voronoi_IO_output_registry,
			IO_class_registry_writer<::rtmath::Voronoi::VoronoiDiagram> >;
	}

	namespace Voronoi
	{
		
		class CachedVoronoi;
		

		/** \brief Encapsulators + generators for Voronoi objects.
		*
		* These objects represent various Voronoi diagrams (voro++ library), usually used in
		* conjunction with shapefiles in the rtmath-ddscat library, and in
		* calculating their associated statistics (like max distance between
		* two points). Another use is in determining the 'surface' of a given
		* flake. This encapsulation is used because, in many cases, the same
		* Voronoi diagram is used in multiple parts of the program. However, it is
		* not normally written out. This encapsulator provides a read/write facility
		* for such diagrams and allows the same calculation to be reused in
		* different code locations. It also provides a 'placeholder' functionality,
		* where the diagram may be lazily evaluated upon usage, and a preexisting
		* calculation may be loaded from disk based on the rtmath standard hashing
		* functions.
		*
		* \todo Add storage and export of connectivity diagrams (depGraph vertices)
		* \todo Add io functions (saving / loading hash, better serialization, silo output)
		**/
		class DLEXPORT_rtmath_voronoi VoronoiDiagram :
			virtual public ::rtmath::registry::usesDLLregistry<
				::rtmath::Voronoi::Voronoi_IO_input_registry,
				::rtmath::registry::IO_class_registry_reader<::rtmath::Voronoi::VoronoiDiagram> >,
			virtual public ::rtmath::registry::usesDLLregistry<
				::rtmath::Voronoi::Voronoi_IO_output_registry,
				::rtmath::registry::IO_class_registry_writer<::rtmath::Voronoi::VoronoiDiagram> >,
			virtual public ::rtmath::io::implementsStandardWriter<VoronoiDiagram, Voronoi_IO_output_registry>,
			virtual public ::rtmath::io::implementsStandardReader<VoronoiDiagram, Voronoi_IO_input_registry>
		{
		public:
			typedef boost::shared_ptr<const Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> > matrixType;
			typedef boost::shared_ptr<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> > matrixTypeMutable;
			VoronoiDiagram();
		public:
			/**
			* \brief Internal pointer to the Voro++ object
			*
			* This object contains the actual Voronoi diagram calculation.
			* It does not persist in serialization and so needs to be recalculated each 
			* time a Voronoi object is loaded.
			**/
			mutable boost::shared_ptr<voro::container> vc;
			/// \brief Internal pointer to the internal Voronoi cache
			/// \todo Move the internal Voro++ object to within here
			/// \todo Add precalced object serialization
			mutable std::map<std::string, boost::shared_ptr<CachedVoronoi> > cache;
			//mutable boost::shared_ptr<CachedVoronoi> precalced, precalcedSmall;

			/// The Eigen source object.
			matrixType src;
			/// Derived matrices from Voronoi-based algorithms. Results get stored / read from here.
			mutable std::map<std::string, matrixType > results;

			HASH_t _hash;
			/// Reconstructs the Voronoi diagram (when constructing, or when restored from serialization)
			void regenerateVoronoi() const;
			/// Recalculates all cells in the Voronoi diagram
			void regenerateFull() const;
			Eigen::Array3f mins;
			Eigen::Array3f maxs;
		public:
			~VoronoiDiagram() {}

			/// Forcibly set the hash to a given value (can be used to match to a shape or stats)
			void setHash(HASH_t hash);
			HASH_t hash() const;

			/// When calculated
			std::string ingest_timestamp;
			/// The system that the calculation was on
			std::string ingest_hostname;
			/// The user account that performed the calculation
			std::string ingest_username;
			/// The host that performed the calculation
			std::string hostname;
			/// Revision of the rtmath code for ingest
			int ingest_rtmath_version;

			void getResultsTable(std::map<std::string, matrixType> &res) const;

			const Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic>* getCellMap() const;
			
			void getBounds(Eigen::Array3f &mins, Eigen::Array3f &maxs, Eigen::Array3f &span) const;
			Eigen::Array3i getSpan() const;
			size_t numPoints() const;
			
			// Takes the output of another function and creates a solid 'hull' for display.
			//matrixType hullPts(const matrixType &src);

			/// \brief Calculate the depth from the surface of each cell, and output 
			/// as an Eigen::Matrix, following the initial point indices.
			matrixType calcSurfaceDepth() const;

			/// \brief Calculate the depth from the surface of each cell, and output 
			/// as an Eigen::Matrix, following the initial point indices.
			matrixType calcSurfaceDepthVectors() const;

			/// \brief Calculate the number of neighbors for each point.
			matrixType calcSurfaceNumNeighs() const;

			/// \brief Debug function to display the filling order of the vertices.
			matrixType calcSurfaceFillingOrder() const;

			/// Calculate candidate convex hull points (used in max diameter calculations).
			matrixType calcCandidateConvexHullPoints() const;

			/// \brief Calculate the external surface area fraction of all 
			/// points.
			matrixType calcPointsSAfracExternal() const;


			/// Calculate the surface area of the bulk figure
			double surfaceArea() const;
			/// Calculate the volume of the bulk figure
			double volume() const;
			/// Calculate projective area of the figure intersecting the specified plane



			/// Save the Voronoi diagram
			//void writeFile(const std::string &filename, const std::string type = "") const;

			// Save the voronoi diagram to the given hash
			//void writeToHash(HASH_t hash) const;
			//inline void writeToHash() const { writeToHash(hash); }


			/// \brief Generate standard Voronoi diagram, with cells starting with pre-contoured size
			/// \todo Add points shared_ptr overload.
			static boost::shared_ptr<VoronoiDiagram> generateStandard(
				const Eigen::Array3f &mins, const Eigen::Array3f &maxs,
				const Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>& points
				);

			// Load a Voronoi diagram from a given hash
			//static boost::shared_ptr<VoronoiDiagram> loadHash(HASH_t hash);

		};
	}
}
