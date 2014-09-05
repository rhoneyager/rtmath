#pragma once

#include <functional>
#include <map>
#include <string>
#include <boost/shared_ptr.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>

#include "../hash.h"
#include "../registry.h"
#include "../io.h"



namespace rtmath {
	namespace Voronoi {
		class VoronoiDiagram;
		class Voronoi_IO_output_registry {};
		class Voronoi_IO_input_registry {};
		class Voronoi_provider_registry {};

		/// Designed to be a singleton
		class DLEXPORT_rtmath_voronoi Voronoi_provider
		{
			Voronoi_provider();
		public:
			~Voronoi_provider();
			typedef std::function<boost::shared_ptr<VoronoiDiagram>
				(const Eigen::Array3f &, const Eigen::Array3f &,
				const Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>&)> voronoiStdGeneratorType;
			voronoiStdGeneratorType generator;
			typedef std::function<boost::shared_ptr<VoronoiDiagram>()> voronoiBlankGeneratorType;
			voronoiBlankGeneratorType voronoiBlankGenerator;

			typedef std::function<boost::shared_ptr<VoronoiDiagram>(boost::shared_ptr<VoronoiDiagram>)> 
				voronoiUpcastGeneratorType;
			voronoiUpcastGeneratorType voronoiUpcastGenerator;

			const char* name;
			//virtual boost::shared_ptr<VoronoiDiagram> generate(
			//	const Eigen::Array3f &mins, const Eigen::Array3f &maxs,
			//	const Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>& points
			//	) = 0;
		};
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

		extern template class usesDLLregistry<
			::rtmath::Voronoi::Voronoi_provider_registry,
			::rtmath::Voronoi::Voronoi_provider >;
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
			public boost::enable_shared_from_this<VoronoiDiagram>,
			virtual public ::rtmath::registry::usesDLLregistry<
				::rtmath::Voronoi::Voronoi_IO_input_registry,
				::rtmath::registry::IO_class_registry_reader<::rtmath::Voronoi::VoronoiDiagram> >,
			virtual public ::rtmath::registry::usesDLLregistry<
				::rtmath::Voronoi::Voronoi_IO_output_registry,
				::rtmath::registry::IO_class_registry_writer<::rtmath::Voronoi::VoronoiDiagram> >,
			virtual public ::rtmath::io::implementsStandardWriter<VoronoiDiagram, Voronoi_IO_output_registry>,
			virtual public ::rtmath::io::implementsStandardReader<VoronoiDiagram, Voronoi_IO_input_registry>,
			virtual public ::rtmath::registry::usesDLLregistry<
				Voronoi_provider_registry, Voronoi_provider >
		{
		public:
			typedef boost::shared_ptr<const Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> > matrixType;
			typedef boost::shared_ptr<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> > matrixTypeMutable;
		public:
			/// The Eigen source object.
			matrixType src;
			/// Derived matrices from Voronoi-based algorithms. Results get stored / read from here.
			mutable std::map<std::string, matrixType > results;
			/// Container for cached Voronoi diagram results. CachedVoronoi is the common base class.
			mutable std::map<std::string, boost::shared_ptr<CachedVoronoi> > cache;

			HASH_t _hash;
			/// Reconstructs the Voronoi diagram (when constructing, or when restored from serialization)
			virtual void regenerateVoronoi() const;
			/// Recalculates all cells in the Voronoi diagram
			virtual void regenerateFull() const;
			Eigen::Array3f mins;
			Eigen::Array3f maxs;
		private:
			void upcast(const char* pluginid = "") const;
		public:
			/// \note Constructor needs to be public for io standard reader (for now)
			VoronoiDiagram();
			~VoronoiDiagram() {}

			/// Forcibly set the hash to a given value (can be used to match to a shape or stats)
			void setHash(HASH_t hash);
			HASH_t hash() const;

			/// When calculated
			std::string ingest_timestamp;
			/// The system that the calculation was on
			std::string hostname;
			/// The user account that performed the calculation
			std::string ingest_username;
			/// Revision of the rtmath code for ingest
			int ingest_rtmath_version;
			/// Plugin id of the generator (used in upcasting)
			std::string pluginId;

			virtual void getResultsTable(std::map<std::string, matrixType> &res) const;

			virtual const Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic>* getCellMap() const;
			
			virtual void getBounds(Eigen::Array3f &mins, Eigen::Array3f &maxs, Eigen::Array3f &span) const;
			virtual Eigen::Array3i getSpan() const;
			virtual size_t numPoints() const;
			
			// Takes the output of another function and creates a solid 'hull' for display.
			//matrixType hullPts(const matrixType &src);

			/// \brief Calculate the depth from the surface of each cell, and output 
			/// as an Eigen::Matrix, following the initial point indices.
			virtual matrixType calcSurfaceDepth() const;

			/// \brief Calculate the depth from the surface of each cell, and output 
			/// as an Eigen::Matrix, following the initial point indices.
			virtual matrixType calcSurfaceDepthVectors() const;

			/// \brief Calculate the number of neighbors for each point.
			virtual matrixType calcSurfaceNumNeighs() const;

			/// \brief Debug function to display the filling order of the vertices.
			virtual matrixType calcSurfaceFillingOrder() const;

			/// Calculate candidate convex hull points (used in max diameter calculations).
			virtual matrixType calcCandidateConvexHullPoints() const;

			/// \brief Calculate the external surface area fraction of all 
			/// points.
			virtual matrixType calcPointsSAfracExternal() const;


			/// Calculate the surface area of the bulk figure
			virtual double surfaceArea() const;
			/// Calculate the volume of the bulk figure
			virtual double volume() const;
			/// Calculate projective area of the figure intersecting the specified plane

			/// \brief Generate standard Voronoi diagram, with cells starting with pre-contoured size
			/// \todo Add points shared_ptr overload.
			/// \note pluginid cannot be defaulted because shapefile has the generateVoronoi function.
			static boost::shared_ptr<VoronoiDiagram> generateStandard(
				const Eigen::Array3f &mins, const Eigen::Array3f &maxs,
				const Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>& points,
				const char* pluginid
				);

			/// Used only by the hdf readers and io storage plugins.
			/// \param is the plugin id of an object to be read. Handles the cast.
			static boost::shared_ptr<VoronoiDiagram> generateBlank(const char* pluginid = "");

			// Load a Voronoi diagram from a given hash
			//static boost::shared_ptr<VoronoiDiagram> loadHash(HASH_t hash);

			/// Convenience functions to load shape based on hash
			/// \throws rtmath::debug::xMissingFile if the hashed shape is not found
			static boost::shared_ptr<VoronoiDiagram> loadHash(
				const HASH_t &hash);
			/// Convenience functions to load shape based on hash
			/// \throws rtmath::debug::xMissingFile if the hashed shape is not found
			static boost::shared_ptr<VoronoiDiagram> loadHash(
				const std::string &hash);
			/// Write to the hash directory (convenience function)
			virtual void writeToHash() const;
		};
	}
}
