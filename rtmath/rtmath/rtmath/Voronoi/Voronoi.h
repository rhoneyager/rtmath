#pragma once

#include <functional>
#include <map>
#include <string>

#include "../Serialization/serialization_macros.h"
#include "../Serialization/eigen_serialization.h"
#include "../hash.h"
#include "../registry.h"
#include <boost/shared_ptr.hpp>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/split_free.hpp>
#include <boost/serialization/export.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/version.hpp>
#include <boost/serialization/set.hpp>
#include <boost/serialization/string.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>

namespace voro
{
	class container;
	class voronicell_neighbor;
	class c_loop_all;
}

namespace rtmath {
	namespace Voronoi {
		class VoronoiDiagram;
		class Voronoi_IO_output_registry{};
	}
	namespace registry {
		extern template struct IO_class_registry<::rtmath::Voronoi::VoronoiDiagram>;

		extern template class usesDLLregistry<
			::rtmath::Voronoi::Voronoi_IO_output_registry,
			IO_class_registry<::rtmath::Voronoi::VoronoiDiagram> >;
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
			::rtmath::Voronoi::Voronoi_IO_output_registry,
			::rtmath::registry::IO_class_registry<::rtmath::Voronoi::VoronoiDiagram> >
		{
		private:
			VoronoiDiagram();
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
			mutable boost::shared_ptr<CachedVoronoi> precalced;

			/// The Eigen source object.
			boost::shared_ptr<const Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> > src;
			/// Derived matrices from Voronoi-based algorithms. Results get stored / read from here.
			mutable std::map<std::string, Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> > results;

			HASH_t hash;
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




			/// \brief Calculate the depth from the surface of each cell, and output 
			/// as an Eigen::Matrix, following the initial point indices.
			const Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>& 
				calcSurfaceDepth() const;

			/// Calculate candidate convex hull points (used in max diameter calculations)
			const Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>& 
				calcCandidateConvexHullPoints() const;

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
				Eigen::Array3f &mins, Eigen::Array3f &maxs,
				Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> points
				);

			// Load a Voronoi diagram from a given hash
			//static boost::shared_ptr<VoronoiDiagram> loadHash(HASH_t hash);

			friend class ::boost::serialization::access;
			template<class Archive>
			void serialize(Archive & ar, const unsigned int version);
		};
	}
}
