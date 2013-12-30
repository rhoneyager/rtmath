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
		//class VoronoiData; // The internal Voronoi object


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
			boost::shared_ptr<voro::container> vc;

			/// The standard Eigen source object.
			boost::shared_ptr<Eigen::Matrix<float, Eigen::Dynamic, 3> > src3;
			/// Eigen source object that is used when a radial weight is used with the points.
			boost::shared_ptr<Eigen::Matrix<float, Eigen::Dynamic, 4> > src4;
			/// Derived matrices from Voronoi-based algorithms. Results get stored / read from here.
			std::map<std::string, Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> > results;
			/// The other map store for scalar quantities
			std::map<std::string, float> results_scalar;
		public:
			~VoronoiDiagram() {}

			/// Forcibly set the hash to a given value (can be used to match to a shape or stats)
			void setHash(HASH_t hash);



			/// \brief Calculate the depth from the surface of each cell, and output 
			/// as an Eigen::Matrix, following the initial point indices.
			void writeSurfaceDepth(
				Eigen::Matrix<float, Eigen::Dynamic, 3> &depthOut) const;

			/// Calculate the surface area of the bulk figure

			/// Calculate the volume of the bulk figure

			/// Calculate projective area of the figure intersecting the specified plane



			/// Save the Voronoi diagram
			//void writeFile(const std::string &filename, const std::string type = "") const;

			/// Save the voronoi diagram to the given hash
			//void writeToHash(HASH_t hash) const;


			/// Generate standard Voronoi diagram, with cells starting with maximum size
			static boost::shared_ptr<VoronoiDiagram> generateStandard(
				float bounds[6],
				Eigen::Matrix<float, Eigen::Dynamic, 3> &points
				);

			/// Load a Voronoi diagram from a given hash
			//static boost::shared_ptr<VoronoiDiagram> loadHash(HASH_t hash);

			/// Generate Voronoi diagram, extracting surface points


			friend class ::boost::serialization::access;
			template<class Archive>
			void serialize(Archive & ar, const unsigned int version);
		};
	}
}
