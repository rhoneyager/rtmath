#pragma once
#include "../defs.h"
#include <string>
#include <vector>
#include <set>
#include <boost/shared_ptr.hpp>
#include <Eigen/Core>
#include <Ryan_Debug/registry.h>
#include "shapefile.h"

/* This contains the necessary functions for doing point-based
 * nearest neighbor searches in arbitrary dimensions.
 */

namespace rtmath
{
	namespace ddscat
	{
		namespace points {
			class points;
			class points_IO_output_registry {};
			class points_provider_registry {};
			class sphereVol;
			class sphereVol_IO_output_registry {};
			typedef ::rtmath::ddscat::shapefile::shapefile::points_type
				backend_type;
			typedef ::rtmath::ddscat::shapefile::shapefile::index_type
				backend_index_type;
			typedef ::rtmath::ddscat::shapefile::shapefile::scalar_type
				backend_scalar_type;

			template <class hullType>
			struct DLEXPORT_rtmath_voronoi points_provider
			{
				typedef std::function <
					boost::shared_ptr<points>(backend_type)
				> pointsGenerator;
				pointsGenerator generator;

				const char* name;
			};
		}
	}
}
namespace Ryan_Debug {
	namespace registry
	{
		extern template class usesDLLregistry <
			::rtmath::ddscat::points::points_provider_registry,
			::rtmath::ddscat::points::points_provider<::rtmath::ddscat::points::points> > ;

		extern template struct IO_class_registry_writer <
			::rtmath::ddscat::points::sphereVol > ;

		extern template class usesDLLregistry <
			::rtmath::ddscat::points::sphereVol_IO_output_registry,
			IO_class_registry_writer<::rtmath::ddscat::points::sphereVol> > ;

	}
}
namespace rtmath {
	namespace ddscat
	{
		namespace shapefile { class shapefile; }
		namespace points {
			typedef boost::shared_ptr<const ::rtmath::ddscat::shapefile::shapefile> shape_t;
			class DLEXPORT_rtmath_voronoi points :
				virtual public ::Ryan_Debug::registry::usesDLLregistry<
				points_provider_registry, points_provider<points> >
			{
			protected:
				points(backend_type backend);
				points();
			public:
				/// Encapsulated in a static function to allow multiple hull generators to return cast objects to the 
				/// appropriate backends.
				static boost::shared_ptr<points> generate
					(backend_type backend);
				virtual ~points() {}

				/// \brief Get neighbors in a given search radius
				/// \returns the number of neighbors found in this radius
				/// \param radius  is the search distance (Euclidean)
				/// \param outpoints is an array expressing the locations
				/// of the found points.
				virtual size_t neighborSearchRadius(
					float radius,
					const Eigen::Array3f &pt,
					backend_index_type &outpoints,
					backend_scalar_type &outdistssq) const = 0;
				/// \brief Get nearest N neighbors
				/// \returns the number of neighbors found
				/// \param N is the number of neighbors to look for
				/// \param outpoints is an array expressing the locations
				/// of the located points.
				virtual size_t nearestNeighbors(
					size_t N,
					const Eigen::Array3f &pt,
					backend_index_type &outpoints,
					backend_scalar_type &outdistssq) const = 0;
				/// Construct the search tree, and populate
				virtual void constructTree() = 0;

				/// \brief Convolution function that fills a dielectric
				static size_t convolutionNeighborsRadius(
					const ::rtmath::ddscat::shapefile::convolutionCellInfo&,
					const shape_t,
					float radius,
					boost::shared_ptr<const points> src);

			};

			// Methods to convolve a shape

			/// Direct (forward) approach
			shape_t convolute_A(shape_t src, size_t kernelrad = 10);

			/// Indirect (reverse) approach
			shape_t convolute_B(shape_t src, size_t kernelrad = 10);

			/** \brief Provides an exact integer volume determination for
			 * reference for sphere-based searches
			 **/
			class DLEXPORT_rtmath_voronoi sphereVol :
				virtual public boost::enable_shared_from_this<sphereVol>,
				virtual public ::Ryan_Debug::registry::usesDLLregistry<
					::rtmath::ddscat::points::sphereVol_IO_output_registry,
					::Ryan_Debug::registry::IO_class_registry_writer<
						::rtmath::ddscat::points::sphereVol> >,
				virtual public ::Ryan_Debug::io::implementsStandardWriter
					<sphereVol, sphereVol_IO_output_registry>
			{
				sphereVol();
			public:
				static boost::shared_ptr<const sphereVol> generate(double radius);
				virtual ~sphereVol();
				int pointsInSphere() const;
				double volSphere() const;
				double radius() const;
				typedef Eigen::Array<int, Eigen::Dynamic, 4> matType;
				typedef boost::shared_ptr<const matType> pType;
				pType getData() const;
			private:
				double rad;
				int ps;
				double vol;
				pType data;
			};
		}
	}
}

