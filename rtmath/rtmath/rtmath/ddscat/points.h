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
	}
}
namespace rtmath {
	namespace ddscat
	{
		namespace points {
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
					float x, float y, float z,
					backend_index_type &outpoints,
					backend_scalar_type &outdistssq) const = 0;
				/// \brief Get nearest N neighbors
				/// \returns the number of neighbors found
				/// \param N is the number of neighbors to look for
				/// \param outpoints is an array expressing the locations
				/// of the located points.
				virtual size_t nearestNeighbors(
					size_t N,
					float x, float y, float z,
					backend_index_type &outpoints,
					backend_scalar_type &outdistssq) const = 0;
				/// Construct the search tree, and populate
				virtual void constructTree() = 0;

				/// \brief Convolution function that fills a dielectric
				static size_t convolutionNeighborsRadius(
					const ::rtmath::ddscat::shapefile::convolutionCellInfo&,
					const boost::shared_ptr<const ::rtmath::ddscat::shapefile::shapefile>,
					float radius,
					boost::shared_ptr<const points> src);

			};
		}
	}
}

