#pragma once
#include "../defs.h"
#include <string>
#include <vector>
#include <set>
#include <boost/shared_ptr.hpp>
#include <Eigen/Core>
#include <Ryan_Debug/registry.h>

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

			typedef boost::shared_ptr< const Eigen::Matrix<float,
				Eigen::Dynamic, Eigen::Dynamic> >
				backend_type;

			template <class hullType>
			struct DLEXPORT_rtmath_voronoi points_provider
			{
				typedef std::function <
					boost::shared_ptr<hullType>(backend_type)
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
				//boost::shared_ptr<hullData> _p;
			public:
				/// Encapsulated in a static function to allow multiple hull generators to return cast objects to the 
				/// appropriate backends.
				static boost::shared_ptr<points> generate
					(backend_type backend);

				virtual ~points() {}
				/// Hull volume
				virtual double volume() const = 0;
				/// Hull surface area
				virtual double surfaceArea() const = 0;
				/// Max distance between two points in hull
				virtual double maxDiameter() const = 0;
				/// Rotation coordinates to principle axes of hull
				virtual void principalAxes(double &beta, double &theta, double &phi) const = 0;

				/// Construct the kd tree, and populate
				virtual void constructTree() = 0;
			};
		}
	}
}

