#include "Stdafx-voronoi.h"
#pragma warning( disable : 4996 )
#pragma warning( disable : 4305 )
#pragma warning( disable : 4244 )

#include <algorithm>
#include <boost/shared_ptr.hpp>
#include <boost/math/constants/constants.hpp>

#include <Eigen/Dense>

#include "../rtmath/ddscat/rotations.h"
#include "../rtmath/ddscat/hulls.h"
#include "../rtmath/error/error.h"

namespace rtmath
{
	namespace registry {
		//template struct IO_class_registry_writer
		//	<::rtmath::Voronoi::VoronoiDiagram>;
		//template class usesDLLregistry<
		//	::rtmath::Voronoi::Voronoi_IO_output_registry,
		//	IO_class_registry_writer<::rtmath::Voronoi::VoronoiDiagram> >;

		
		template class usesDLLregistry<
			::rtmath::ddscat::hull_provider_registry,
			::rtmath::ddscat::hull_provider<::rtmath::ddscat::convexHull> >;
	}

	namespace ddscat
	{
		
		hull::hull()
		{
		}

		hull::hull(boost::shared_ptr< const Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> > backend)
		{
		}

		convexHull::convexHull(boost::shared_ptr< const Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> >  src) : hull(src)
		{
		}
		
		boost::shared_ptr<convexHull> convexHull::generate
			(boost::shared_ptr< const Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> > backend)
		{
			auto hooks = ::rtmath::registry::usesDLLregistry<hull_provider_registry, hull_provider<convexHull> >::getHooks();
			for (const auto &h : *(hooks.get()))
			{
				if (!h.generator) continue;
				return h.generator(backend);
			}
			return nullptr;
		}

	}
}

