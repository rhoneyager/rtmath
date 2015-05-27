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
#include <boost/log/sources/global_logger_storage.hpp>


namespace {
	BOOST_LOG_INLINE_GLOBAL_LOGGER_CTOR_ARGS(
		m_hull,
		blog::sources::severity_channel_logger_mt< >,
		(blog::keywords::severity = ::rtmath::debug::error)(blog::keywords::channel = "hull"));
}

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
			auto& lg = m_hull::get();
			BOOST_LOG_SEV(lg, rtmath::debug::normal) << "Generating convex hull for backend " << &backend;

			auto hooks = ::rtmath::registry::usesDLLregistry<hull_provider_registry, hull_provider<convexHull> >::getHooks();
			//std::cerr << hooks->size() << std::endl;
			for (const auto &h : *(hooks.get()))
			{
				if (!h.generator) continue;
				return h.generator(backend);
			}
			// Only return nullptr if unable to find a usable hook.
			BOOST_LOG_SEV(lg, rtmath::debug::error) << "No registered handler for hull calculation found. Throwing error.";
			std::cerr << "Ping" << std::endl;
			RDthrow(rtmath::debug::xUpcast());
			return nullptr;
		}

	}
}

