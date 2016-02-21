#include "Stdafx-voronoi.h"
#pragma warning( disable : 4996 )
#pragma warning( disable : 4305 )
#pragma warning( disable : 4244 )

#include <algorithm>
#include <boost/shared_ptr.hpp>
#include <boost/math/constants/constants.hpp>

#include <Eigen/Dense>

#include "../rtmath/ddscat/rotations.h"
#include "../rtmath/ddscat/points.h"
#include <Ryan_Debug/error.h>
#include <Ryan_Debug/logging.h>
#include <Ryan_Debug/logging_base.h>
#include <boost/log/sources/global_logger_storage.hpp>


namespace {
	BOOST_LOG_INLINE_GLOBAL_LOGGER_CTOR_ARGS(
		m_points,
		boost::log::sources::severity_channel_logger_mt< >,
		(boost::log::keywords::severity = Ryan_Debug::log::error)(boost::log::keywords::channel = "points"));
}

namespace Ryan_Debug
{
	namespace registry {
		template class usesDLLregistry <
			::rtmath::ddscat::points::points_provider_registry,
			::rtmath::ddscat::points::points_provider<::rtmath::ddscat::points::points> > ;
	}
}
namespace rtmath {
	namespace ddscat
	{
		namespace points {
			points::points() {}

			points::points(backend_type backend) {} // Ignored, as points has no members.

			boost::shared_ptr<points> points::generate
				(backend_type backend)
			{
				auto& lg = m_points::get();
				BOOST_LOG_SEV(lg, Ryan_Debug::log::normal) << "Generating kd tree for backend " << &backend;

				auto hooks = ::Ryan_Debug::registry::usesDLLregistry<
					points_provider_registry, points_provider<points> >::getHooks();
				//std::cerr << hooks->size() << std::endl;
				for (const auto &h : *(hooks.get()))
				{
					if (!h.generator) continue;
					return h.generator(backend);
				}
				// Only return nullptr if unable to find a usable hook.
				BOOST_LOG_SEV(lg, Ryan_Debug::log::error) << "No registered handler for kd tree found. Throwing error.";
				RDthrow(Ryan_Debug::error::xUpcast())
					<< Ryan_Debug::error::otherErrorText("No registered handler for kd tree calculation found. Throwing error.");
				return nullptr;
			}

			size_t points::convolutionNeighborsRadius(
				const ::rtmath::ddscat::shapefile::convolutionCellInfo& ci,
				double radius,
				boost::shared_ptr<const points> src)
			{
				backend_type t; // Will get set, but is unused.
				// Report the number of points in a radius around the target cell
				// Target cell specified in ci, radius is radius
				size_t num = src->neighborSearchRadius(
					radius, ci.x, ci.y, ci.z, t);
				return num;
			}
		}
	}
}
