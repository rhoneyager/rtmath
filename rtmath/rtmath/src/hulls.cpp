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
#include <Ryan_Debug/error.h>
#include <Ryan_Debug/logging.h>
#include <Ryan_Debug/logging_base.h>
#include <boost/log/sources/global_logger_storage.hpp>


namespace {
	BOOST_LOG_INLINE_GLOBAL_LOGGER_CTOR_ARGS(
		m_hull,
		boost::log::sources::severity_channel_logger_mt< >,
		(boost::log::keywords::severity = Ryan_Debug::log::error)(boost::log::keywords::channel = "hull"));
}

namespace Ryan_Debug
{
	namespace registry {
		//template struct IO_class_registry_writer
		//	<::rtmath::Voronoi::VoronoiDiagram>;
		//template class usesDLLregistry<
		//	::rtmath::Voronoi::Voronoi_IO_output_registry,
		//	IO_class_registry_writer<::rtmath::Voronoi::VoronoiDiagram> >;


		template class usesDLLregistry <
			::rtmath::ddscat::hull_provider_registry,
			::rtmath::ddscat::hull_provider<::rtmath::ddscat::convexHull> > ;
	}
}
namespace rtmath {
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
			(boost::shared_ptr< const Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> > backend,
			 const char* defaultPlugin)
		{
			auto& lg = m_hull::get();
			BOOST_LOG_SEV(lg, Ryan_Debug::log::normal) << "Generating convex hull for backend " << &backend;

			auto hooks = ::Ryan_Debug::registry::usesDLLregistry<hull_provider_registry, hull_provider<convexHull> >::getHooks();
			//std::cerr << hooks->size() << std::endl;
			// Two passes, in case a default plugin is specified.
			if (defaultPlugin) {
				for (const auto &h : *(hooks.get()))
				{
					if (!h.generator) continue;
					if (!h.name) continue;
					std::string sname(h.name), pname(defaultPlugin);
					if (sname == pname) {
						BOOST_LOG_SEV(lg, Ryan_Debug::log::normal)
							<< "Selected " << h.name;
						return h.generator(backend);
					}
				}
			}
			for (const auto &h : *(hooks.get()))
			{
				if (!h.generator) continue;
				if (h.name)
					BOOST_LOG_SEV(lg, Ryan_Debug::log::normal)
						<< "Selected " << h.name;
				else BOOST_LOG_SEV(lg, Ryan_Debug::log::normal)
					 << "Selected unnamed plugin";

				return h.generator(backend);
			}
			// Only return nullptr if unable to find a usable hook.
			BOOST_LOG_SEV(lg, Ryan_Debug::log::error) << "No registered handler for hull calculation found. Throwing error.";
			RDthrow(Ryan_Debug::error::xUpcast())
				<< Ryan_Debug::error::otherErrorText("No registered handler for hull calculation found. Throwing error.");
			return nullptr;
		}

	}
}

