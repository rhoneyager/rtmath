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

		template struct IO_class_registry_writer
			< ::rtmath::ddscat::points::sphereVol > ;

		template class usesDLLregistry <
			::rtmath::ddscat::points::sphereVol_IO_output_registry,
			IO_class_registry_writer<::rtmath::ddscat::points::sphereVol> > ;

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
				const boost::shared_ptr<const ::rtmath::ddscat::shapefile::shapefile>,
				float radius,
				boost::shared_ptr<const points> src)
			{
				backend_index_type t;
				backend_scalar_type u; // Will get set, but is unused.
				// Report the number of points in a radius around the target cell
				// Target cell specified in ci, radius is radius
				size_t num = src->neighborSearchRadius(
					radius*radius, ci.x, ci.y, ci.z, t, u);
				return num;
			}

			sphereVol::~sphereVol() {}
			sphereVol::sphereVol() {}
			sphereVol::pType sphereVol::getData() const { return data; }
			double sphereVol::volSphere() const { return vol; }
			double sphereVol::radius() const { return rad; }
			int sphereVol::pointsInSphere() const { return ps; }
			boost::shared_ptr<const sphereVol> sphereVol::generate(double r) {
				boost::shared_ptr<sphereVol> res(new sphereVol);
				res->rad = r;
				const double pi = boost::math::constants::pi<double>();
				res->vol = std::pow(res->rad,3.) * 4. * pi / 3.;

				int nd = (int)(2.*(res->rad)) + 3;
				boost::shared_ptr<matType> mat(new matType);
				mat->resize(nd*nd*nd,4);
				mat->setZero();
				for (int i=0; i < mat->rows(); ++i) {
					// Determine coordinates
					// Start with x, y, z = -rad.
					// Increment first in x, then y, then z.
					auto &x = (*mat)(i,0), &y = (*mat)(i,1),
						 &z = (*mat)(i,2), &v = (*mat)(i,3);
					x = (-(int)(res->rad)-1) + (i % nd);
					y = (-(int)(res->rad)-1) + ((i / nd) % nd);
					z = (-(int)(res->rad)-1) + (i / (nd*nd));
					// Determine if point is within or on the sphere
					int resq = (x*x) + (y*y) + (z*z);
					double rsq = std::pow(res->rad + 0.001,2.);
					if (resq < rsq) v = 1;
					//cout << "x " << x << " y " << y << " z "
					//	<< z << " resq " << resq << " rsq " << rsq
					//	<< " v " << v << endl;
				}
				res->ps = mat->block(0,3,mat->rows(),1).sum();
				res->data = mat;

				return res;
			}
		}
	}
}
