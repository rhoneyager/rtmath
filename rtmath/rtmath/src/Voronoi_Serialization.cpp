#include "Stdafx-voronoi.h"
#include <Eigen/Core>
//#include "../rtmath/matrixop.h"
#include "../rtmath/Voronoi/Voronoi.h"

#include <boost/serialization/base_object.hpp>
#include <boost/serialization/export.hpp>
#include <boost/serialization/version.hpp>
#include <boost/serialization/map.hpp>
#include <boost/serialization/set.hpp>
#include <boost/serialization/string.hpp>
#include <boost/serialization/shared_ptr.hpp>

#include "../rtmath/Serialization/serialization_macros.h"
#include "../rtmath/Serialization/eigen_serialization.h"


namespace rtmath
{
	namespace Voronoi
	{
		/// \todo Add precalced object serialization
		template<class Archive>
		void VoronoiDiagram::serialize(Archive &ar, const unsigned int version)
		{
			ar & boost::serialization::make_nvp("hash", hash);
			ar & boost::serialization::make_nvp("Source", src);
			ar & boost::serialization::make_nvp("mins", mins);
			ar & boost::serialization::make_nvp("maxs", maxs);
			ar & boost::serialization::make_nvp("results", results);
		}

		EXPORTINTERNAL(rtmath::Voronoi::VoronoiDiagram::serialize);
	}
}

BOOST_CLASS_EXPORT_IMPLEMENT(rtmath::Voronoi::VoronoiDiagram);

