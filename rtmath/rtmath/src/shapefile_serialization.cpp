#include "../rtmath/Stdafx.h"
#include "../rtmath/matrixop.h"
#include "../rtmath/ddscat/shapefile.h"
#include "../rtmath/Serialization/matrixop_serialization.h"

#include "../rtmath/Serialization/serialization_macros.h"

#include <boost/serialization/base_object.hpp>
#include <boost/serialization/export.hpp>
#include <boost/serialization/version.hpp>

namespace boost
{
	namespace serialization
	{
		template<class Archive>
		void serialize(Archive &ar, rtmath::ddscat::shapefile &g, const unsigned int version)
		{
				ar & boost::serialization::make_nvp("filename", g._filename);
				ar & boost::serialization::make_nvp("N", g._numPoints);
				ar & boost::serialization::make_nvp("Dielectrics", g._Dielectrics);
				ar & boost::serialization::make_nvp("a1", g._a1);
				ar & boost::serialization::make_nvp("a2", g._a2);
				ar & boost::serialization::make_nvp("a3", g._a3);
				ar & boost::serialization::make_nvp("d", g._d);
				ar & boost::serialization::make_nvp("x0", g._x0);
				ar & boost::serialization::make_nvp("xd", g._xd);
		}

		EXPORT(rtmath::ddscat::shapefile);
	}
}

//BOOST_CLASS_EXPORT_IMPLEMENT(rtmath::ddscat::shapefile)

