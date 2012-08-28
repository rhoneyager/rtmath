#pragma once
#include <boost/archive/xml_oarchive.hpp>
#include <boost/archive/xml_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/export.hpp>
#include <boost/serialization/shared_ptr.hpp>
#include <boost/serialization/split_member.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/version.hpp>

#include "matrixop_serialization.h"

#include "../ddscat/shapefile.h"

namespace boost
{
	namespace serialization
	{
		template <class Archive>
		void serialize(Archive & ar, rtmath::ddscat::shapefile & g, const unsigned int version)
		{
					ar & boost::serialization::make_nvp("filename", _filename);
					ar & boost::serialization::make_nvp("N", _numPoints);
					ar & boost::serialization::make_nvp("Dielectrics", _Dielectrics);
					ar & boost::serialization::make_nvp("a1", _a1);
					ar & boost::serialization::make_nvp("a2", _a2);
					ar & boost::serialization::make_nvp("a3", _a3);
					ar & boost::serialization::make_nvp("d", _d);
					ar & boost::serialization::make_nvp("x0", _x0);
					ar & boost::serialization::make_nvp("xd", _xd);
		}
	}
}


//BOOST_CLASS_EXPORT_KEY(rtmath::ddscat::shapefile)
