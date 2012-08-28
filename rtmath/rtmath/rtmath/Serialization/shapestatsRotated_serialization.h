#pragma once
#include <boost/archive/xml_oarchive.hpp>
#include <boost/archive/xml_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/serialization/access.hpp>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/map.hpp>
#include <boost/serialization/set.hpp>
#include <boost/serialization/shared_ptr.hpp>
#include <boost/serialization/version.hpp>

#include "../ddscat/shapestatsRotated.h"

#include "matrixop_serialization.h"
//#include "shapefile_serialization.h"

namespace boost
{
	namespace serialization
	{
		template <class Archive>
		void serialize(Archive & ar, rtmath::ddscat::shapeFileStatsRotated & g, const unsigned int version)
		{
					ar & BOOST_SERIALIZATION_NVP(g.beta);
					ar & BOOST_SERIALIZATION_NVP(g.theta);
					ar & BOOST_SERIALIZATION_NVP(g.phi);
					ar & BOOST_SERIALIZATION_NVP(g.min);
					ar & BOOST_SERIALIZATION_NVP(g.max);
					ar & BOOST_SERIALIZATION_NVP(g.sum);
					ar & BOOST_SERIALIZATION_NVP(g.covariance);
					ar & BOOST_SERIALIZATION_NVP(g.skewness);
					ar & BOOST_SERIALIZATION_NVP(g.kurtosis);
					ar & BOOST_SERIALIZATION_NVP(g.mom1);
					ar & BOOST_SERIALIZATION_NVP(g.mom2);
					ar & boost::serialization::make_nvp("Moment_Inertia", g.mominert);
					ar & BOOST_SERIALIZATION_NVP(g.PE);
					ar & BOOST_SERIALIZATION_NVP(g.abs_min);
					ar & BOOST_SERIALIZATION_NVP(g.abs_max);
					ar & BOOST_SERIALIZATION_NVP(g.abs_mean);
					ar & BOOST_SERIALIZATION_NVP(g.rms_mean);
					ar & BOOST_SERIALIZATION_NVP(g.as_abs);
					ar & BOOST_SERIALIZATION_NVP(g.as_abs_mean);
					ar & BOOST_SERIALIZATION_NVP(g.as_rms);
		}

	}
}

//BOOST_CLASS_VERSION(rtmath::ddscat::shapeFileStatsRotated, 0)
