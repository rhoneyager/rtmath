#include "Stdafx-ddscat.h"
#include "../rtmath/ddscat/rotations.h"
#include "../rtmath/Serialization/serialization_macros.h"

#include <boost/serialization/base_object.hpp>
#include <boost/serialization/export.hpp>
#include <boost/serialization/version.hpp>
#include <boost/serialization/set.hpp>

namespace rtmath
{
	namespace ddscat
	{
		template<class Archive>
		void rotationsBase::serialize(Archive &ar, const unsigned int version)
		{
				ar & boost::serialization::make_nvp("bMin", _bMin);
				ar & boost::serialization::make_nvp("bMax",_bMax);
				ar & boost::serialization::make_nvp("bN",_bN);
				ar & boost::serialization::make_nvp("tMin", _tMin);
				ar & boost::serialization::make_nvp("tMax", _tMax);
				ar & boost::serialization::make_nvp("tN", _tN);
				ar & boost::serialization::make_nvp("pMin",_pMin);
				ar & boost::serialization::make_nvp("pMax",_pMax);
				ar & boost::serialization::make_nvp("pN", _pN);
		}

		template <class Archive>
		void rotations::serialize(Archive & ar, const unsigned int version)
		{
			ar & boost::serialization::make_nvp("rtmath_ddscat_rotationsBase", 
				boost::serialization::base_object<rtmath::ddscat::rotationsBase>(*this));
		}

		EXPORTINTERNAL(rtmath::ddscat::rotationsBase::serialize);
		EXPORTINTERNAL(rtmath::ddscat::rotations::serialize);
	}
}


BOOST_CLASS_EXPORT_IMPLEMENT(rtmath::ddscat::rotationsBase);
BOOST_CLASS_EXPORT_IMPLEMENT(rtmath::ddscat::rotations);
