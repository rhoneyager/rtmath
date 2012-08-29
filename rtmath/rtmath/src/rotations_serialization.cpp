#include "../rtmath/Stdafx.h"
#include "../rtmath/ddscat/rotations.h"
#include "../rtmath/Serialization/rotations_serialization.h"
#include "../rtmath/Serialization/serialization_macros.h"

#include <boost/serialization/base_object.hpp>
#include <boost/serialization/export.hpp>
#include <boost/serialization/version.hpp>
#include <boost/serialization/set.hpp>

namespace boost
{
	namespace serialization
	{
		template<class Archive>
		void serialize(Archive &ar, rtmath::ddscat::rotationsBase &g, const unsigned int version)
		{
				ar & boost::serialization::make_nvp("bMin", g._bMin);
				ar & boost::serialization::make_nvp("bMax",g._bMax);
				ar & boost::serialization::make_nvp("bN",g._bN);
				ar & boost::serialization::make_nvp("tMin", g._tMin);
				ar & boost::serialization::make_nvp("tMax", g._tMax);
				ar & boost::serialization::make_nvp("tN", g._tN);
				ar & boost::serialization::make_nvp("pMin",g._pMin);
				ar & boost::serialization::make_nvp("pMax",g._pMax);
				ar & boost::serialization::make_nvp("pN", g._pN);
		}

		template <class Archive>
		void serialize(Archive & ar, rtmath::ddscat::rotations & g, const unsigned int version)
		{
			ar & boost::serialization::make_nvp("rtmath::ddscat::rotationsBase", 
				boost::serialization::base_object<rtmath::ddscat::rotationsBase>(g));
		}

		EXPORT(serialize,rtmath::ddscat::rotationsBase);
		EXPORT(serialize,rtmath::ddscat::rotations);
	}
}

