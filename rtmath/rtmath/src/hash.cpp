#include "../rtmath/Stdafx.h"
#include "../rtmath/defs.h"
#include "../rtmath/hash.h"
#include "../rtmath/Public_Domain/MurmurHash3.h"
#include "../rtmath/Serialization/serialization_macros.h"

#include <boost/serialization/export.hpp>
#include <boost/serialization/version.hpp>


namespace rtmath {
	HASH_t HASH(const void *key, int len)
	{
		HASH_t res;
		MurmurHash3_x64_128(key, len, HASHSEED, &res);
		return res;
	}

}

namespace boost
{
	namespace serialization
	{
		template<class Archive>
		void serialize(Archive &ar, rtmath::UINT128 &g, const unsigned int version)
		{
			ar & boost::serialization::make_nvp("upper", g.upper);
			ar & boost::serialization::make_nvp("lower", g.lower);
		}

		EXPORT(serialize, rtmath::UINT128);
		//EXPORTINTERNAL(rtmath::hash::serialize);
	}
}


BOOST_CLASS_EXPORT_IMPLEMENT(rtmath::UINT128);
