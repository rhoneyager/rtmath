#include "../rtmath/Stdafx.h"
#include "../rtmath/ddscat/ddOutput.h"
#include "../rtmath/ddscat/ddOutputEnsemble.h"

#include "../rtmath/Serialization/serialization_macros.h"

#include <boost/serialization/base_object.hpp>
#include <boost/serialization/export.hpp>
#include <boost/serialization/version.hpp>
#include <boost/serialization/string.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/set.hpp>
#include <boost/serialization/shared_ptr.hpp>
#include <boost/serialization/complex.hpp>

namespace rtmath
{
	namespace ddscat
	{
		template<class Archive>
		void ddOutputEnsemble::serialize(Archive &ar, const unsigned int version)
		{
			ar & boost::serialization::make_nvp("Source", src);
			ar & boost::serialization::make_nvp("Result", res);
		}

		EXPORTINTERNAL(rtmath::ddscat::ddOutputEnsemble::serialize);
	}
}

BOOST_CLASS_EXPORT_IMPLEMENT(rtmath::ddscat::ddOutputEnsemble);
