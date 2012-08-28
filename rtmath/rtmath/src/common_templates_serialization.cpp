#include "../rtmath/Stdafx.h"
#include <string>
#include "../rtmath/common_templates.h"
#include "../rtmath/Serialization/common_templates_serialization.h"

#include "../rtmath/Serialization/serialization_macros.h"

#include <boost/serialization/base_object.hpp>
#include <boost/serialization/export.hpp>
#include <boost/serialization/version.hpp>

namespace boost
{
	namespace serialization
	{
		template <class T, class Archive>
		void serialize(Archive & ar, rtmath::paramSet<T> & g, const unsigned int version)
		{
			ar & boost::serialization::make_nvp("values_short", g._shorthand);
			ar & boost::serialization::make_nvp("values_expanded", g._expanded);
		}

		EXPORT(rtmath::paramSet<double>);
		EXPORT(rtmath::paramSet<float>);
		EXPORT(rtmath::paramSet<int>);
		EXPORT(rtmath::paramSet<size_t>);
		EXPORT(rtmath::paramSet<std::string>);

	}
}
