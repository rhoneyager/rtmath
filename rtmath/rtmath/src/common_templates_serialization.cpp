#include "../rtmath/Stdafx.h"
#include <string>
#include "../rtmath/common_templates.h"
#include "../rtmath/Serialization/common_templates_serialization.h"

#include "../rtmath/Serialization/serialization_macros.h"

#include <boost/serialization/base_object.hpp>
#include <boost/serialization/export.hpp>
#include <boost/serialization/version.hpp>
#include <boost/serialization/set.hpp>
#include <boost/serialization/string.hpp>

namespace boost
{
	namespace serialization
	{
		template <class Archive, class T>
		void serialize(Archive & ar, rtmath::paramSet<T> & g, const unsigned int version)
		{
			ar & boost::serialization::make_nvp("values_short", g._shorthand);
			ar & boost::serialization::make_nvp("values_expanded", g._expanded);
		}

		/* These do not work:
		EXPORT(serialize,rtmath::paramSet<double>);
		EXPORT(serialize,rtmath::paramSet<float>);
		EXPORT(serialize,rtmath::paramSet<int>);
		EXPORT(serialize,rtmath::paramSet<size_t>);
		EXPORT(serialize,rtmath::paramSet<std::string>);
		*/
	
		/// \todo Reimplement the paramSet serialization definition for base types other than doubles.

		template void serialize<boost::archive::xml_oarchive, double>(boost::archive::xml_oarchive &, rtmath::paramSet<double> &, const unsigned int);
		template void serialize(boost::archive::xml_iarchive &, rtmath::paramSet<double> &, const unsigned int);


	}
}
