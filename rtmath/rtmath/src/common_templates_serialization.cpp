#include "../rtmath/Stdafx.h"
#include <string>
#include "../rtmath/common_templates.h"

#include "../rtmath/Serialization/serialization_macros.h"

#include <boost/serialization/base_object.hpp>
#include <boost/serialization/export.hpp>
#include <boost/serialization/version.hpp>
#include <boost/serialization/set.hpp>
#include <boost/serialization/string.hpp>

namespace rtmath
{
	template <class Archive, class T>
	void paramSet<T>::serialize(Archive & ar, const unsigned int version)
	{
		ar & boost::serialization::make_nvp("values_short", _shorthand);
		ar & boost::serialization::make_nvp("values_expanded", _expanded);
	}

	EXPORTINTERNAL(rtmath::paramSet<double>::serialize);
	/*
	EXPORTINTERNAL(rtmath::paramSet<float>::serialize);
	EXPORTINTERNAL(rtmath::paramSet<int>::serialize);
	EXPORTINTERNAL(rtmath::paramSet<size_t>::serialize);
	EXPORTINTERNAL(rtmath::paramSet<std::string>::serialize);
	*/
}

BOOST_CLASS_EXPORT_IMPLEMENT(rtmath::paramSet<double>);
/*
BOOST_CLASS_EXPORT_IMPLEMENT(rtmath::paramSet<float>);
BOOST_CLASS_EXPORT_IMPLEMENT(rtmath::paramSet<int>);
BOOST_CLASS_EXPORT_IMPLEMENT(rtmath::paramSet<size_t>);
BOOST_CLASS_EXPORT_IMPLEMENT(rtmath::paramSet<std::string>);
*/
