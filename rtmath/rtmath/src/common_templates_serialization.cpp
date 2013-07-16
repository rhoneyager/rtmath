#include "../rtmath/Stdafx.h"
#include <string>
#include "../rtmath/common_templates.h"

#include "../rtmath/Serialization/serialization_macros.h"

#include <boost/serialization/base_object.hpp>
#include <boost/serialization/export.hpp>
#include <boost/serialization/version.hpp>
#include <boost/serialization/set.hpp>
#include <boost/serialization/string.hpp>

//#include "../rtmath/Serialization/common_templates_serialization.h"
/*
namespace boost
{
	namespace serialization
	{
		/// paramSet serialization
		/// \note MSVC 2012 cannot compile the internal definition properly. It has issues 
		/// with nested templates. This is why the external definition is still used.
		template <class Archive, class T>
		void serialize(Archive & ar, rtmath::paramSet<T> & g, const unsigned int version)
		{
			ar & boost::serialization::make_nvp("values_short", g._shorthand);
			ar & boost::serialization::make_nvp("values_expanded", g._expanded);
		}
	
		EXPORTTEMPLATE(serialize, double, rtmath::paramSet);
		EXPORTTEMPLATE(serialize, float, rtmath::paramSet);
		EXPORTTEMPLATE(serialize, int, rtmath::paramSet);
		EXPORTTEMPLATE(serialize, size_t, rtmath::paramSet);
		EXPORTTEMPLATE(serialize, std::string, rtmath::paramSet);

		//template void serialize<boost::archive::xml_oarchive, double>(boost::archive::xml_oarchive &, rtmath::paramSet<double> &, const unsigned int);
		//template void serialize(boost::archive::xml_iarchive &, rtmath::paramSet<double> &, const unsigned int);

	}
}
BOOST_CLASS_EXPORT_IMPLEMENT(rtmath::paramSet<double>);
BOOST_CLASS_EXPORT_IMPLEMENT(rtmath::paramSet<float>);
BOOST_CLASS_EXPORT_IMPLEMENT(rtmath::paramSet<int>);
BOOST_CLASS_EXPORT_IMPLEMENT(rtmath::paramSet<size_t>);
BOOST_CLASS_EXPORT_IMPLEMENT(rtmath::paramSet<std::string>);
*/
// MSVC2012 can't compile this...
/*
namespace rtmath
{
	template <class Archive>
	void paramSet<double>::serialize<Archive>(Archive & ar, const unsigned int version)
	{
		ar & boost::serialization::make_nvp("values_short", _shorthand);
		ar & boost::serialization::make_nvp("values_expanded", _expanded);
	}

	template void paramSet<double>::serialize<boost::archive::xml_oarchive>(boost::archive::xml_oarchive &, const unsigned int);
	template void paramSet<double>::serialize<boost::archive::xml_iarchive>(boost::archive::xml_iarchive &, const unsigned int);

	EXPORTINTERNAL(rtmath::paramSet<double>::serialize);
	EXPORTINTERNAL(rtmath::paramSet<float>::serialize);
	EXPORTINTERNAL(rtmath::paramSet<int>::serialize);
	EXPORTINTERNAL(rtmath::paramSet<size_t>::serialize);
	EXPORTINTERNAL(rtmath::paramSet<std::string>::serialize);
}

BOOST_CLASS_EXPORT_IMPLEMENT(rtmath::paramSet<double>);
BOOST_CLASS_EXPORT_IMPLEMENT(rtmath::paramSet<float>);
BOOST_CLASS_EXPORT_IMPLEMENT(rtmath::paramSet<int>);
BOOST_CLASS_EXPORT_IMPLEMENT(rtmath::paramSet<size_t>);
BOOST_CLASS_EXPORT_IMPLEMENT(rtmath::paramSet<std::string>);
*/

