#pragma once
#include <boost/archive/xml_oarchive.hpp>
#include <boost/archive/xml_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/serialization/nvp.hpp> 
#include <boost/preprocessor/repetition.hpp> // used for boost tuple serialization
#include <boost/serialization/serialization.hpp>
#include <boost/serialization/string.hpp> 
#include <boost/serialization/access.hpp>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/set.hpp>
#include <boost/serialization/version.hpp>

namespace boost
{
	namespace serialization
	{
		template <typename T, class Archive>
		void serialize(Archive & ar, rtmath::paramSet & g, const unsigned int version)
		{
			ar & boost::serialization::make_nvp("values_short", g._shorthand);
			ar & boost::serialization::make_nvp("values_expanded", g._expanded);
		}



		// boost tuple serialization - from http://uint32t.blogspot.com/2008/03/update-serializing-boosttuple-using.html


#define GENERATE_ELEMENT_SERIALIZE(z,which,unused) \
	ar & boost::serialization::make_nvp("element",t.get< which >());

#define GENERATE_TUPLE_SERIALIZE(z,nargs,unused)                        \
	template< typename Archive, BOOST_PP_ENUM_PARAMS(nargs,typename T) > \
	void serialize(Archive & ar,                                        \
	boost::tuple< BOOST_PP_ENUM_PARAMS(nargs,T) > & t,   \
	const unsigned int version)                          \
		{                                                                   \
		BOOST_PP_REPEAT_FROM_TO(0,nargs,GENERATE_ELEMENT_SERIALIZE,~);    \
		}


		BOOST_PP_REPEAT_FROM_TO(1,6,GENERATE_TUPLE_SERIALIZE,~);

	}
}

