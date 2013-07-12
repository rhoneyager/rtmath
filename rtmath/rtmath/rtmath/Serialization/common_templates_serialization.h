#pragma once
#pragma message("common_templates_serialization.h is deprecated")
//#include <boost/preprocessor/repetition.hpp> // used for boost tuple serialization

namespace rtmath
{
	template <class T>
	class paramSet;
}

namespace boost
{
	namespace serialization
	{
		/// Definition to serialize all paramSet objects.
		//template <class Archive, class T>
		//void serialize(Archive & ar, rtmath::paramSet<T> & g, const unsigned int version);


		/// \todo Fix boost tuple serialization to work with LLVM/CLANG.
		// boost tuple serialization - from http://uint32t.blogspot.com/2008/03/update-serializing-boosttuple-using.html
		// Breaks in CLANG!
/*
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
*/
	}
}

