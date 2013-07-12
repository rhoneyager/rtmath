#pragma once
#include <boost/archive/xml_oarchive.hpp>
#include <boost/archive/xml_iarchive.hpp>
//#include <boost/archive/text_oarchive.hpp>
//#include <boost/archive/text_iarchive.hpp>

/// \def EXPORT(U,T) Defines the exportable types (txt, xml) for all rtmath objects. 
/// Macro used in serialization source files.

#define EXPORT(U,T) \
	template void DLEXPORT_RTMATH U(::boost::archive::xml_oarchive &, T &, const unsigned int); \
	template void DLEXPORT_RTMATH U(::boost::archive::xml_iarchive &, T &, const unsigned int);

/// \def EXPORTTEMPLATE(U,T,W) Defines exportable types for template objects.
/// It's odd structure is to avoid accidental specialization of all objects.
/// \see rtmath::paramSet<T> for an example.
/// \note This is generally used with nested template specializations, as MSVC 2012 
/// and below have a compiler bug.

#define EXPORTTEMPLATE(U,T,W) \
	template void DLEXPORT_RTMATH U<::boost::archive::xml_oarchive, T>(::boost::archive::xml_oarchive &,W<T>&, const unsigned int); \
	template void DLEXPORT_RTMATH U(::boost::archive::xml_iarchive &, W<T> &, const unsigned int);

/// \def EXPORTINTERNAL(U) Is used when defining serializations that are defined internally 
/// in a class (where boost::serialization::access is a friend). This is the preferred type of export.
/// \note DLEXPORT_RTMATH tag is not needed, as it is defined in an exported class.

#define EXPORTINTERNAL(U) \
	template void U(::boost::archive::xml_oarchive &, const unsigned int); \
	template void U(::boost::archive::xml_iarchive &, const unsigned int);

// 	template void U(boost::archive::text_oarchive &, T &, const unsigned int); 
//	template void U(boost::archive::text_iarchive &, T &, const unsigned int); 
