#pragma once
#pragma deprecated(tmData_serialization_h)
#pragma message("tmData_serialization.h is deprecated")
#include <boost/serialization/version.hpp>
#include <boost/serialization/vector.hpp>
namespace rtmath
{
	namespace tmatrix
	{
		class tmStats;
		class tmData;
	}
}

namespace boost
{
	namespace serialization
	{
		template<class Archive>
		void serialize(Archive & ar, rtmath::tmatrix::tmStats & g, const unsigned int version);
		template<class Archive>
		void serialize(Archive & ar, rtmath::tmatrix::tmData & g, const unsigned int version);
	}
}

BOOST_CLASS_VERSION(rtmath::tmatrix::tmData, 1);
