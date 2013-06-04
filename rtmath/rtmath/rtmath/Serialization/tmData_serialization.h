#pragma once
#pragma message("TODO: no need for separate serialization header if forward declared properly")

#include <boost/serialization/version.hpp>

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
