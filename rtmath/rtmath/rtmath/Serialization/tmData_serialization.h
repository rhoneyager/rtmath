#pragma once

namespace rtmath
{
	namespace tmatrix
	{
		class tmData;
	}
}

namespace boost
{
	namespace serialization
	{
		template<class Archive>
		void serialize(Archive & ar, rtmath::tmatrix::tmData & g, const unsigned int version);
	}
}
