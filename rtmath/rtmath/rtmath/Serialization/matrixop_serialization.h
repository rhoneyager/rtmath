#pragma once

namespace rtmath
{
	class matrixop;
}

namespace boost
{
	namespace serialization
	{
		template <class Archive>
		void serialize(Archive & ar, rtmath::matrixop & g, const unsigned int version);
	}
}

