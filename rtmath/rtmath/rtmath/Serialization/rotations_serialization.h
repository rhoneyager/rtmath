#pragma once
#pragma deprecated(rotations_serialization_h)
#pragma message("rotations_serialization.h is deprecated")

namespace rtmath
{
	namespace ddscat
	{
		class rotationsBase;
		class rotations;
	}
}

namespace boost
{
	namespace serialization
	{
		template<class Archive>
		void serialize(Archive & ar, rtmath::ddscat::rotationsBase & g, const unsigned int version);

		template <class Archive>
		void serialize(Archive & ar, rtmath::ddscat::rotations & g, const unsigned int version);
	}
}
