#pragma once

namespace rtmath
{
	namespace ddscat
	{
		class ddPar;
	}
}

namespace boost
{
	namespace serialization
	{
		template<class Archive>
		void save(Archive &ar, const rtmath::ddscat::ddPar &g, const unsigned int version);

		template<class Archive>
		void load(Archive &ar, rtmath::ddscat::ddPar &g, const unsigned int version);

		template <class Archive>
		void serialize(Archive & ar, rtmath::ddscat::ddPar & g, const unsigned int version);
	}
}

