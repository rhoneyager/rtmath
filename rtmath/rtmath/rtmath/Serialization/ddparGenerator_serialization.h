#pragma once
#include <boost/serialization/export.hpp>

#include "shapes_serialization.h"
#include "ddpar_serialization.h"

namespace rtmath
{
	namespace ddscat
	{
		class ddParGeneratorBase;
		class ddParIterator;
		class ddParIteration;
		class ddParGenerator;
	}
}

namespace boost
{
	namespace serialization
	{
		template <class Archive>
		void serialize(Archive & ar, rtmath::ddscat::ddParGeneratorBase & g, const unsigned int version);

		template <class Archive>
		void serialize(Archive & ar, rtmath::ddscat::ddParIterator & g, const unsigned int version);

		template <class Archive>
		void serialize(Archive & ar, rtmath::ddscat::ddParIteration & g, const unsigned int version);

		template <class Archive>
		void serialize(Archive & ar, rtmath::ddscat::ddParGenerator & g, const unsigned int version);
	}
}

BOOST_CLASS_EXPORT_KEY(rtmath::ddscat::ddParGeneratorBase)
BOOST_CLASS_EXPORT_KEY(rtmath::ddscat::ddParGenerator)
