#pragma once
#include <boost/serialization/export.hpp>
#include "matrixop_serialization.h"

namespace rtmath
{
	namespace ddscat
	{
		class ddScattMatrix;
		class ddScattMatrixF;
		class ddScattMatrixP;
	}
}

namespace boost
{
	namespace serialization
	{
		template <class Archive>
		void serialize(Archive & ar, rtmath::ddscat::ddScattMatrix & g, const unsigned int version);

		template <class Archive>
		void serialize(Archive & ar, rtmath::ddscat::ddScattMatrixF & g, const unsigned int version);

		template <class Archive>
		void serialize(Archive & ar, rtmath::ddscat::ddScattMatrixP & g, const unsigned int version);
	}
}

BOOST_CLASS_EXPORT_KEY(rtmath::ddscat::ddScattMatrix)
BOOST_CLASS_EXPORT_KEY(rtmath::ddscat::ddScattMatrixF)
BOOST_CLASS_EXPORT_KEY(rtmath::ddscat::ddScattMatrixP)

