#pragma once
#include <boost/serialization/export.hpp>

namespace rtmath
{
	namespace ddscat
	{
		class shapeConstraint;
		class constrainable;
		class shape;
		class shapeModifiable;
		namespace shapes
		{
			class from_ddscat;
			class from_file;
			class ellipsoid;
		}
	}
}

namespace boost
{
	namespace serialization
	{
		template <class Archive>
		void serialize(Archive & ar, rtmath::ddscat::shapeConstraint & g, const unsigned int version);

		template <class Archive>
		void serialize(Archive & ar, rtmath::ddscat::constrainable & g, const unsigned int version);

		template <class Archive>
		void serialize(Archive & ar, rtmath::ddscat::shape & g, const unsigned int version);

		template <class Archive>
		void serialize(Archive & ar, rtmath::ddscat::shapeModifiable & g, const unsigned int version);

		template <class Archive>
		void serialize(Archive & ar, rtmath::ddscat::shapes::from_ddscat & g, const unsigned int version);

		template <class Archive>
		void serialize(Archive & ar, rtmath::ddscat::shapes::from_file & g, const unsigned int version);

		template <class Archive>
		void serialize(Archive & ar, rtmath::ddscat::shapes::ellipsoid & g, const unsigned int version);
	}
}

BOOST_CLASS_EXPORT_KEY(rtmath::ddscat::constrainable)
BOOST_CLASS_EXPORT_KEY(rtmath::ddscat::shape)
BOOST_CLASS_EXPORT_KEY(rtmath::ddscat::shapeModifiable)
BOOST_CLASS_EXPORT_KEY(rtmath::ddscat::shapes::from_ddscat)
BOOST_CLASS_EXPORT_KEY(rtmath::ddscat::shapes::from_file)
BOOST_CLASS_EXPORT_KEY(rtmath::ddscat::shapes::ellipsoid)
