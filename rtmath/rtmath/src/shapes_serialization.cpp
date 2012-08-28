#include "../rtmath/Stdafx.h"
#include "../rtmath/ddscat/shapes.h"
#include "../rtmath/Serialization/shapes_serialization.h"
#include "../rtmath/Serialization/serialization_macros.h"

#include <boost/serialization/base_object.hpp>
#include <boost/serialization/export.hpp>

namespace boost
{
	namespace serialization
	{
		template <class Archive>
		void serialize(Archive & ar, rtmath::ddscat::shapeConstraint & g, const unsigned int version)
		{
				ar & boost::serialization::make_nvp("variable", varname);
				ar & boost::serialization::make_nvp("paramSet",pset);
				ar & boost::serialization::make_nvp("units", units);
		}

		template <class Archive>
		void serialize(Archive & ar, rtmath::ddscat::constrainable & g, const unsigned int version)
		{
				ar & boost::serialization::make_nvp("shapeConstraints", shapeConstraints);
		}

		template <class Archive>
		void serialize(Archive & ar, rtmath::ddscat::shape & g, const unsigned int version)
		{
				ar & boost::serialization::base_object<constrainable>(g);
				ar & boost::serialization::make_nvp("densities", _densities);
		}

		template <class Archive>
		void serialize(Archive & ar, rtmath::ddscat::shapeModifiable & g, const unsigned int version)
		{
			// TODO: save the vertex maps?
			ar & boost::serialization::base_object<shape>(g);
			ar & boost::serialization::make_nvp("rotations", _rots);
		}

		template <class Archive>
		void serialize(Archive & ar, rtmath::ddscat::shapes::from_ddscat & g, const unsigned int version)
		{
			// TODO: save the vertex maps?
			ar & boost::serialization::base_object<shapeModifiable>(g);
		}

		template <class Archive>
		void serialize(Archive & ar, rtmath::ddscat::shapes::from_file & g, const unsigned int version)
		{
			// TODO: save the vertex maps?
			ar & boost::serialization::base_object<from_ddscat>(g);
		}

		template <class Archive>
		void serialize(Archive & ar, rtmath::ddscat::shapes::ellipsoid & g, const unsigned int version)
		{
			// TODO: save the vertex maps?
			ar & boost::serialization::base_object<from_ddscat>(g);
		}

		EXPORT(rtmath::ddscat::shapeConstraint);
		EXPORT(rtmath::ddscat::constrainable);
		EXPORT(rtmath::ddscat::shape);
		EXPORT(rtmath::ddscat::shapeModifiable);
		EXPORT(rtmath::ddscat::shapes::from_ddscat);
		EXPORT(rtmath::ddscat::shapes::from_file);
		EXPORT(rtmath::ddscat::shapes::ellipsoid);
	}
}

BOOST_CLASS_EXPORT_IMPLEMENT(rtmath::ddscat::constrainable)
BOOST_CLASS_EXPORT_IMPLEMENT(rtmath::ddscat::shape)
BOOST_CLASS_EXPORT_IMPLEMENT(rtmath::ddscat::shapeModifiable)
BOOST_CLASS_EXPORT_IMPLEMENT(rtmath::ddscat::shapes::from_ddscat)
BOOST_CLASS_EXPORT_IMPLEMENT(rtmath::ddscat::shapes::from_file)
BOOST_CLASS_EXPORT_IMPLEMENT(rtmath::ddscat::shapes::ellipsoid)
