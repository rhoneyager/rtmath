#pragma once
#include <boost/archive/xml_oarchive.hpp>
#include <boost/archive/xml_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/export.hpp>
#include <boost/serialization/shared_ptr.hpp>
#include <boost/serialization/split_member.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/version.hpp>

#include "../ddscat/shapes.h"

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
	}
}

BOOST_CLASS_EXPORT_KEY(rtmath::ddscat::constrainable)
BOOST_CLASS_EXPORT_KEY(rtmath::ddscat::shape)
BOOST_CLASS_EXPORT_KEY(rtmath::ddscat::shapeModifiable)
BOOST_CLASS_EXPORT_KEY(rtmath::ddscat::shapes::from_ddscat)
BOOST_CLASS_EXPORT_KEY(rtmath::ddscat::shapes::from_file)
BOOST_CLASS_EXPORT_KEY(rtmath::ddscat::shapes::ellipsoid)
