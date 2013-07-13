#include "../rtmath/Stdafx.h"
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/export.hpp>
#include <boost/serialization/map.hpp>
#include <boost/serialization/shared_ptr.hpp>

#include "../rtmath/ddscat/shapes.h"
#include "../rtmath/common_templates.h"
//#include "../rtmath/Serialization/shapes_serialization.h"
#include "../rtmath/Serialization/eigen_serialization.h"
#include "../rtmath/Serialization/serialization_macros.h"


namespace rtmath
{
	namespace ddscat
	{
		template <class Archive>
		void shapeConstraint::serialize(Archive & ar, const unsigned int version)
		{
				ar & boost::serialization::make_nvp("variable", varname);
				ar & boost::serialization::make_nvp("paramSet",pset);
				ar & boost::serialization::make_nvp("units", units);
		}

		template <class Archive>
		void constrainable::serialize(Archive & ar, const unsigned int version)
		{
				ar & boost::serialization::make_nvp("shapeConstraints", shapeConstraints);
		}

		template <class Archive>
		void shape::serialize(Archive & ar, const unsigned int version)
		{
				ar & boost::serialization::make_nvp(
					"rtmath_ddscat_constrainable",
					boost::serialization::base_object<rtmath::ddscat::constrainable>(*this));
				ar & boost::serialization::make_nvp("densities", _densities);
		}

		template <class Archive>
		void shapeModifiable::serialize(Archive & ar, const unsigned int version)
		{
			// TODO: save the vertex maps?
			ar & boost::serialization::make_nvp(
				"rtmath_ddscat_shape",
				boost::serialization::base_object<rtmath::ddscat::shape>(*this));
			ar & boost::serialization::make_nvp("rotations", _rots);
		}

		namespace shapes
		{
			template <class Archive>
			void from_ddscat::serialize(Archive & ar, const unsigned int version)
			{
				// TODO: save the vertex maps?
				ar & boost::serialization::make_nvp(
					"rtmath_ddscat_shapeModifiable",
					boost::serialization::base_object<rtmath::ddscat::shapeModifiable>(*this));
			}

			template <class Archive>
			void from_file::serialize(Archive & ar, const unsigned int version)
			{
				// TODO: save the vertex maps?
				ar & boost::serialization::make_nvp(
					"rtmath_ddscat_shapes_from_ddscat",
					boost::serialization::base_object<rtmath::ddscat::shapes::from_ddscat>(*this));
			}

			template <class Archive>
			void ellipsoid::serialize(Archive & ar, const unsigned int version)
			{
				// TODO: save the vertex maps?
				ar & boost::serialization::make_nvp(
					"rtmath_ddscat_shapes_from_ddscat",
					boost::serialization::base_object<rtmath::ddscat::shapes::from_ddscat>(*this));
			}
		}

		EXPORTINTERNAL(rtmath::ddscat::shapeConstraint::serialize);
		EXPORTINTERNAL(rtmath::ddscat::constrainable::serialize);
		EXPORTINTERNAL(rtmath::ddscat::shape::serialize);
		EXPORTINTERNAL(rtmath::ddscat::shapeModifiable::serialize);
		EXPORTINTERNAL(rtmath::ddscat::shapes::from_ddscat::serialize);
		EXPORTINTERNAL(rtmath::ddscat::shapes::from_file::serialize);
		EXPORTINTERNAL(rtmath::ddscat::shapes::ellipsoid::serialize);
	}
}

BOOST_CLASS_EXPORT_IMPLEMENT(rtmath::ddscat::constrainable)
BOOST_CLASS_EXPORT_IMPLEMENT(rtmath::ddscat::shape)
BOOST_CLASS_EXPORT_IMPLEMENT(rtmath::ddscat::shapeModifiable)
BOOST_CLASS_EXPORT_IMPLEMENT(rtmath::ddscat::shapes::from_ddscat)
BOOST_CLASS_EXPORT_IMPLEMENT(rtmath::ddscat::shapes::from_file)
BOOST_CLASS_EXPORT_IMPLEMENT(rtmath::ddscat::shapes::ellipsoid)
