#include "../rtmath/Stdafx.h"
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/export.hpp>
#include <boost/serialization/map.hpp>
#include <boost/serialization/shared_ptr.hpp>

#include "../rtmath/ddscat/shapes.h"
#include "../rtmath/Serialization/common_templates_serialization.h"
#include "../rtmath/Serialization/rotations_serialization.h"
#include "../rtmath/Serialization/shapes_serialization.h"
#include "../rtmath/Serialization/eigen_serialization.h"
#include "../rtmath/Serialization/serialization_macros.h"


namespace boost
{
	namespace serialization
	{
		template <class Archive>
		void serialize(Archive & ar, rtmath::ddscat::shapeConstraint & g, const unsigned int version)
		{
				ar & boost::serialization::make_nvp("variable", g.varname);
				ar & boost::serialization::make_nvp("paramSet",g.pset);
				ar & boost::serialization::make_nvp("units", g.units);
		}

		template <class Archive>
		void serialize(Archive & ar, rtmath::ddscat::constrainable & g, const unsigned int version)
		{
				ar & boost::serialization::make_nvp("shapeConstraints", g.shapeConstraints);
		}

		template <class Archive>
		void serialize(Archive & ar, rtmath::ddscat::shape & g, const unsigned int version)
		{
				ar & boost::serialization::make_nvp(
					"rtmath_ddscat_constrainable",
					boost::serialization::base_object<rtmath::ddscat::constrainable>(g));
				ar & boost::serialization::make_nvp("densities", g._densities);
		}

		template <class Archive>
		void serialize(Archive & ar, rtmath::ddscat::shapeModifiable & g, const unsigned int version)
		{
			// TODO: save the vertex maps?
			ar & boost::serialization::make_nvp(
				"rtmath_ddscat_shape",
				boost::serialization::base_object<rtmath::ddscat::shape>(g));
			ar & boost::serialization::make_nvp("rotations", g._rots);
		}

		template <class Archive>
		void serialize(Archive & ar, rtmath::ddscat::shapes::from_ddscat & g, const unsigned int version)
		{
			// TODO: save the vertex maps?
			ar & boost::serialization::make_nvp(
				"rtmath_ddscat_shapeModifiable",
				boost::serialization::base_object<rtmath::ddscat::shapeModifiable>(g));
		}

		template <class Archive>
		void serialize(Archive & ar, rtmath::ddscat::shapes::from_file & g, const unsigned int version)
		{
			// TODO: save the vertex maps?
			ar & boost::serialization::make_nvp(
				"rtmath_ddscat_shapes_from_ddscat",
				boost::serialization::base_object<rtmath::ddscat::shapes::from_ddscat>(g));
		}

		template <class Archive>
		void serialize(Archive & ar, rtmath::ddscat::shapes::ellipsoid & g, const unsigned int version)
		{
			// TODO: save the vertex maps?
			ar & boost::serialization::make_nvp(
				"rtmath_ddscat_shapes_from_ddscat",
				boost::serialization::base_object<rtmath::ddscat::shapes::from_ddscat>(g));
		}

		EXPORT(serialize,rtmath::ddscat::shapeConstraint);
		EXPORT(serialize,rtmath::ddscat::constrainable);
		EXPORT(serialize,rtmath::ddscat::shape);
		EXPORT(serialize,rtmath::ddscat::shapeModifiable);
		EXPORT(serialize,rtmath::ddscat::shapes::from_ddscat);
		EXPORT(serialize,rtmath::ddscat::shapes::from_file);
		EXPORT(serialize,rtmath::ddscat::shapes::ellipsoid);
	}
}

BOOST_CLASS_EXPORT_IMPLEMENT(rtmath::ddscat::constrainable)
BOOST_CLASS_EXPORT_IMPLEMENT(rtmath::ddscat::shape)
BOOST_CLASS_EXPORT_IMPLEMENT(rtmath::ddscat::shapeModifiable)
BOOST_CLASS_EXPORT_IMPLEMENT(rtmath::ddscat::shapes::from_ddscat)
BOOST_CLASS_EXPORT_IMPLEMENT(rtmath::ddscat::shapes::from_file)
BOOST_CLASS_EXPORT_IMPLEMENT(rtmath::ddscat::shapes::ellipsoid)
