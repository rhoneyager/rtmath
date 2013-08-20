#include "Stdafx-ddscat.h"
#include "../rtmath/ddscat/ddOutput.h"
#include "../rtmath/ddscat/ddOutputGenerator.h"

#include "../rtmath/Serialization/serialization_macros.h"

#include <boost/serialization/base_object.hpp>
#include <boost/serialization/export.hpp>
#include <boost/serialization/version.hpp>
#include <boost/serialization/string.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/set.hpp>
#include <boost/serialization/shared_ptr.hpp>
#include <boost/serialization/complex.hpp>

namespace rtmath
{
	namespace ddscat
	{
		template<class Archive>
		void ddOutputGenerator::serialize(Archive &ar, const unsigned int version)
		{
			ar & boost::serialization::make_nvp("Source", src);
			ar & boost::serialization::make_nvp("Result", res);
		}
		/*
		template<class Archive>
		void ddOutputGeneratorIsoAll::serialize(Archive &ar, const unsigned int version)
		{
			ar & boost::serialization::make_nvp(
				"rtmath_ddscat_ddOutputGenerator", 
				boost::serialization::base_object<rtmath::ddscat::ddOutputGenerator>(*this));

		}

		template<class Archive>
		void ddOutputGeneratorDDSCAT::serialize(Archive &ar, const unsigned int version)
		{
			ar & boost::serialization::make_nvp(
				"rtmath_ddscat_ddOutputGenerator", 
				boost::serialization::base_object<rtmath::ddscat::ddOutputGenerator>(*this));

		}

		template<class Archive>
		void ddOutputGeneratorThetaAligned::serialize(Archive &ar, const unsigned int version)
		{
			ar & boost::serialization::make_nvp(
				"rtmath_ddscat_ddOutputGenerator", 
				boost::serialization::base_object<rtmath::ddscat::ddOutputGenerator>(*this));

		}
		*/
		EXPORTINTERNAL(rtmath::ddscat::ddOutputGenerator::serialize);
		//EXPORTINTERNAL(rtmath::ddscat::ddOutputGeneratorIsoAll::serialize);
		//EXPORTINTERNAL(rtmath::ddscat::ddOutputGeneratorDDSCAT::serialize);
		//EXPORTINTERNAL(rtmath::ddscat::ddOutputGeneratorThetaAligned::serialize);
	}
}

BOOST_CLASS_EXPORT_IMPLEMENT(rtmath::ddscat::ddOutputGenerator);
//BOOST_CLASS_EXPORT_IMPLEMENT(rtmath::ddscat::ddOutputGeneratorIsoAll);
//BOOST_CLASS_EXPORT_IMPLEMENT(rtmath::ddscat::ddOutputGeneratorDDSCAT);
//BOOST_CLASS_EXPORT_IMPLEMENT(rtmath::ddscat::ddOutputGeneratorThetaAligned);
