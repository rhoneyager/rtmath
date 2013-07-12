#include "../rtmath/Stdafx.h"
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/export.hpp>
#include <boost/serialization/set.hpp>
#include <boost/serialization/shared_ptr.hpp>

#include "../rtmath/ddscat/ddparGenerator.h"
//#include "../rtmath/Serialization/ddparGenerator_serialization.h"
#include "../rtmath/Serialization/serialization_macros.h"



namespace rtmath
{
	namespace ddscat
	{
		template <class Archive>
		void ddParGeneratorBase::serialize(Archive & ar, const unsigned int version)
		{
			ar & boost::serialization::make_nvp("name",name);
			ar & boost::serialization::make_nvp("description",description);
			ar & boost::serialization::make_nvp("outLocation",outLocation);
			ar & boost::serialization::make_nvp("ddscatVer",ddscatVer);
			ar & boost::serialization::make_nvp("strPreCmds",strPreCmds);
			ar & boost::serialization::make_nvp("strPostCdms",strPostCdms);

			ar & boost::serialization::make_nvp("compressResults",compressResults);
			ar & boost::serialization::make_nvp("genIndivScripts",genIndivScripts);
			ar & boost::serialization::make_nvp("genMassScript",genMassScript);
			ar & boost::serialization::make_nvp("shapeStats",shapeStats);
			ar & boost::serialization::make_nvp("registerDatabase",registerDatabase);
			ar & boost::serialization::make_nvp("doExport",doExport);
			ar & boost::serialization::make_nvp("exportLoc",exportLoc);

			// The par file needs serialization, as it holds some properties
			ar & boost::serialization::make_nvp("baseParFile", base);
			ar & boost::serialization::make_nvp("rtmath_ddscat_constrainable",
				boost::serialization::base_object<rtmath::ddscat::constrainable>(*this));

			ar & boost::serialization::make_nvp("shapes", shapes);
			ar & boost::serialization::make_nvp("rots", rots);
		}

		template <class Archive>
		void ddParIterator::serialize(Archive & ar, const unsigned int version)
		{
			ar & boost::serialization::make_nvp("ddParGenerator", _genp);
			ar & boost::serialization::make_nvp("shape", shape);
			ar & boost::serialization::make_nvp("rots", rots);
		}

		template <class Archive>
		void ddParIteration::serialize(Archive & ar, const unsigned int version)
		{
			ar & boost::serialization::make_nvp("elements", _elements);
			ar & boost::serialization::make_nvp("ddParGenerator", _genp);
		}

		template <class Archive>
		void ddParGenerator::serialize(Archive & ar, const unsigned int version)
		{
			ar & boost::serialization::make_nvp(
				"rtmath_ddscat_ddParGeneratorBase", 
				boost::serialization::base_object<rtmath::ddscat::ddParGeneratorBase>(*this));
		}

		EXPORTINTERNAL(rtmath::ddscat::ddParGeneratorBase::serialize);
		EXPORTINTERNAL(rtmath::ddscat::ddParIterator::serialize);
		EXPORTINTERNAL(rtmath::ddscat::ddParIteration::serialize);
		EXPORTINTERNAL(rtmath::ddscat::ddParGenerator::serialize);
	}
}

BOOST_CLASS_EXPORT_IMPLEMENT(rtmath::ddscat::ddParGeneratorBase)
BOOST_CLASS_EXPORT_IMPLEMENT(rtmath::ddscat::ddParGenerator)

