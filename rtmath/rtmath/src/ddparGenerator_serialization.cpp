#include "../rtmath/Stdafx.h"
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/export.hpp>
#include <boost/serialization/set.hpp>
#include <boost/serialization/shared_ptr.hpp>

#include "../rtmath/ddscat/ddparGenerator.h"
#include "../rtmath/Serialization/ddparGenerator_serialization.h"
#include "../rtmath/Serialization/serialization_macros.h"



namespace boost
{
	namespace serialization
	{
		template <class Archive>
		void serialize(Archive & ar, rtmath::ddscat::ddParGeneratorBase & g, const unsigned int version)
		{
			ar & boost::serialization::make_nvp("name",g.name);
			ar & boost::serialization::make_nvp("description",g.description);
			ar & boost::serialization::make_nvp("outLocation",g.outLocation);
			ar & boost::serialization::make_nvp("ddscatVer",g.ddscatVer);
			ar & boost::serialization::make_nvp("strPreCmds",g.strPreCmds);
			ar & boost::serialization::make_nvp("strPostCdms",g.strPostCdms);

			ar & boost::serialization::make_nvp("compressResults",g.compressResults);
			ar & boost::serialization::make_nvp("genIndivScripts",g.genIndivScripts);
			ar & boost::serialization::make_nvp("genMassScript",g.genMassScript);
			ar & boost::serialization::make_nvp("shapeStats",g.shapeStats);
			ar & boost::serialization::make_nvp("registerDatabase",g.registerDatabase);
			ar & boost::serialization::make_nvp("doExport",g.doExport);
			ar & boost::serialization::make_nvp("exportLoc",g.exportLoc);

			// The par file needs serialization, as it holds some properties
			ar & boost::serialization::make_nvp("baseParFile", g.base);
			ar & boost::serialization::make_nvp("rtmath_ddscat_constrainable",
				boost::serialization::base_object<rtmath::ddscat::constrainable>(g));

			ar & boost::serialization::make_nvp("shapes", g.shapes);
			ar & boost::serialization::make_nvp("rots", g.rots);
		}

		template <class Archive>
		void serialize(Archive & ar, rtmath::ddscat::ddParIterator & g, const unsigned int version)
		{
			ar & boost::serialization::make_nvp("ddParGenerator", g._genp);
			ar & boost::serialization::make_nvp("shape", g.shape);
			ar & boost::serialization::make_nvp("rots", g.rots);
		}

		template <class Archive>
		void serialize(Archive & ar, rtmath::ddscat::ddParIteration & g, const unsigned int version)
		{
			ar & boost::serialization::make_nvp("elements", g._elements);
			ar & boost::serialization::make_nvp("ddParGenerator", g._genp);
		}

		template <class Archive>
		void serialize(Archive & ar, rtmath::ddscat::ddParGenerator & g, const unsigned int version)
		{
			ar & boost::serialization::make_nvp(
				"rtmath_ddscat_ddParGeneratorBase", 
				boost::serialization::base_object<rtmath::ddscat::ddParGeneratorBase>(g));
		}

		EXPORT(serialize,rtmath::ddscat::ddParGeneratorBase);
		EXPORT(serialize,rtmath::ddscat::ddParIterator);
		EXPORT(serialize,rtmath::ddscat::ddParIteration);
		EXPORT(serialize,rtmath::ddscat::ddParGenerator);
	}
}

BOOST_CLASS_EXPORT_IMPLEMENT(rtmath::ddscat::ddParGeneratorBase)
BOOST_CLASS_EXPORT_IMPLEMENT(rtmath::ddscat::ddParGenerator)

