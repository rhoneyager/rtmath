#include "../rtmath/Stdafx.h"
#include "../rtmath/ddscat/ddparGenerator.h"
#include "../rtmath/Serialization/ddparGenerator_serialization.h"
#include "../rtmath/Serialization/serialization_macros.h"

#include <boost/serialization/base_object.hpp>
#include <boost/serialization/export.hpp>

namespace boost
{
	namespace serialization
	{
		template <class Archive>
		void serialize(Archive & ar, rtmath::ddscat::ddParGeneratorBase & g, const unsigned int version)
		{
			ar & BOOST_SERIALIZATION_NVP(g.name);
			ar & BOOST_SERIALIZATION_NVP(g.description);
			ar & BOOST_SERIALIZATION_NVP(g.outLocation);
			ar & BOOST_SERIALIZATION_NVP(g.ddscatVer);
			ar & BOOST_SERIALIZATION_NVP(g.strPreCmds);
			ar & BOOST_SERIALIZATION_NVP(g.strPostCdms);

			ar & BOOST_SERIALIZATION_NVP(g.compressResults);
			ar & BOOST_SERIALIZATION_NVP(g.genIndivScripts);
			ar & BOOST_SERIALIZATION_NVP(g.genMassScript);
			ar & BOOST_SERIALIZATION_NVP(g.shapeStats);
			ar & BOOST_SERIALIZATION_NVP(g.registerDatabase);
			ar & BOOST_SERIALIZATION_NVP(g.doExport);
			ar & BOOST_SERIALIZATION_NVP(g.exportLoc);

			// The par file needs serialization, as it holds some properties
			ar & boost::serialization::make_nvp("baseParFile", g.base);
			ar & boost::serialization::base_object<rtmath::ddscat::constrainable>(g);

			ar & BOOST_SERIALIZATION_NVP(g.rots);
			ar & BOOST_SERIALIZATION_NVP(g.shapes);
		}

		template <class Archive>
		void serialize(Archive & ar, rtmath::ddscat::ddParIterator & g, const unsigned int version)
		{
			ar & boost::serialization::make_nvp("ddParGenerator", g._genp);
			ar & BOOST_SERIALIZATION_NVP(g.shape);
			ar & BOOST_SERIALIZATION_NVP(g.rots);
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
			ar & boost::serialization::base_object<rtmath::ddscat::ddParGeneratorBase>(g);
		}

		EXPORT(rtmath::ddscat::ddParGeneratorBase);
		EXPORT(rtmath::ddscat::ddParIterator);
		EXPORT(rtmath::ddscat::ddParIteration);
		EXPORT(rtmath::ddscat::ddParGenerator);
	}
}

BOOST_CLASS_EXPORT_IMPLEMENT(rtmath::ddscat::ddParGeneratorBase)
BOOST_CLASS_EXPORT_IMPLEMENT(rtmath::ddscat::ddParGenerator)

