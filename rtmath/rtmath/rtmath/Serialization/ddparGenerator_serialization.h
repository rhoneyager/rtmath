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

#include "shapes_serialization.h"
#include "ddpar_serialization.h"

#include "../ddscat/ddparGenerator.h"

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
	}
}

BOOST_CLASS_EXPORT_KEY(rtmath::ddscat::ddParGeneratorBase)
BOOST_CLASS_EXPORT_KEY(rtmath::ddscat::ddParGenerator)

