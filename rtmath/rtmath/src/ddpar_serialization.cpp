#include "../rtmath/Stdafx.h"
#include "../rtmath/ddscat/ddpar.h"

#include "../rtmath/Serialization/serialization_macros.h"

#include <boost/serialization/base_object.hpp>
#include <boost/serialization/split_free.hpp>
#include <boost/serialization/export.hpp>
#include <boost/serialization/version.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/map.hpp>
#include <boost/serialization/shared_ptr.hpp>

namespace rtmath
{
	namespace ddscat
	{
		
		template<class Archive>
		void ddPar::save(Archive &ar, const unsigned int version) const
		{
				std::ostringstream out;
				write(out);
				std::string _savedata;
				_savedata = out.str();
				ar & boost::serialization::make_nvp("Par_File", _savedata);
		}

		template<class Archive>
		void ddPar::load(Archive &ar, const unsigned int version)
		{
				std::string _savedata;
				ar & boost::serialization::make_nvp("Par_File", _savedata);
				std::istringstream in(_savedata);
				read(in);
		}
		/*
		template <class Archive>
		void serialize(Archive & ar, rtmath::ddscat::ddPar & g, const unsigned int version)
		{
			boost::serialization::split_free(ar, g, version);
		}
		*/

		template <class Archive>
		void ddPar::serialize(Archive & ar, const unsigned int version)
		{
			if (version)
			{
				ar & boost::serialization::make_nvp("version", _version);
				ar & boost::serialization::make_nvp("parsedData", _parsedData);
				ar & boost::serialization::make_nvp("scaPlanes", _scaPlanes);
				ar & boost::serialization::make_nvp("diels", _diels);
			} else {
				// Older versions just write the ddscat.par file as a big strin
				//boost::serialization::split_free(ar, g, version);
				boost::serialization::split_member(ar, *this, version);
			}
		}
		EXPORTINTERNAL(rtmath::ddscat::ddPar::serialize);
		//EXPORTINTERNAL(rtmath::ddscat::ddPar::load);
		//EXPORTINTERNAL(rtmath::ddscat::ddPar::save);
	}
}

BOOST_CLASS_VERSION(rtmath::ddscat::ddPar, 1);
BOOST_CLASS_EXPORT_IMPLEMENT(rtmath::ddscat::ddPar);
