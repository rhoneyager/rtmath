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

namespace boost
{
	namespace serialization
	{
		
		template<class Archive>
		void save(Archive &ar, const rtmath::ddscat::ddPar &g, const unsigned int version)
		{
				std::ostringstream out;
				g.write(out);
				std::string _savedata;
				_savedata = out.str();
				ar & boost::serialization::make_nvp("Par_File", _savedata);
		}

		template<class Archive>
		void load(Archive &ar, rtmath::ddscat::ddPar &g, const unsigned int version)
		{
				std::string _savedata;
				ar & boost::serialization::make_nvp("Par_File", _savedata);
				std::istringstream in(_savedata);
				g.read(in);
		}
		/*
		template <class Archive>
		void serialize(Archive & ar, rtmath::ddscat::ddPar & g, const unsigned int version)
		{
			boost::serialization::split_free(ar, g, version);
		}
		*/

		template <class Archive>
		void serialize(Archive & ar, rtmath::ddscat::ddPar & g, const unsigned int version)
		{
			if (version)
			{
				ar & boost::serialization::make_nvp("version", _version);
				ar & boost::serialization::make_nvp("parsedData", _parsedData);
				ar & boost::serialization::make_nvp("scaPlanes", _scaPlanes);
				ar & boost::serialization::make_nvp("diels", _diels);
			} else {
				// Older versions just write the ddscat.par file as a big string.
				boost::serialization::split_free(ar, g, version);
			}
		}
		EXPORT(serialize, rtmath::ddscat::ddPar);
	}
}

BOOST_CLASS_VERSION(rtmath::ddscat::ddPar, 1);

