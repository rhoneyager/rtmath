#pragma once
#include "../ddscat/ddpar.h"

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

#include <boost/lexical_cast.hpp>
#include <boost/tokenizer.hpp>

namespace boost
{
	namespace serialization
	{
		template<class Archive>
		void save(Archive &ar, rtmath::ddscat::ddPar &g, const unsigned int version)
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

		template <class Archive>
		void serialize(Archive & ar, rtmath::ddscat::ddPar & g, const unsigned int version)
		{
			boost::serialization::split_free(ar, g, version);
		}
	}
}

