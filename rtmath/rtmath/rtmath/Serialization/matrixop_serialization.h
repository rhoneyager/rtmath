#pragma once
#include "../matrixop.h"

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
		void save(Archive &ar, rtmath::matrixop &g, const unsigned int version)
		{
			ar & boost::serialization::make_nvp("Dimensions", g._dims);
			//ar & boost::serialization::make_nvp("Size", _datasize);
			std::ostringstream out;
			for (size_t i=0; i<g._datasize-1; i++)
			{
				out << g._data[i] << ",";
			}
			out << g._data[g._datasize-1];
			std::string savedata = out.str();
			ar & boost::serialization::make_nvp("Data", savedata);
		}

		template<class Archive>
		void load(Archive &ar, rtmath::matrixop &g, const unsigned int version)
		{
			std::string savedata;
			ar & boost::serialization::make_nvp("Dimensions", g._dims);
			g.resize(g._dims);
			// Loading _datasize is sort of pointless, as resize sets it
			//ar & boost::serialization::make_nvp("Size", _datasize);
			ar & boost::serialization::make_nvp("Data", savedata);
			typedef boost::tokenizer<boost::char_separator<char> >
				tokenizer;
			boost::char_separator<char> sep(",");
			tokenizer tcom(savedata,sep);
			size_t i=0;
			for (auto ot = tcom.begin(); ot != tcom.end(); ot++, i++)
			{
				g._data[i] = boost::lexical_cast<double>(*ot);
			}
		}

		template <class Archive>
		void serialize(Archive & ar, rtmath::matrixop & g, const unsigned int version)
		{
			boost::serialization::split_free(ar, g, version);
		}
	}
}

