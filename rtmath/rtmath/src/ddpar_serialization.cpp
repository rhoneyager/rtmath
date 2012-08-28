#include "../rtmath/Stdafx.h"
#include "../rtmath/ddscat/ddpar.h"
#include "../rtmath/Serialization/ddpar_serialization.h"

#include "../rtmath/Serialization/serialization_macros.h"

#include <boost/serialization/base_object.hpp>
#include <boost/serialization/split_free.hpp>
#include <boost/serialization/export.hpp>
#include <boost/serialization/version.hpp>

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

		EXPORT(rtmath::ddscat::ddPar);
	}
}
