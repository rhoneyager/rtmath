#include "Stdafx-ddscat_base.h"
#include "../rtmath/defs.h"
#include "../rtmath/ddscat/ddpar.h"
#if USE_RYAN_SERIALIZATION
#include "../rtmath/Serialization/serialization_macros.h"

#include <boost/serialization/base_object.hpp>
#include <boost/serialization/split_free.hpp>
#include <boost/serialization/export.hpp>
#include <boost/serialization/version.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/map.hpp>
#include <boost/serialization/shared_ptr.hpp>
#include <boost/type_traits/is_enum.hpp>

template <class T>
typename boost::enable_if<boost::is_enum<T> >::type serialize(const T&); //use casts here

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
			// Version set to zero to avoid tripping on _parsedData
			/// \todo Fix serialization issue
			if (version)
			{
				ar & boost::serialization::make_nvp("version", _version);
				ar & boost::serialization::make_nvp("parsedData", _parsedData); //!
				ar & boost::serialization::make_nvp("scaPlanes", _scaPlanes);
				ar & boost::serialization::make_nvp("dielectrics", _diels);
				ar & boost::serialization::make_nvp("dielHashes", _dielHashes);
				ar & boost::serialization::make_nvp("from_filename", _filename);
			} else {
				// Older versions just write the ddscat.par file as a big string
				//boost::serialization::split_free(ar, g, version);
				boost::serialization::split_member(ar, *this, version);
			}
		}
		EXPORTINTERNAL(rtmath::ddscat::ddPar::serialize);
		//EXPORTINTERNAL(rtmath::ddscat::ddPar::load);
		//EXPORTINTERNAL(rtmath::ddscat::ddPar::save);
	}
}

BOOST_CLASS_EXPORT_IMPLEMENT(rtmath::ddscat::ddPar);
#endif
