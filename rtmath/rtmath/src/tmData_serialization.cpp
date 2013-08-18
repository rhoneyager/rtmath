#include "Stdafx-ddscat.h"
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/export.hpp>
#include <boost/serialization/version.hpp>
#include <boost/serialization/set.hpp>
#include <boost/serialization/map.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/shared_ptr.hpp>
#include <boost/serialization/string.hpp>
#include <boost/serialization/complex.hpp>

#include "../rtmath/ddscat/tmData.h"
#include <tmatrix/tmatrix.h>
//#include <tmatrix/tmatrix-serialization.h>
#include "../rtmath/Serialization/serialization_macros.h"
#include "../rtmath/mie/mie.h"


namespace rtmath
{
	namespace tmatrix
	{
		template<class Archive>
		void tmStats::serialize(Archive &ar, const unsigned int version)
		{
				ar & boost::serialization::make_nvp("statmap",stats);
				//ar & boost::serialization::make_nvp("nested", nested);
		}

		template<class Archive>
		void tmData::serialize(Archive &ar, const unsigned int version)
		{
				ar & boost::serialization::make_nvp("ddparpath",ddparpath);
				ar & boost::serialization::make_nvp("dielpath",dielpath);
				ar & boost::serialization::make_nvp("dipoleSpacing", dipoleSpacing);
				ar & boost::serialization::make_nvp("T", T);
				ar & boost::serialization::make_nvp("freq", freq);
				ar & boost::serialization::make_nvp("nu",nu);
				ar & boost::serialization::make_nvp("sizep",sizep);
				ar & boost::serialization::make_nvp("reff",reff);
				ar & boost::serialization::make_nvp("volMeth", volMeth);
				ar & boost::serialization::make_nvp("dielMeth", dielMeth);
				ar & boost::serialization::make_nvp("shapeMeth", shapeMeth);
				ar & boost::serialization::make_nvp("angleMeth", angleMeth);
				ar & boost::serialization::make_nvp("stats", stats);
				ar & boost::serialization::make_nvp("tstats", tstats);
				ar & boost::serialization::make_nvp("OriAngleData", data);
				if (version >= 1)
					ar & boost::serialization::make_nvp("MieAngleData", miedata);

		}
		
		EXPORTINTERNAL(rtmath::tmatrix::tmStats::serialize);
		EXPORTINTERNAL(rtmath::tmatrix::tmData::serialize);
	}
}

//BOOST_CLASS_VERSION(rtmath::tmatrix::tmData, 1);

BOOST_CLASS_EXPORT_IMPLEMENT(rtmath::tmatrix::tmStats)
BOOST_CLASS_EXPORT_IMPLEMENT(rtmath::tmatrix::tmData)