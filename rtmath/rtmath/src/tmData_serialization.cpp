#include "../rtmath/Stdafx.h"
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
#include "../rtmath/Serialization/tmData_serialization.h"
#include "../rtmath/Serialization/shapestats_serialization.h"
#include <tmatrix/tmatrix.h>
#include <tmatrix/tmatrix-serialization.h>
#include "../rtmath/Serialization/serialization_macros.h"
#include "../rtmath/mie/mie-serialization.h"
#include "../rtmath/mie/mie.h"


namespace boost
{
	namespace serialization
	{
		template<class Archive>
		void serialize(Archive &ar, rtmath::tmatrix::tmStats &g, const unsigned int version)
		{
				ar & boost::serialization::make_nvp("statmap",g.stats);
				//ar & boost::serialization::make_nvp("nested", g.nested);
		}
		EXPORT(serialize,rtmath::tmatrix::tmStats);

		template<class Archive>
		void serialize(Archive &ar, rtmath::tmatrix::tmData &g, const unsigned int version)
		{
				ar & boost::serialization::make_nvp("ddparpath",g.ddparpath);
				ar & boost::serialization::make_nvp("dielpath",g.dielpath);
				ar & boost::serialization::make_nvp("dipoleSpacing", g.dipoleSpacing);
				ar & boost::serialization::make_nvp("T", g.T);
				ar & boost::serialization::make_nvp("freq", g.freq);
				ar & boost::serialization::make_nvp("nu",g.nu);
				ar & boost::serialization::make_nvp("sizep",g.sizep);
				ar & boost::serialization::make_nvp("reff",g.reff);
				ar & boost::serialization::make_nvp("volMeth", g.volMeth);
				ar & boost::serialization::make_nvp("dielMeth", g.dielMeth);
				ar & boost::serialization::make_nvp("shapeMeth", g.shapeMeth);
				ar & boost::serialization::make_nvp("angleMeth", g.angleMeth);
				ar & boost::serialization::make_nvp("stats", g.stats);
				ar & boost::serialization::make_nvp("tstats", g.tstats);
				ar & boost::serialization::make_nvp("OriAngleData", g.data);
				if (version >= 1)
					ar & boost::serialization::make_nvp("MieAngleData", g.miedata);

		}

		EXPORT(serialize,rtmath::tmatrix::tmData);
	}
}

//BOOST_CLASS_VERSION(rtmath::tmatrix::tmData, 1);
