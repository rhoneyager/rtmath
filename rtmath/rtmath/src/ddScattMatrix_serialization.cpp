#include "Stdafx-ddscat_base.h"
#include "../rtmath/ddscat/ddScattMatrix.h"

#include "../rtmath/Serialization/serialization_macros.h"

#include <boost/serialization/base_object.hpp>
#include <boost/serialization/export.hpp>
#include <boost/serialization/version.hpp>
#include <boost/serialization/set.hpp>
#include <boost/serialization/shared_ptr.hpp>
#include <boost/serialization/complex.hpp>

namespace rtmath
{
	namespace ddscat
	{
		template<class Archive>
		void ddScattMatrixConnector::serialize(Archive &ar, const unsigned int version)
		{
			ar & boost::serialization::make_nvp("e01x", e01x);
			ar & boost::serialization::make_nvp("e01y", e01y);
			ar & boost::serialization::make_nvp("e01z", e01z);
			ar & boost::serialization::make_nvp("e02x", e02x);
			ar & boost::serialization::make_nvp("e02y", e02y);
			ar & boost::serialization::make_nvp("e02z", e02z);
		}

		template<class Archive>
		void ddScattMatrix::serialize(Archive &ar, const unsigned int version)
		{
			ar & boost::serialization::make_nvp("Polarization", _pol);
			ar & boost::serialization::make_nvp("Frequency", _freq);
			ar & boost::serialization::make_nvp("Theta", _theta);
			ar & boost::serialization::make_nvp("Theta_0", _thetan);
			ar & boost::serialization::make_nvp("Phi", _phi);
			ar & boost::serialization::make_nvp("Phi_0", _phin);

			ar & boost::serialization::make_nvp("Pnn", _Pnn);
		}

		template<class Archive>
		void ddScattMatrixF::serialize(Archive &ar, const unsigned int version)
		{
			ar & boost::serialization::make_nvp(
				"rtmath_ddscat_ddScattMatrix",
				boost::serialization::base_object<rtmath::ddscat::ddScattMatrix>(*this));
			ar & boost::serialization::make_nvp("f", _f);
			ar & boost::serialization::make_nvp("S", _s);
			ar & boost::serialization::make_nvp("frame", frame);
		}

		template<class Archive>
		void ddScattMatrixP::serialize(Archive &ar, const unsigned int version)
		{
			ar & boost::serialization::make_nvp(
				"rtmath_ddscat_ddScattMatrix",
				boost::serialization::base_object<rtmath::ddscat::ddScattMatrix>(*this));
		}

		EXPORTINTERNAL(rtmath::ddscat::ddScattMatrixConnector::serialize);
		EXPORTINTERNAL(rtmath::ddscat::ddScattMatrix::serialize);
		EXPORTINTERNAL(rtmath::ddscat::ddScattMatrixF::serialize);
		EXPORTINTERNAL(rtmath::ddscat::ddScattMatrixP::serialize);
	}
}

//BOOST_CLASS_EXPORT_IMPLEMENT(rtmath::ddscat::shapefile)

BOOST_CLASS_EXPORT_IMPLEMENT(rtmath::ddscat::ddScattMatrixConnector);
BOOST_CLASS_EXPORT_IMPLEMENT(rtmath::ddscat::ddScattMatrix);
BOOST_CLASS_EXPORT_IMPLEMENT(rtmath::ddscat::ddScattMatrixF);
BOOST_CLASS_EXPORT_IMPLEMENT(rtmath::ddscat::ddScattMatrixP);
