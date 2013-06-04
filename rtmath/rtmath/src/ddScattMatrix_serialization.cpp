#include "../rtmath/Stdafx.h"
#include "../rtmath/ddscat/ddScattMatrix.h"
#include "../rtmath/Serialization/ddScattMatrix_serialization.h"

#include "../rtmath/Serialization/serialization_macros.h"

#include <boost/serialization/base_object.hpp>
#include <boost/serialization/export.hpp>
#include <boost/serialization/version.hpp>
#include <boost/serialization/set.hpp>
#include <boost/serialization/shared_ptr.hpp>
#include <boost/serialization/complex.hpp>

namespace boost
{
	namespace serialization
	{
		template<class Archive>
		void serialize(Archive &ar, rtmath::ddscat::ddScattMatrix &g, const unsigned int version)
		{
			ar & boost::serialization::make_nvp("Polarization", g._pol);
			ar & boost::serialization::make_nvp("Frequency", g._freq);
			ar & boost::serialization::make_nvp("Theta", g._theta);
			ar & boost::serialization::make_nvp("Theta_0", g._thetan);
			ar & boost::serialization::make_nvp("Phi", g._phi);
			ar & boost::serialization::make_nvp("Phi_0", g._phin);

			ar & boost::serialization::make_nvp("Pnn", g._Pnn);
		}

		template<class Archive>
		void serialize(Archive &ar, rtmath::ddscat::ddScattMatrixF &g, const unsigned int version)
		{
			ar & boost::serialization::make_nvp(
				"rtmath_ddscat_ddScattMatrix",
				boost::serialization::base_object<rtmath::ddscat::ddScattMatrix>(g));
			ar & boost::serialization::make_nvp("f", g._f);
			ar & boost::serialization::make_nvp("S", g._s);
		}

		template<class Archive>
		void serialize(Archive &ar, rtmath::ddscat::ddScattMatrixP &g, const unsigned int version)
		{
			ar & boost::serialization::make_nvp(
				"rtmath_ddscat_ddScattMatrix",
				boost::serialization::base_object<rtmath::ddscat::ddScattMatrix>(g));
		}

		EXPORT(serialize,rtmath::ddscat::ddScattMatrix);
		EXPORT(serialize,rtmath::ddscat::ddScattMatrixF);
		EXPORT(serialize,rtmath::ddscat::ddScattMatrixP);
	}
}

//BOOST_CLASS_EXPORT_IMPLEMENT(rtmath::ddscat::shapefile)

BOOST_CLASS_EXPORT_IMPLEMENT(rtmath::ddscat::ddScattMatrix)
BOOST_CLASS_EXPORT_IMPLEMENT(rtmath::ddscat::ddScattMatrixF)
BOOST_CLASS_EXPORT_IMPLEMENT(rtmath::ddscat::ddScattMatrixP)
