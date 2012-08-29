#include "../rtmath/Stdafx.h"
#include "../rtmath/matrixop.h"
#include "../rtmath/ddscat/shapestatsRotated.h"
#include "../rtmath/Serialization/shapestatsRotated_serialization.h"
#include "../rtmath/Serialization/serialization_macros.h"

#include <boost/serialization/base_object.hpp>
#include <boost/serialization/export.hpp>
#include <boost/serialization/version.hpp>
#include <boost/serialization/vector.hpp>

namespace boost
{
	namespace serialization
	{
		template <class Archive>
		void serialize(Archive & ar, rtmath::ddscat::shapeFileStatsRotated & g, const unsigned int version)
		{
			ar & boost::serialization::make_nvp("beta", g.beta);
			ar & boost::serialization::make_nvp("theta", g.theta);
			ar & boost::serialization::make_nvp("phi", g.phi);
			ar & boost::serialization::make_nvp("min", g.min);
			ar & boost::serialization::make_nvp("max", g.max);
			ar & boost::serialization::make_nvp("sum", g.sum);
			ar & boost::serialization::make_nvp("covariance", g.covariance);
			ar & boost::serialization::make_nvp("skewness", g.skewness);
			ar & boost::serialization::make_nvp("kurtosis", g.kurtosis);
			ar & boost::serialization::make_nvp("mom1", g.mom1);
			ar & boost::serialization::make_nvp("mom2", g.mom2);
			ar & boost::serialization::make_nvp("Moment_Inertia", g.mominert);
			ar & boost::serialization::make_nvp("PE", g.PE);
			ar & boost::serialization::make_nvp("abs_min", g.abs_min);
			ar & boost::serialization::make_nvp("abs_max", g.abs_max);
			ar & boost::serialization::make_nvp("abs_mean", g.abs_mean);
			ar & boost::serialization::make_nvp("rms_mean", g.rms_mean);
			ar & boost::serialization::make_nvp("as_abs", g.as_abs);
			ar & boost::serialization::make_nvp("as_abs_mean", g.as_abs_mean);
			ar & boost::serialization::make_nvp("as_rms", g.as_rms);
		}

		EXPORT(serialize,rtmath::ddscat::shapeFileStatsRotated);
	}
}

//BOOST_CLASS_VERSION(rtmath::ddscat::shapeFileStatsRotated, 0)
