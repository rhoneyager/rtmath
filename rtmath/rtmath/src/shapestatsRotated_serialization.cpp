#include "../rtmath/Stdafx.h"
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/export.hpp>
#include <boost/serialization/version.hpp>
#include <boost/serialization/vector.hpp>
#include "../rtmath/ddscat/shapestatsRotated.h"
#include "../rtmath/Serialization/serialization_macros.h"
#include "../rtmath/Serialization/eigen_serialization.h"

namespace rtmath
{
	namespace ddscat
	{
		template <class Archive>
		void shapeFileStatsRotated::serialize(Archive & ar, const unsigned int version)
		{
			using boost::serialization::make_nvp;
			ar & make_nvp("beta", beta);
			ar & make_nvp("theta", theta);
			ar & make_nvp("phi", phi);
			ar & make_nvp("min", min);
			ar & make_nvp("max", max);
			ar & make_nvp("sum", sum);
			ar & make_nvp("covariance", covariance);
			ar & make_nvp("skewness", skewness);
			ar & make_nvp("kurtosis", kurtosis);

			// vector<vector4f> needs extra help
			ar & make_nvp("mom1", mom1);
			ar & make_nvp("mom2", mom2);
			//auto sV4f = [&](

			ar & make_nvp("Moment_Inertia", mominert);
			ar & make_nvp("PE", PE);
			ar & make_nvp("abs_min", abs_min);
			ar & make_nvp("abs_max", abs_max);
			ar & make_nvp("abs_mean", abs_mean);
			ar & make_nvp("rms_mean", rms_mean);
			ar & make_nvp("as_abs", as_abs);
			ar & make_nvp("as_abs_mean", as_abs_mean);
			ar & make_nvp("as_rms", as_rms);
			ar & make_nvp("areas", areas);
		}

		EXPORTINTERNAL(rtmath::ddscat::shapeFileStatsRotated::serialize);
	}
}
BOOST_CLASS_EXPORT_IMPLEMENT(rtmath::ddscat::shapeFileStatsRotated)
//BOOST_CLASS_VERSION(rtmath::ddscat::shapeFileStatsRotated, 1)
