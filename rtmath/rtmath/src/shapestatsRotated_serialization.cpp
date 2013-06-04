#include "../rtmath/Stdafx.h"
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/export.hpp>
#include <boost/serialization/version.hpp>
#include <boost/serialization/vector.hpp>
#include "../rtmath/ddscat/shapestatsRotated.h"
#include "../rtmath/Serialization/shapestatsRotated_serialization.h"
#include "../rtmath/Serialization/serialization_macros.h"
#include "../rtmath/Serialization/eigen_serialization.h"



namespace boost
{
	namespace serialization
	{
		template <class Archive>
		void serialize(Archive & ar, rtmath::ddscat::shapeFileStatsRotated & g, const unsigned int version)
		{
			ar & make_nvp("beta", g.beta);
			ar & make_nvp("theta", g.theta);
			ar & make_nvp("phi", g.phi);
			ar & make_nvp("min", g.min);
			ar & make_nvp("max", g.max);
			ar & make_nvp("sum", g.sum);
			ar & make_nvp("covariance", g.covariance);
			ar & make_nvp("skewness", g.skewness);
			ar & make_nvp("kurtosis", g.kurtosis);

			// vector<vector4f> needs extra help
			ar & make_nvp("mom1", g.mom1);
			ar & make_nvp("mom2", g.mom2);
			//auto sV4f = [&](

			ar & make_nvp("Moment_Inertia", g.mominert);
			ar & make_nvp("PE", g.PE);
			ar & make_nvp("abs_min", g.abs_min);
			ar & make_nvp("abs_max", g.abs_max);
			ar & make_nvp("abs_mean", g.abs_mean);
			ar & make_nvp("rms_mean", g.rms_mean);
			ar & make_nvp("as_abs", g.as_abs);
			ar & make_nvp("as_abs_mean", g.as_abs_mean);
			ar & make_nvp("as_rms", g.as_rms);
			ar & make_nvp("areas", g.areas);
		}

		EXPORT(serialize,rtmath::ddscat::shapeFileStatsRotated);
	}
}

//BOOST_CLASS_VERSION(rtmath::ddscat::shapeFileStatsRotated, 1)
