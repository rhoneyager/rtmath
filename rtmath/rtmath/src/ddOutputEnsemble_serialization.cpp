#include "../rtmath/Stdafx.h"
#include "../rtmath/ddscat/ddOutput.h"
#include "../rtmath/ddscat/ddOutputEnsemble.h"

#include "../rtmath/Serialization/serialization_macros.h"

#include <boost/serialization/base_object.hpp>
#include <boost/serialization/export.hpp>
#include <boost/serialization/version.hpp>
#include <boost/serialization/string.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/set.hpp>
#include <boost/serialization/shared_ptr.hpp>
#include <boost/serialization/complex.hpp>

namespace rtmath
{
	namespace ddscat
	{
		template<class Archive>
		void ddOutputEnsemble::serialize(Archive &ar, const unsigned int version)
		{
			/*
			ar & boost::serialization::make_nvp("Description", description);
			ar & boost::serialization::make_nvp("Tags", tags);
			ar & boost::serialization::make_nvp("Sources", sources);
			ar & boost::serialization::make_nvp("Frequency", freq);
			ar & boost::serialization::make_nvp("aeff", aeff);
			ar & boost::serialization::make_nvp("Refractive Indices", ms);

			// stats and shape are handled by the loadShape function
			ar & boost::serialization::make_nvp("shapeHash", shapeHash);
			
			// Generator may point to nothing (indicates original ddscat data)
			ar & boost::serialization::make_nvp("generator", generator);

			ar & boost::serialization::make_nvp("avg", avg);
			ar & boost::serialization::make_nvp("scas", scas);
			ar & boost::serialization::make_nvp("fmls", fmls);
			ar & boost::serialization::make_nvp("Weights", weights);
			*/
		}

		EXPORTINTERNAL(rtmath::ddscat::ddOutputEnsemble::serialize);
	}
}

BOOST_CLASS_EXPORT_IMPLEMENT(rtmath::ddscat::ddOutputEnsemble);
