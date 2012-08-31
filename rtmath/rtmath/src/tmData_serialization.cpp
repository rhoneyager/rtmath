#include "../rtmath/Stdafx.h"
#include "../rtmath/ddscat/tmData.h"
#include "../rtmath/Serialization/tmData_serialization.h"
#include "../rtmath/Serialization/shapestats_serialization.h"
#include "../../deps/tmatrix/src/headers/tmatrix.h"
#include "../rtmath/Serialization/serialization_macros.h"

#include <boost/serialization/base_object.hpp>
#include <boost/serialization/export.hpp>
#include <boost/serialization/version.hpp>
#include <boost/serialization/set.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/shared_ptr.hpp>
#include <boost/serialization/string.hpp>
#include <boost/serialization/complex.hpp>

namespace boost
{
	namespace serialization
	{
		template<class Archive>
		void serialize(Archive &ar, rtmath::tmatrix::tmIn &g, const unsigned int version)
		{
			ar & boost::serialization::make_nvp("axi", g.axi);
			ar & boost::serialization::make_nvp("rat", g.rat);
			ar & boost::serialization::make_nvp("lam", g.lam);
			ar & boost::serialization::make_nvp("mrr", g.mrr);
			ar & boost::serialization::make_nvp("mri", g.mri);
			ar & boost::serialization::make_nvp("eps", g.eps);
			ar & boost::serialization::make_nvp("ddelt", g.ddelt);
			ar & boost::serialization::make_nvp("alpha", g.alpha);
			ar & boost::serialization::make_nvp("beta", g.beta);
			ar & boost::serialization::make_nvp("thet0", g.thet0);
			ar & boost::serialization::make_nvp("thet", g.thet);
			ar & boost::serialization::make_nvp("phi0", g.phi0);
			ar & boost::serialization::make_nvp("phi", g.phi);
			ar & boost::serialization::make_nvp("np", g.np);
			ar & boost::serialization::make_nvp("ndgs", g.ndgs);
		}

		template<class Archive>
		void serialize(Archive &ar, rtmath::tmatrix::tmOut &g, const unsigned int version)
		{
			ar & boost::serialization::make_nvp("S", g.S);
			ar & boost::serialization::make_nvp("P", g.P);
		}

		template<class Archive>
		void serialize(Archive &ar, rtmath::tmatrix::tmRun &g, const unsigned int version)
		{
			ar & boost::serialization::make_nvp("in", g.invars);
			ar & boost::serialization::make_nvp("out", g.res);

		}

		template<class Archive>
		void serialize(Archive &ar, rtmath::tmatrix::tmData &g, const unsigned int version)
		{
				ar & boost::serialization::make_nvp("ddparpath",g.ddparpath);
				ar & boost::serialization::make_nvp("dipoleSpacing", g.dipoleSpacing);
				ar & boost::serialization::make_nvp("T", g.T);
				ar & boost::serialization::make_nvp("freq", g.freq);
				ar & boost::serialization::make_nvp("nu",g.nu);
				ar & boost::serialization::make_nvp("reff",g.reff);
				ar & boost::serialization::make_nvp("volMeth", g.volMeth);
				ar & boost::serialization::make_nvp("dielMeth", g.dielMeth);
				ar & boost::serialization::make_nvp("shapeMeth", g.shapeMeth);
				ar & boost::serialization::make_nvp("angleMeth", g.angleMeth);
				ar & boost::serialization::make_nvp("stats", g.stats);
				ar & boost::serialization::make_nvp("data", g.data);
		}

		EXPORT(serialize,rtmath::tmatrix::tmData);
	}
}

