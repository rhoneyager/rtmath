#include "Stdafx-ddscat.h"
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/export.hpp>
#include <boost/serialization/version.hpp>
#include <boost/serialization/shared_ptr.hpp>
#include <boost/serialization/set.hpp>
#include "../rtmath/ddscat/shapefile.h"
#include "../rtmath/ddscat/shapestats.h"
#include "../rtmath/Serialization/eigen_serialization.h"
#include "../rtmath/Serialization/serialization_macros.h"

namespace rtmath
{
	namespace ddscat
	{
		template <class Archive>
		void shapeFileStatsBase::volumetric::serialize(Archive & ar, const unsigned int version)
		{
			ar & boost::serialization::make_nvp("V", V);
			ar & boost::serialization::make_nvp("aeff_V", aeff_V);
			ar & boost::serialization::make_nvp("SA", SA);
			ar & boost::serialization::make_nvp("aeff_SA", aeff_SA);
			ar & boost::serialization::make_nvp("f", f);
		}

		template <class Archive>
		void shapeFileStatsBase::serialize(Archive & ar, const unsigned int version)
		{
			_currVersion = version;
			ar & boost::serialization::make_nvp("shapefile", _shp);

			ar & boost::serialization::make_nvp("beta", beta);
			ar & boost::serialization::make_nvp("theta", theta);
			ar & boost::serialization::make_nvp("phi", phi);
			ar & boost::serialization::make_nvp("Effective_Rotation", rot);
			ar & boost::serialization::make_nvp("Inverse_Effective_Rotation", invrot);

			ar & boost::serialization::make_nvp("b_min", b_min);
			ar & boost::serialization::make_nvp("b_max", b_max);
			ar & boost::serialization::make_nvp("b_mean", b_mean);

			ar & boost::serialization::make_nvp("V_cell_const", V_cell_const);
			ar & boost::serialization::make_nvp("V_dipoles_const", V_dipoles_const);
			ar & boost::serialization::make_nvp("aeff_dipoles_const", aeff_dipoles_const);

			ar & boost::serialization::make_nvp("max_distance", max_distance);

			ar & boost::serialization::make_nvp("Scircum_sphere", Scircum_sphere);
			ar & boost::serialization::make_nvp("Sconvex_hull", Sconvex_hull);
			ar & boost::serialization::make_nvp("SVoronoi_hull", SVoronoi_hull);
			ar & boost::serialization::make_nvp("Sellipsoid_max", Sellipsoid_max);
			ar & boost::serialization::make_nvp("Sellipsoid_rms", Sellipsoid_rms);


			ar & boost::serialization::make_nvp("Rotation_Dependent", rotations);
		}

		template <class Archive>
		void shapeFileStats::serialize(Archive & ar, const unsigned int version)
		{
			ar & boost::serialization::make_nvp(
				"rtmath_ddscat_shapeFileStatsBase",
				boost::serialization::base_object<rtmath::ddscat::shapeFileStatsBase>(*this));
		}

		EXPORTINTERNAL(rtmath::ddscat::shapeFileStatsBase::volumetric::serialize);
		EXPORTINTERNAL(rtmath::ddscat::shapeFileStatsBase::serialize);
		EXPORTINTERNAL(rtmath::ddscat::shapeFileStats::serialize);
	}
}


