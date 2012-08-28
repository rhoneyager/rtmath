#include "../rtmath/Stdafx.h"
#include "../rtmath/matrixop.h"
#include "../rtmath/ddscat/shapestats.h"
#include "../rtmath/Serialization/shapestats_serialization.h"
#include "../rtmath/Serialization/serialization_macros.h"

#include <boost/serialization/base_object.hpp>
#include <boost/serialization/export.hpp>
#include <boost/serialization/version.hpp>

namespace boost
{
	namespace serialization
	{
		template <class Archive>
		void serialize(Archive & ar, rtmath::ddscat::shapeFileStatsBase & g, const unsigned int version)
		{
			g._currVersion = version;
			ar & boost::serialization::make_nvp("shapefile", _shp);

			ar & boost::serialization::make_nvp("beta", g.beta);
			ar & boost::serialization::make_nvp("theta", g.theta);
			ar & boost::serialization::make_nvp("phi", g.phi);
			ar & boost::serialization::make_nvp("Effective_Rotation", g.rot);
			ar & boost::serialization::make_nvp("Inverse_Effective_Rotation", g.invrot);

			ar & boost::serialization::make_nvp("b_min", g.b_min);
			ar & boost::serialization::make_nvp("b_max", g.b_max);
			ar & boost::serialization::make_nvp("b_mean", g.b_mean);

			ar & boost::serialization::make_nvp("V_cell_const", g.V_cell_const);
			ar & boost::serialization::make_nvp("V_dipoles_const", g.V_dipoles_const);
			ar & boost::serialization::make_nvp("aeff_dipoles_const", g.aeff_dipoles_const);

			switch (version)
			{
			default:
			case 0:
				ar & boost::serialization::make_nvp("max_distance", g.max_distance);
				ar & boost::serialization::make_nvp("a_circum_sphere", g.a_circum_sphere);
				ar & boost::serialization::make_nvp("V_circum_sphere", g.V_circum_sphere);
				ar & boost::serialization::make_nvp("SA_circum_sphere", g.SA_circum_sphere);
				ar & boost::serialization::make_nvp("V_convex_hull", g.V_convex_hull);
				ar & boost::serialization::make_nvp("aeff_V_convex_hull", g.aeff_V_convex_hull);
				ar & boost::serialization::make_nvp("SA_convex_hull", g.SA_convex_hull);
				ar & boost::serialization::make_nvp("aeff_SA_convex_hull", g.aeff_SA_convex_hull);
				ar & boost::serialization::make_nvp("V_ellipsoid_max", g.V_ellipsoid_max);
				ar & boost::serialization::make_nvp("aeff_ellipsoid_max", g.aeff_ellipsoid_max);
				ar & boost::serialization::make_nvp("V_ellipsoid_rms", g.V_ellipsoid_rms);
				ar & boost::serialization::make_nvp("aeff_ellipsoid_rms", g.aeff_ellipsoid_rms);
				ar & boost::serialization::make_nvp("f_circum_sphere", g.f_circum_sphere);
				ar & boost::serialization::make_nvp("f_convex_hull", g.f_convex_hull);
				ar & boost::serialization::make_nvp("f_ellipsoid_max", g.f_ellipsoid_max);
				ar & boost::serialization::make_nvp("f_ellipsoid_rms", g.f_ellipsoid_rms);
				break;
			}

			ar & boost::serialization::make_nvp("Rotation_Dependent", g.rotations);
		}

		template <class Archive>
		void serialize(Archive & ar, rtmath::ddscat::shapeFileStats & g, const unsigned int version)
		{
			ar & boost::serialization::base_object<rtmath::ddscat::shapeFileStatsBase>(g);
		}

		EXPORT(rtmath::ddscat::shapeFileStatsBase);
		EXPORT(rtmath::ddscat::shapeFileStats);
	}
}

//BOOST_CLASS_VERSION(rtmath::ddscat::shapeFileStatsBase, 0)
