#pragma once
#include <boost/archive/xml_oarchive.hpp>
#include <boost/archive/xml_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/serialization/access.hpp>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/map.hpp>
#include <boost/serialization/set.hpp>
#include <boost/serialization/shared_ptr.hpp>
#include <boost/serialization/version.hpp>

#include "../ddscat/shapestats.h"

#include "matrixop_serialization.h"
#include "shapefile_serialization.h"

namespace boost
{
	namespace serialization
	{
		template <class Archive>
		void serialize(Archive & ar, rtmath::ddscat::shapeFileStatsBase & g, const unsigned int version)
		{
			g._currVersion = version;
			ar & boost::serialization::make_nvp("shapefile", _shp);

			ar & BOOST_SERIALIZATION_NVP(g.beta);
			ar & BOOST_SERIALIZATION_NVP(g.theta);
			ar & BOOST_SERIALIZATION_NVP(g.phi);
			ar & boost::serialization::make_nvp("Effective_Rotation", g.rot);
			ar & boost::serialization::make_nvp("Inverse_Effective_Rotation", g.invrot);

			ar & BOOST_SERIALIZATION_NVP(g.b_min);
			ar & BOOST_SERIALIZATION_NVP(g.b_max);
			ar & BOOST_SERIALIZATION_NVP(g.b_mean);

			ar & BOOST_SERIALIZATION_NVP(g.V_cell_const);
			ar & BOOST_SERIALIZATION_NVP(g.V_dipoles_const);
			ar & BOOST_SERIALIZATION_NVP(g.aeff_dipoles_const);

			switch (version)
			{
			default:
			case 0:
				ar & BOOST_SERIALIZATION_NVP(g.max_distance);
				ar & BOOST_SERIALIZATION_NVP(g.a_circum_sphere);
				ar & BOOST_SERIALIZATION_NVP(g.V_circum_sphere);
				ar & BOOST_SERIALIZATION_NVP(g.SA_circum_sphere);
				ar & BOOST_SERIALIZATION_NVP(g.V_convex_hull);
				ar & BOOST_SERIALIZATION_NVP(g.aeff_V_convex_hull);
				ar & BOOST_SERIALIZATION_NVP(g.SA_convex_hull);
				ar & BOOST_SERIALIZATION_NVP(g.aeff_SA_convex_hull);
				ar & BOOST_SERIALIZATION_NVP(g.V_ellipsoid_max);
				ar & BOOST_SERIALIZATION_NVP(g.aeff_ellipsoid_max);
				ar & BOOST_SERIALIZATION_NVP(g.V_ellipsoid_rms);
				ar & BOOST_SERIALIZATION_NVP(g.aeff_ellipsoid_rms);
				ar & BOOST_SERIALIZATION_NVP(g.f_circum_sphere);
				ar & BOOST_SERIALIZATION_NVP(g.f_convex_hull);
				ar & BOOST_SERIALIZATION_NVP(g.f_ellipsoid_max);
				ar & BOOST_SERIALIZATION_NVP(g.f_ellipsoid_rms);
				break;
			}

			ar & boost::serialization::make_nvp("Rotation_Dependent", g.rotations);
		}

		template <class Archive>
		void serialize(Archive & ar, rtmath::ddscat::shapeFileStats & g, const unsigned int version)
		{
			ar & boost::serialization::base_object<rtmath::ddscat::shapeFileStatsBase>(g);
		}
	}
}

//BOOST_CLASS_VERSION(rtmath::ddscat::shapeFileStatsBase, 0)

//BOOST_CLASS_EXPORT_KEY(rtmath::ddscat::shapeFileStatsRotated)
//BOOST_CLASS_EXPORT_KEY(rtmath::ddscat::shapeFileStatsBase)
//BOOST_CLASS_EXPORT_KEY(rtmath::ddscat::shapeFileStats)


//BOOST_CLASS_EXPORT_IMPLEMENT(rtmath::ddscat::shapeFileStatsRotated)
//BOOST_CLASS_EXPORT_IMPLEMENT(rtmath::ddscat::shapeFileStatsBase)
//BOOST_CLASS_EXPORT_IMPLEMENT(rtmath::ddscat::shapeFileStats)
