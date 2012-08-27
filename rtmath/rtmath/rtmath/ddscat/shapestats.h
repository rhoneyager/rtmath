#pragma once
#include <vector>
#include <map>
#include <set>
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
#include <boost/shared_ptr.hpp>
#include <boost/enable_shared_from_this.hpp>
#include "../matrixop.h"
#include "shapefile.h"

namespace rtmath {
	namespace ddscat {

		// Lightweight POD class that can be placed in a set
		class shapeFileStatsRotated
		{
		public:
			shapeFileStatsRotated();
			~shapeFileStatsRotated();
			bool operator<(const shapeFileStatsRotated &rhs) const;
			double beta;
			double theta;
			double phi;
			// Derived stats quantities
			// PE is a potential energy-like function.
			// All of the vector<matrixop> quantities are split by dielectric material.
			// This is because they have different densities. Coord zero corresponds 
			// to all of the dipoles simply combined.
			// For physically-united quantities, construct a shapeFileStatsRotatedView.

			// After normalization
			matrixop min, max, sum, skewness, kurtosis;
			// Moments
			std::vector<matrixop> mom1, mom2, mominert;
			std::vector<matrixop> covariance, PE;

			matrixop abs_min, abs_max, abs_mean, rms_mean;

			// Aspect ratios
			matrixop as_abs, as_abs_mean, as_rms;
		private:
			//bool _valid;
			friend class boost::serialization::access;
			template<class Archive>
				void serialize(Archive & ar, const unsigned int version)
				{
					ar & BOOST_SERIALIZATION_NVP(beta);
					ar & BOOST_SERIALIZATION_NVP(theta);
					ar & BOOST_SERIALIZATION_NVP(phi);
					ar & BOOST_SERIALIZATION_NVP(min);
					ar & BOOST_SERIALIZATION_NVP(max);
					ar & BOOST_SERIALIZATION_NVP(sum);
					ar & BOOST_SERIALIZATION_NVP(covariance);
					ar & BOOST_SERIALIZATION_NVP(skewness);
					ar & BOOST_SERIALIZATION_NVP(kurtosis);
					ar & BOOST_SERIALIZATION_NVP(mom1);
					ar & BOOST_SERIALIZATION_NVP(mom2);
					ar & boost::serialization::make_nvp("Moment_Inertia", mominert);
					ar & BOOST_SERIALIZATION_NVP(PE);
					ar & BOOST_SERIALIZATION_NVP(abs_min);
					ar & BOOST_SERIALIZATION_NVP(abs_max);
					ar & BOOST_SERIALIZATION_NVP(abs_mean);
					ar & BOOST_SERIALIZATION_NVP(rms_mean);
					ar & BOOST_SERIALIZATION_NVP(as_abs);
					ar & BOOST_SERIALIZATION_NVP(as_abs_mean);
					ar & BOOST_SERIALIZATION_NVP(as_rms);

				}
		};

		class shapeFileStatsBase
		{
		public:
			// Function that, if the shapefile referenced is not loaded, reloads the shapefile
			// Required for hulling or stats adding operations
			bool load();

			// Set rotation matrix, with each value in degrees
			//void setRot(double beta, double theta, double phi);
			void calcStatsBase();
			// calcStatsRot calculates the stats RELATIVE to the shapefile default rot.
			void calcStatsRot(double beta, double theta, double phi);

			// rot is the effective rotation designated by the choice of a1 and a2
			matrixop rot, invrot;
			double beta, theta, phi;

			// The constant multipliers! d is unknown!
			double V_cell_const, V_dipoles_const;
			double aeff_dipoles_const;

			// These require convex hull calculations
			double max_distance;
			double a_circum_sphere;
			double V_circum_sphere;
			double SA_circum_sphere;
			
			double V_convex_hull;
			double aeff_V_convex_hull;
			double SA_convex_hull;
			double aeff_SA_convex_hull;

			// Special stats calculated only in default orientation
			//
			double V_ellipsoid_max;
			double aeff_ellipsoid_max;
			double V_ellipsoid_rms;
			double aeff_ellipsoid_rms;

			// Extend to get volume fractions
			double f_circum_sphere;
			double f_convex_hull;
			double f_ellipsoid_max;
			double f_ellipsoid_rms;

			static const unsigned int _maxVersion;
			unsigned int _currVersion;

			// Before normalization and rotation
			matrixop b_min, b_max, b_mean;

			std::set<boost::shared_ptr<shapeFileStatsRotated> > rotations;

			// The object
			boost::shared_ptr<shapefile> _shp;
		protected:
			shapeFileStatsBase();
			virtual ~shapeFileStatsBase();
			
			bool _valid;
		private:
			friend class boost::serialization::access;
			template<class Archive>
				void serialize(Archive & ar, const unsigned int version)
				{
					_currVersion = version;
					ar & boost::serialization::make_nvp("shapefile", _shp);

					ar & BOOST_SERIALIZATION_NVP(beta);
					ar & BOOST_SERIALIZATION_NVP(theta);
					ar & BOOST_SERIALIZATION_NVP(phi);
					ar & boost::serialization::make_nvp("Effective_Rotation", rot);
					ar & boost::serialization::make_nvp("Inverse_Effective_Rotation", invrot);

					ar & BOOST_SERIALIZATION_NVP(b_min);
					ar & BOOST_SERIALIZATION_NVP(b_max);
					ar & BOOST_SERIALIZATION_NVP(b_mean);

					ar & BOOST_SERIALIZATION_NVP(V_cell_const);
					ar & BOOST_SERIALIZATION_NVP(V_dipoles_const);
					ar & BOOST_SERIALIZATION_NVP(aeff_dipoles_const);

					switch (version)
					{
					default:
					case 0:
						ar & BOOST_SERIALIZATION_NVP(max_distance);
						ar & BOOST_SERIALIZATION_NVP(a_circum_sphere);
						ar & BOOST_SERIALIZATION_NVP(V_circum_sphere);
						ar & BOOST_SERIALIZATION_NVP(SA_circum_sphere);
						ar & BOOST_SERIALIZATION_NVP(V_convex_hull);
						ar & BOOST_SERIALIZATION_NVP(aeff_V_convex_hull);
						ar & BOOST_SERIALIZATION_NVP(SA_convex_hull);
						ar & BOOST_SERIALIZATION_NVP(aeff_SA_convex_hull);
						ar & BOOST_SERIALIZATION_NVP(V_ellipsoid_max);
						ar & BOOST_SERIALIZATION_NVP(aeff_ellipsoid_max);
						ar & BOOST_SERIALIZATION_NVP(V_ellipsoid_rms);
						ar & BOOST_SERIALIZATION_NVP(aeff_ellipsoid_rms);
						ar & BOOST_SERIALIZATION_NVP(f_circum_sphere);
						ar & BOOST_SERIALIZATION_NVP(f_convex_hull);
						ar & BOOST_SERIALIZATION_NVP(f_ellipsoid_max);
						ar & BOOST_SERIALIZATION_NVP(f_ellipsoid_rms);
						break;
					}
					
					ar & boost::serialization::make_nvp("Rotation_Dependent", rotations);
				}
		};

		class shapeFileStats : public shapeFileStatsBase
		{
		public:
			shapeFileStats();
			shapeFileStats(const shapefile &shp);
			shapeFileStats(const boost::shared_ptr<const shapefile> &shp);

		private:
			friend class boost::serialization::access;
			template<class Archive>
				void serialize(Archive & ar, const unsigned int version)
				{
					ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(shapeFileStatsBase);
				}

		public:
			static boost::shared_ptr<shapeFileStats> genStats(
				const std::string &shpfile, const std::string &statsfile = "");
		};

	}
}

BOOST_CLASS_VERSION(rtmath::ddscat::shapeFileStatsRotated, 0)
BOOST_CLASS_VERSION(rtmath::ddscat::shapeFileStatsBase, 0)

//BOOST_CLASS_EXPORT_KEY(rtmath::ddscat::shapeFileStatsRotated)
//BOOST_CLASS_EXPORT_KEY(rtmath::ddscat::shapeFileStatsBase)
BOOST_CLASS_EXPORT_KEY(rtmath::ddscat::shapeFileStats)
