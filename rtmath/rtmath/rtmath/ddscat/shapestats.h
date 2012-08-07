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

		class shapefile;

		// Lightweight POD class that can be placed in a set
		class shapeFileStatsRotated
		{
		public:
			shapeFileStatsRotated();
			~shapeFileStatsRotated();
			double beta;
			double theta;
			double phi;
			// Derived stats quantities
			// PE is a potential energy-like function.
			// PE = sum_i(x_i) (Note: masses assumed to be constant. use scaling factor)
			matrixop PE;

			// After normalization
			matrixop min, max, sum, skewness, kurtosis;
			// Moments
			matrixop mom1, mom2, mominert;
			matrixop covariance;

			matrixop abs_min, abs_max, abs_mean; // - abs_mean is also PE

			// Aspect ratios
			matrixop as_abs, as_abs_mean, as_rms;

			//void calc();
			bool operator<(const shapeFileStatsRotated &rhs) const;
		private:
			//bool _valid;
			friend class boost::serialization::access;
			template<class Archive>
				void serialize(Archive & ar, const unsigned int version)
				{
					ar & BOOST_SERIALIZATION_NVP(beta);
					ar & BOOST_SERIALIZATION_NVP(theta);
					ar & BOOST_SERIALIZATION_NVP(phi);
					ar & BOOST_SERIALIZATION_NVP(PE);
					ar & BOOST_SERIALIZATION_NVP(min);
					ar & BOOST_SERIALIZATION_NVP(max);
					ar & BOOST_SERIALIZATION_NVP(sum);
					ar & BOOST_SERIALIZATION_NVP(skewness);
					ar & BOOST_SERIALIZATION_NVP(kurtosis);
					ar & BOOST_SERIALIZATION_NVP(mom1);
					ar & BOOST_SERIALIZATION_NVP(mom2);
					ar & BOOST_SERIALIZATION_NVP(covariance);
					ar & BOOST_SERIALIZATION_NVP(abs_min);
					ar & BOOST_SERIALIZATION_NVP(abs_max);
					ar & BOOST_SERIALIZATION_NVP(abs_mean);
					ar & BOOST_SERIALIZATION_NVP(as_abs);
					ar & BOOST_SERIALIZATION_NVP(as_abs_mean);
					ar & BOOST_SERIALIZATION_NVP(as_rms);
					ar & boost::serialization::make_nvp("Moment_Inertia", mominert);
				}
		};

		class shapeFileStatsBase
		{
		public:
			inline size_t N() const {return _N;}
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

			double max_distance;
			static const unsigned int _maxVersion;
			unsigned int _currVersion;

			// Before normalization and rotation
			matrixop b_min, b_max, b_mean;
			

			std::set<shapeFileStatsRotated> rotations;
		protected:
			shapeFileStatsBase();
			virtual ~shapeFileStatsBase();
			
			size_t _N;// Number of dipoles (used in checking for a valid read)

			// The object
			boost::shared_ptr<shapefile> _shp;
			bool _valid;
		private:
			friend class boost::serialization::access;
			template<class Archive>
				void serialize(Archive & ar, const unsigned int version)
				{
					ar & boost::serialization::make_nvp("shapefile", _shp);
					if (version < 2)
						ar & boost::serialization::make_nvp("N", _N);
					
					ar & BOOST_SERIALIZATION_NVP(V_cell_const);
					ar & BOOST_SERIALIZATION_NVP(V_dipoles_const);
					ar & BOOST_SERIALIZATION_NVP(aeff_dipoles_const);

					switch (version)
					{
					default:
					case 1:
						ar & BOOST_SERIALIZATION_NVP(max_distance);
					case 0:
						break;
					}
					_currVersion = version;

					ar & BOOST_SERIALIZATION_NVP(beta);
					ar & BOOST_SERIALIZATION_NVP(theta);
					ar & BOOST_SERIALIZATION_NVP(phi);
					ar & boost::serialization::make_nvp("Effective_Rotation", rot);
					ar & boost::serialization::make_nvp("Inverse_Effective_Rotation", invrot);
					ar & BOOST_SERIALIZATION_NVP(b_min);
					ar & BOOST_SERIALIZATION_NVP(b_max);
					ar & BOOST_SERIALIZATION_NVP(b_mean);
					
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
		};

	}
}

BOOST_CLASS_VERSION(rtmath::ddscat::shapeFileStatsBase, 2)

BOOST_CLASS_EXPORT_KEY(rtmath::ddscat::shapeFileStatsRotated)
BOOST_CLASS_EXPORT_KEY(rtmath::ddscat::shapeFileStatsBase)
BOOST_CLASS_EXPORT_KEY(rtmath::ddscat::shapeFileStats)
