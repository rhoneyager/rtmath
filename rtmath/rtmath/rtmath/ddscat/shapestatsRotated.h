#pragma once
#include <vector>
#include <map>
#include <set>
#include <boost/shared_ptr.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <boost/serialization/export.hpp>
#include <boost/serialization/access.hpp>
#include <boost/serialization/version.hpp>

namespace rtmath {
	namespace ddscat {

		// Lightweight POD class that can be placed in a set
		class shapeFileStatsRotated
		{
			friend class ::boost::serialization::access;
			template<class Archive>
			void serialize(Archive & ar, const unsigned int version);
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
			// for components, col 0-x,1-y,2-z,3-r. row is for each material, 0 row is out of all
			Eigen::Matrix<float, Eigen::Dynamic, 4>
				min, max, sum, skewness, kurtosis,
				mom1, mom2;
			// vectors of 3x3 matrices
			std::vector<Eigen::Matrix3f> mominert, covariance;
			// Row vectors for each material, cols are x, y, z
			Eigen::Matrix<float, Eigen::Dynamic, 3> PE;

			Eigen::Matrix3f abs_min, abs_max, abs_mean, rms_mean;

			// Aspect ratios
			Eigen::Matrix3f as_abs, as_abs_mean, as_rms;

			// Cross-sectional areas
			Eigen::Vector3f areas;

			/// Should the stats be recalculated in the newest version?
			bool needsUpgrade() const;
		private:
			static const unsigned int _maxVersion;
			unsigned int _currVersion;
		public:
			EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		};
	}
}

BOOST_CLASS_EXPORT_KEY(rtmath::ddscat::shapeFileStatsRotated);

#define SHAPESTATS_ROTATED_VERSION 2
BOOST_CLASS_VERSION(rtmath::ddscat::shapeFileStatsRotated, SHAPESTATS_ROTATED_VERSION);

