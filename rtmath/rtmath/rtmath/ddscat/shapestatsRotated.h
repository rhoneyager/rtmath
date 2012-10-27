#pragma once
#include <vector>
#include <map>
#include <set>
#include <boost/shared_ptr.hpp>
#include "shapefile.h"
#include <Eigen/Core>
#include <Eigen/Dense>
#include<Eigen/StdVector>

namespace rtmath {
	//class matrixop;
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
			Eigen::Vector4f min, max, sum, skewness, kurtosis;
			// Moments
			std::vector<Eigen::Vector4f,Eigen::aligned_allocator<Eigen::Vector4f> > 
				mom1, mom2;
			std::vector<Eigen::Matrix3f> mominert, covariance;
			std::vector<Eigen::Vector3f> PE;

			Eigen::Vector4f abs_min, abs_max, abs_mean, rms_mean;

			// Aspect ratios
			Eigen::Matrix3f as_abs, as_abs_mean, as_rms;

			// Cross-sectional areas
			Eigen::Vector3f areas;
			EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		};
	}
}

