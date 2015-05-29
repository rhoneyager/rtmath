#pragma once
#error "Unused header"
#include "../defs.h"
#include <vector>
#include <map>
#include <set>
#include <boost/shared_ptr.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/StdVector>

namespace rtmath {
	namespace ddscat {
		namespace stats {
			/// Contains definitions used in shapeFileStatsRotated.
			namespace shapeFileStatsRotatedEnums
			{
				/// Identifiers for columns of the dir_mat_t type
				enum dir_mat_t_columns { X, Y, Z, R, NUM_DIR_MAT_T_COLUMNS };
			};

			/// \brief Lightweight POD class that can be placed in a set
			/// \see shapeFileStatsBase for versioning information.
			/// \todo Split generation function to improve readability.
			class DLEXPORT_rtmath_ddscat shapeFileStatsRotated
			{
			public:
				/** \brief Trivial constructor for serialization. 
				*
				* Each shapeFileStats instance generates its own rotated stats directly.
				**/
				shapeFileStatsRotated();
				~shapeFileStatsRotated();
				/// Comparison operator used primarily for set-based ordering.
				bool operator<(const shapeFileStatsRotated &rhs) const;
				/// Rotation angle beta
				double beta;
				/// Rotation angle theta
				double theta;
				/// Rotation angle phi
				double phi;

				/**
				* \brief Indicates that a quantity is calculated per-material.
				*
				* Each column corresponds to a different direction: 0-x, 1-y, 2-z, 3-r.
				* Row 0 is for all materials combined. The remaining rows match the 
				* materials following the dielectric ordering.
				**/
				typedef Eigen::Matrix<float, Eigen::Dynamic, 
					shapeFileStatsRotatedEnums::NUM_DIR_MAT_T_COLUMNS> dir_mat_t;

				/// Lower bounds
				dir_mat_t min;
				/// Upper bounds
				dir_mat_t max;
				/// Sum
				dir_mat_t sum;
				/// Skewness
				dir_mat_t skewness;
				/// Kurtosis
				dir_mat_t kurtosis;
				/// 1st moment of inertia (mean)
				dir_mat_t mom1;
				/// 2nd moment of inertia
				dir_mat_t mom2;

				/**
				* \brief Indicates a matrix quantity, where the rows and columns 
				* represent the x, y and z directions, respectively.
				**/
				typedef Eigen::Matrix3f mat_3_t;

				/**
				* \brief Indicates a quantity that is calculated in 3x3 matrix form.
				*
				* Each vector element represents a different material. Element 0 
				* represents all constituents. It is divided this way because 
				* the materials all have different densities.
				*
				* The internal Eigen::Matrix3f is a 3x3 matrix, where the rows and 
				* columns represent the x, y and z directions, respectively.
				**/
				typedef std::vector<mat_3_t> vec_mat_t;

				/// \brief Indicates a stored 3-vector.
				typedef Eigen::Vector3f vec_3_t;
				/// \brief Indicates a stored 4-vector.
				typedef Eigen::Vector4f vec_4_t;

				/// Moment of inertia
				vec_mat_t mominert;
				/// Covariance
				vec_mat_t covariance;

				/// A potential energy-like function. Calculated against X, Y and Z only (no R).
				dir_mat_t PE;

				// For physically-united quantities, construct a shapeFileStatsRotatedView.

				/// Absolute value min
				vec_3_t abs_min;
				/// Absolute value max
				vec_3_t abs_max;
				/// Absolute value mean
				vec_3_t abs_mean;
				/// RMS mean (sqrt(variance))
				vec_4_t rms_mean;

				/// Absolute aspect ratios (max of abs_1 / max of abs_2)
				mat_3_t as_abs;
				/// Mean absolute aspect ratios (mean of abs_1 / mean of abs_2)
				mat_3_t as_abs_mean;
				/// RMS aspect ratios (sqrt(mom2(1)/mom2(2)))
				mat_3_t as_rms;

				/// \brief Cross-sectional areas (placeholder)
				/// \todo Implement cross-sectional area calculation using Voro++.
				Eigen::Vector3f areas;

				/// Should the stats be recalculated in the newest version?
				bool needsUpgrade() const;
			private:
				/// Internally indicates the most current class definition (used for upgrade check).
				static const int _maxVersion;
				/// Internally indicates the serialized version (used for upgrade check).
				int _currVersion;
			public:
				EIGEN_MAKE_ALIGNED_OPERATOR_NEW
			};
		}
	}
}

BOOST_CLASS_EXPORT_KEY(rtmath::ddscat::stats::shapeFileStatsRotated);

#define SHAPESTATS_ROTATED_VERSION 3
BOOST_CLASS_VERSION(rtmath::ddscat::stats::shapeFileStatsRotated, SHAPESTATS_ROTATED_VERSION);

