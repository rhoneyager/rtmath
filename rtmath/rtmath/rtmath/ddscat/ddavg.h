#pragma once
#include "../defs.h"
#include <string>
#include <vector>
#include <functional>
#include <boost/shared_ptr.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>

namespace rtmath
{
	namespace weights { class OrientationWeights3d; }

	namespace weights
	{
		/** \brief Computes averages of ddOutput ori objects given certain constraints.
		*
		* Averages may be done over a specified range, such as averaging over all betas, 
		* or betas and phis, or over all orientation directions. This is accomplished by 
		* generating the weights for each individual orientation, and then performing a 
		* weighted average along the scanning direction.
		**/
		class DLEXPORT_rtmath_ddscat ddOutputAvg
		{
			boost::shared_ptr<OrientationWeights3d> weighter;
			/// These functions are applied sequentially to generate the final weights
			std::vector<std::function<void()> > averagers;
		public:
			ddOutputAvg(boost::shared_ptr<OrientationWeights3d>);
			virtual ~ddOutputAvg();

			void applyAvgBeta();
			void applyAvgTheta();
			void applyAvgPhi();
			void avgAll();
			void clear();

			/// Average the data (second parameter) with the given orientations (first parameter).
			void doAvg(const Eigen::MatrixXf &orientations, const Eigen::MatrixXf &data,
				Eigen::MatrixXf &outCoords, Eigen::MatrixXf &outwts, Eigen::MatrixXf &outdata) const;
		};
	}
}

