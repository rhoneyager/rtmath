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

	namespace ddscat {
		class ddOutput; class ddOriData;
		namespace weights
		{
			class OrientationWeights3d;
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
			public:
				ddOutputAvg(boost::shared_ptr<OrientationWeights3d>);
				virtual ~ddOutputAvg();

				void doAvgAll(const ddscat::ddOutput *in,
					boost::shared_ptr<ddscat::ddOutput> &out, Eigen::MatrixXf &outwts) const;
			};
		}
	}
}

