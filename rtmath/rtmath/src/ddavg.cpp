#include "Stdafx-ddscat.h"
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <algorithm>
#include <functional>
#include <boost/shared_ptr.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>


#include "../rtmath/ddscat/ddweights.h"
#include "../rtmath/ddscat/rotations.h"
#include "../rtmath/ddscat/ddUtil.h"
#include "../rtmath/error/debug.h"
#include "../rtmath/error/error.h"

namespace rtmath {
	namespace weights {
		ddOutputAvg::~ddOutputAvg() {}

		ddOutputAvg::ddOutputAvg(boost::shared_ptr<OrientationWeights3d> wtr) : weighter(wtr) {}

		void ddOutputAvg::clear() { averagers.clear(); }

		void ddOutputAvg::applyAvgBeta()
		{
			// Add a function to the end of the averaging deck that takes the input data 
			// (coordinates, current weights, ...) and averages over all betas.
			
			static auto doBeta = [](
				const Eigen::MatrixXf &orientations, 
				const Eigen::MatrixXf &weights,
				const Eigen::MatrixXf &data, 
				Eigen::MatrixXf &outOrientations,
				Eigen::MatrixXf &outWeights,
				Eigen::MatrixXf &outData)
			{
				// Do binning based on beta coordinate.
				// Set precision / tolerance to 1.e-4 for selecting matching orientations.
				std::map<float, float> sumWts;
				for (size_t i = 0; i < (size_t)orientations.rows(); ++i)
				{
					float beta = orientations(i, 0);
					float theta = orientations(i, 1); // Also count these to see how the output gets resized
					float phi = orientations(i, 2);
					float wt = weights(i);
					auto it = std::find_if(sumWts.begin(), sumWts.end(), [&](std::pair<float, float> t) {
						float tol = abs(t.first - beta);
						if (tol < 1.e-4) return true;
						return false;
					});
					if (it == sumWts.end()) {
						sumWts[beta] = wt;
					} else {
						it->second += wt;
					}
				}
			};

			averagers.push_back(doBeta);
		}
	}
}
