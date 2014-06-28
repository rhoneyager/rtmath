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

#include "../rtmath/ddscat/ddavg.h"
#include "../rtmath/ddscat/ddOriData.h"
#include "../rtmath/ddscat/ddOutput.h"
#include "../rtmath/ddscat/ddweights.h"
#include "../rtmath/ddscat/rotations.h"
#include "../rtmath/ddscat/ddUtil.h"
#include "../rtmath/error/debug.h"
#include "../rtmath/error/error.h"

namespace rtmath {
	namespace ddscat {
		namespace weights {
			ddOutputAvg::~ddOutputAvg() {}

			ddOutputAvg::ddOutputAvg(boost::shared_ptr<OrientationWeights3d> wtr) : weighter(wtr) {}
			/*
			void ddOutputAvg::doAvgBeta(const Eigen::MatrixXf &inOri,
			const Eigen::MatrixXf &data,
			Eigen::MatrixXf &outOri,
			Eigen::MatrixXf &outwts,
			Eigen::MatrixXf &outdata) const
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
			}
			else {
			it->second += wt;
			}
			}

			}
			*/

			void ddOutputAvg::doAvgAll(
				const ddscat::ddOutput *in,
				boost::shared_ptr<ddscat::ddOutput> &out,
				Eigen::MatrixXf &outwts) const
			{
				using namespace rtmath::ddscat;
				// If an avg table is specified, just copy it to the resulting object
				if (in->avgdata.hasAvg)
				{
					out = boost::shared_ptr<ddOutput>(new ddOutput(*in));
					outwts.resize(0, 0);
				}
				else {
					// Have to do averaging separately
					// The weights are provided already!
					out = boost::shared_ptr<ddOutput>(new ddOutput(*in));
					outwts.resize(in->oridata_d.rows(), 4);
					out->avgdata.avg.setZero();

					double cdf = 0;
					for (size_t i = 0; i < (size_t)in->oridata_d.rows(); ++i)
					{
						const auto od = in->oridata_d.block<1, ddOutput::stat_entries::NUM_STAT_ENTRIES_DOUBLES>(i, 0);
						const float beta = (float)od(ddOutput::stat_entries::BETA);
						const float theta = (float)od(ddOutput::stat_entries::THETA);
						const float phi = (float)od(ddOutput::stat_entries::PHI);

						double wt = weighter->getWeight(beta, theta, phi);
						auto ow = outwts.block<1, 4>(i, 0);
						ow(0) = beta; ow(1) = theta; ow(2) = phi; ow(3) = (float) wt;

						out->avgdata.avg = out->avgdata.avg + (od * wt);

						cdf += wt;
					}
					// Weights should sum to unity. If not, tweak to fix.
					out->avgdata.avg = out->avgdata.avg / cdf;
					if (!cdf) out->avgdata.avg.setZero();

					// Finally, set irrelevant fields to indicate avg output.
					out->avgdata.avg(ddOutput::stat_entries::BETA) = -1;
					out->avgdata.avg(ddOutput::stat_entries::THETA) = -1;
					out->avgdata.avg(ddOutput::stat_entries::PHI) = -1;
				}

			}
		}
	}
}
