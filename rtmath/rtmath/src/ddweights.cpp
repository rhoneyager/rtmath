#include "../rtmath/Stdafx.h"
#include <iostream>
#include <boost/math/special_functions/erf.hpp>
#include <memory>
#include <cmath>
#include <boost/math/distributions/normal.hpp>
//#include "../rtmath/ddscat.h"
#include "../rtmath/ddweights.h"
#include "../rtmath/error/error.h"


// The ddscat code was becoming too large, so the weighting functions
// will go in this file.


namespace rtmath {
	namespace ddscat {

		void weights::getWeights(std::map<double,double> &weights) const
		{
			weights = _pointWeights;
		}

		gaussianPosWeights::~gaussianPosWeights()
		{
		}

		gaussianPosWeights::gaussianPosWeights(double sigma,
			const std::set<double> &points)
		{
			// TODO: totally need to multiline this error message
			if (points.size() < 3) 
				throw rtmath::debug::xBadInput("Fewer than three points specified when constructing Gaussian weighting function. While possible, poor performance would result.");
			_mu = 0;
			_sigma = sigma;
			// Iterate through each point in points
			std::set<double>::const_iterator it, ot, pt;
			for (it = points.begin(); it != points.end(); it++)
			{
				// Look at the point. If it is the first or the last,
				// it needs special handling because it is at the 
				// end on the interval. Otherwise, find the two middle
				// points between the adjacent points, and this area
				// will be the interval considered for weight calculation
				if (*it < 0) throw rtmath::debug::xAssert("Point < 0");
				double _a = 0, _b = 0;
				ot = it;
				ot++;
				if (it == points.begin())
				{
					_a = 0;
					_b = 0.5 * (*it + *ot);
				} else {
					pt = it;
					pt--; // Senseless if it == begin().
					if (ot == points.end())
					{
						_a = 0.5 * (*it + *pt);
						_b = 0; // Signify that we're doing the interval complement
					} else {
						// We're between the beginning and the end
						_a = 0.5 * (*it + *pt);
						_b = 0.5 * (*it + *ot);
					}
				}

				// Actually calculate stuff
				double p = 0, pa = 0, pb = 0;
				
				// Special handling needed as we don't accept values below 0...
				// mu must be zero for this technique to work, as I am 
				// exploiting symmetry
				boost::math::normal_distribution<double> dist(0,sigma);
				pa = boost::math::cdf(dist,_a) - 0.5;
				if (_b)
					pb = boost::math::cdf(dist,_b) - 0.5;
				else
					pb = 0.5;

				p = pb - pa;

				// And, because it's really a positive-only gaussian dist...
				p *= 2.0;

				// Should work without error because points is a standard set
				_pointWeights[*it] = p;
			}
		}

		double gaussianPosWeights::weight(double point) const
		{
			if (_pointWeights.count(point))
				return _pointWeights.at(point);
			return 0.0;
		}


	}; // end ddscat
}; // end rtmath
