#include "Stdafx-ddscat.h"
#include <iostream>
#include <boost/math/special_functions/erf.hpp>
#include <memory>
#include <cmath>
#include <boost/math/distributions/normal.hpp>
//#include "../rtmath/ddscat/ddscat.h"
#include "../rtmath/ddscat/ddweights.h"
#include "../rtmath/error/error.h"


// The ddscat code was becoming too large, so the weighting functions
// will go in this file.


namespace rtmath {
	namespace ddscat {

		void weights::getWeights(std::map<double,double> &weights) const
		{
			weights = _pointWeights;
		}

		void weights::getFreqs(std::map<double,size_t> &freqs) const
		{
			freqs = _pointFreqs;
		}

		gaussianPosWeights::~gaussianPosWeights()
		{
		}

		gaussianPosWeights::gaussianPosWeights(double sigma,
			const std::multiset<double> &points)
		{
			// TODO: totally need to multiline this error message
			if (points.size() < 3) 
				throw rtmath::debug::xBadInput("Fewer than three points specified when constructing Gaussian weighting function. While possible, poor performance would result.");
			_mu = 0;
			_sigma = sigma;
			double total = 0;
			// Iterate through each point in points
			std::multiset<double>::const_iterator it, ot, pt;
			for (it = points.begin(); it != points.end(); it++)
			{
				// Multiset consideration:
				// If point is a duplicate of one already in _pointWeights, skip it
				if (_pointWeights.count(*it)) continue;

				// Look at the point. If it is the first or the last,
				// it needs special handling because it is at the 
				// end on the interval. Otherwise, find the two middle
				// points between the adjacent points, and this area
				// will be the interval considered for weight calculation
				if (*it < 0) throw rtmath::debug::xAssert("Point < 0");
				double _a = 0, _b = 0;
				ot = it;
				// Advance ot until it no longer matches it (multiset)
				while (*it == *ot && ot != points.end())
				{
					ot++;
					if (ot == points.end())
						break;
				}
				if (it == points.begin())
				{
					_a = 0;
					if (ot != points.end())
						_b = 0.5 * (*it + *ot);
					else
						_b = 0; // Shouldn't reach here, as I have at least 3 points for interpolation
				} else {
					pt = it;
					// No need to do long seek, as duplicate case detection
					// guarantees that (it) is the first point of this value 
					// in this inner loop
					pt--; // Senseless if (it) == begin().
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

				// Scale on account of multiple points being like this
				p /= (double) points.count(*it);

				total += p;
				// Should work without error because points is a standard set
				_pointWeights[*it] = p;
				_pointFreqs[*it] = points.count(*it);
			}

			// Rescale to unity
			// Total is the sum of all the weights. If this is less than one, then increase 
			// the weighting by dividing by total
			if (total == 0) return; // Avoid NaN!
			std::map<double,double> _newpointWeights;
			for (auto it = _pointWeights.begin(); it != _pointWeights.end(); it++)
			{
				double f = it->first;
				double w = it->second / total;
				_newpointWeights[f] = w;
			}
			_pointWeights = _newpointWeights;
		}

		double weights::weight(double point) const
		{
			if (_pointWeights.count(point))
				return _pointWeights.at(point);
			return 0.0;
		}

		isoPosWeights::~isoPosWeights()
		{
		}

		isoPosWeights::isoPosWeights(const std::multiset<double> &points)
		{
			// TODO: totally need to multiline this error message
			if (points.size() < 3) 
				throw rtmath::debug::xBadInput("Fewer than three points specified when constructing isotropic weighting function. While possible, poor performance would result.");

			// Iterate through each point in points
			std::multiset<double>::const_iterator it, ot, pt;
			for (it = points.begin(); it != points.end(); it++)
			{
				// Multiset consideration:
				// If point is a duplicate of one already in _pointWeights, skip it
				if (_pointWeights.count(*it)) continue;

				// Uniformity is easy!
				_pointWeights[*it] = 1.0 / ((double) points.size() );
				_pointFreqs[*it] = points.count(*it);
			}
		}

	}; // end ddscat
}; // end rtmath
