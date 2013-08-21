#include "Stdafx-ddscat.h"
#include <iostream>
#include <boost/math/special_functions/erf.hpp>
#include <memory>
#include <cmath>
#include <boost/math/distributions/normal.hpp>
//#include "../rtmath/ddscat/ddscat.h"
#include "../rtmath/splitSet.h"
#include "../rtmath/ddscat/ddweights.h"
#include "../rtmath/ddscat/rotations.h"
#include "../rtmath/error/error.h"

namespace rtmath {
	namespace ddscat {

		void ddWeights::getWeights(std::map<double,double> &weights) const
		{
			weights = weights;
		}

		void ddWeights::getFreqs(std::map<double,size_t> &freqs) const
		{
			freqs = this->freqs;
		}
		
		double ddWeights::weightBase(double point) const
		{
			// Implementing a comparator here because exact double comparisons 
			// cause the usual problems.
			auto it = std::find_if(weights.cbegin(), weights.cend(), [&](const std::pair<double,double> &p)
			{
				if (abs((point - p.first) / p.first) < 0.0001) return true;
				if (abs(point - p.first) < 0.0001) return true; // Near-zero value
				return false;
			});
			if (it != weights.cend())
				return it->second;
			return 0.0;
		}

		double ddWeights::weightDegen(double point) const
		{
			double wt = weightBase(point);
			if (wt)
			{
				double degen = 1.0;
				// Comparator for double precision issues
				auto it = std::find_if(freqs.cbegin(), freqs.cend(), [&](const std::pair<double,size_t> &p)
				{
					if (abs((point - p.first) / p.first) < 0.0001) return true;
					if (abs(point - p.first) < 0.0001) return true; // Near-zero value
					return false;
				});
				if (it != freqs.cend()) degen *= (double) it->second;
				return wt / degen;
			}
			return 0.0;
		}

		ddWeightsLinInt::ddWeightsLinInt(double start, double end, size_t n)
		{
			double dn = (double) n;
			std::set<double> pts;
			rtmath::config::splitSet(start, end, dn, "lin", pts);
			for (auto &pt : pts)
				weights[pt] = 1. / (double) pts.size();
		}

		ddWeightsCosInt::ddWeightsCosInt(double start, double end, size_t n)
		{
			double dn = (double) n;
			std::set<double> pts;
			rtmath::config::splitSet(start, end, dn, "cos", pts);
			if (pts.size() % 2 == 0) // even n
			{
				for (auto &pt : pts)
					weights[pt] = 1. / (double) pts.size();
			} else { // odd n
				// First (0) and last (pts.size()-1) points have weighting of 1 / pts.size()
				// Points 1, 3, 5, ... have weights of 4 / pts.size()
				// Points 2, 4, 6, ... have weights of 2 / pts.size()
				size_t i=0;
				double wtb = 1. / (3. * ((double) n - 1.));
				for (auto it = pts.cbegin(); it != pts.cend(); ++it, ++i)
				{
					weights[*it] = wtb;
					if (i == 0 || i == n - 1)
						continue;
					if (i % 2) // 1, 3, 5, ... Remember, fortran starts at different index.
						weights[*it] = 4. * wtb;
					else if (n >= 5)
						weights[*it] = 2. * wtb;
				}
			}
		}

		ddWeightsDDSCAT::ddWeightsDDSCAT(
			double bMin, double bMax, size_t nB,
			double tMin, double tMax, size_t nT,
			double pMin, double pMax, size_t nP) :
			wThetas(tMin, tMax, nT),
			wBetas(bMin, bMax, nB),
			wPhis(pMin, pMax, nP)
		{ }

		ddWeightsDDSCAT::ddWeightsDDSCAT(const rotations& rots) :
			wThetas(rots.tMin(), rots.tMax(), rots.tN()),
			wBetas(rots.bMin(), rots.bMax(), rots.bN()),
			wPhis(rots.pMin(), rots.pMax(), rots.pN())
		{ }

		double ddWeightsDDSCAT::getWeight(double beta, double theta, double phi) const
		{
			return wBetas.weightBase(beta)
				* wThetas.weightBase(theta)
				* wPhis.weightBase(phi);
		}

		/*
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
		*/

	}
}
