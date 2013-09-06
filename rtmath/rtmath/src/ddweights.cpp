#include "Stdafx-ddscat.h"
#include <cmath>
#include <boost/math/distributions/normal.hpp>
#include <boost/math/special_functions/bessel.hpp>
#include <boost/math/special_functions/erf.hpp>
#include "../rtmath/splitSet.h"
#include "../rtmath/ddscat/ddweights.h"
#include "../rtmath/ddscat/rotations.h"
#include "../rtmath/error/error.h"

namespace rtmath {
	namespace ddscat {
		namespace weights {

			void ddWeights::getWeights(IndependentWeights &weights) const
			{
				weights = weights;
			}

			void ddWeights::getFreqs(DegeneracyTable &freqs) const
			{
				freqs = this->freqs;
			}

			void ddWeights::getIntervals(IntervalTable &intervals) const
			{
				intervals = this->intervals;
			}

			void ddWeights::getIntervals(IntervalTable1d &intervals1d) const
			{
				intervals1d = this->intervals1d;
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

			bool ddWeights::interval(double point, double &start, double &end, double &pivot) const
			{
				start = point;
				end = point;
				auto it = std::find_if(intervals.cbegin(), intervals.cend(), 
					[&](const std::pair<double,std::pair<double, double> > &p)
				{
					/* // Exact match
					if (abs((point - p.first) / p.first) < 0.0001) return true;
					if (abs(point - p.first) < 0.0001) return true; // Near-zero value
					return false;
					*/
					// I instead just want to find the containing interval
					if (p.second.first <= point && p.second.second > point) return true;
					return false;
				});
				if (it != intervals.cend())
				{
					start = it->second.first;
					end = it->second.second;
					pivot = it->first;
					return true;
				}
				return false;
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

			void ddWeights::fillIntervalTable(double start, double end,
				const std::function<double(double,double)> &midpointFunction)
			{
				for (auto it = weights.cbegin(); it != weights.cend(); ++it)
				{
					double min = 0;
					double max = 0;
					if (it == weights.cbegin()) min = start;
					else
					{
						auto ot = it;
						ot--;
						min = midpointFunction(it->first, ot->first);
					}
					auto jt = it;
					jt++;
					if (jt == weights.cend()) max = end;
					else max = midpointFunction(it->first, jt->first);

					intervals.emplace(std::pair<double, std::pair<double, double> >
						(it->first, std::pair<double, double>(min,max) ) );

					IntervalTable1dEntry ii;
					ii[IntervalTable1dDefs::MIN] = min;
					ii[IntervalTable1dDefs::MAX] = max;
					ii[IntervalTable1dDefs::PIVOT] = it->first;
					ii[IntervalTable1dDefs::WEIGHT_RAW] = it->second;
					ii[IntervalTable1dDefs::DEGENERACY] = 1;
					ii[IntervalTable1dDefs::WEIGHT_DEGEN] = it->second;
					intervals1d.push_back(std::move(ii));
				}
			}

			ddWeightsLinInt::ddWeightsLinInt(double start, double end, size_t n)
			{
				double dn = (double) n;
				std::set<double> pts;
				rtmath::config::splitSet(start, end, dn, "lin", pts);
				for (auto &pt : pts)
					weights[pt] = 1. / (double) pts.size();
				
				auto mp = [](double a,double b) -> double
				{
					return ( ( a / 2.) + (b / 2.) );
				};
				fillIntervalTable(start, end, mp);
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

				auto mpc = [](double a,double b) -> double
				{
					// Cosine intervals use degrees
					const double pi = boost::math::constants::pi<double>();
					double ca = cos(pi*a/180.);
					double cb = cos(pi*b/180.);
					double cmp = ( ( ca / 2.) + (cb / 2.) );
					return acos(cmp) * 180. / pi;
				};
				fillIntervalTable(start, end, mpc);
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
			{
				IntervalTable1d IntBeta, IntTheta, IntPhi;
				wBetas.getIntervals(IntBeta);
				wThetas.getIntervals(IntTheta);
				wPhis.getIntervals(IntPhi);

				// Populate the IntervalWeights object
				for (auto &b : IntBeta)
				for (auto &t : IntTheta)
				for (auto &p : IntPhi)
				{
					IntervalTable3dEntry it;
					it[IntervalTable3dDefs::BETA_MIN] = b[IntervalTable1dDefs::MIN];
					it[IntervalTable3dDefs::BETA_MAX] = b[IntervalTable1dDefs::MAX];
					it[IntervalTable3dDefs::BETA_PIVOT] = b[IntervalTable1dDefs::PIVOT];
					it[IntervalTable3dDefs::THETA_MIN] = t[IntervalTable1dDefs::MIN];
					it[IntervalTable3dDefs::THETA_MAX] = t[IntervalTable1dDefs::MAX];
					it[IntervalTable3dDefs::THETA_PIVOT] = t[IntervalTable1dDefs::PIVOT];
					it[IntervalTable3dDefs::PHI_MIN] = p[IntervalTable1dDefs::MIN];
					it[IntervalTable3dDefs::PHI_MAX] = p[IntervalTable1dDefs::MAX];
					it[IntervalTable3dDefs::PHI_PIVOT] = p[IntervalTable1dDefs::PIVOT];
					it[IntervalTable3dDefs::WEIGHT] = b[IntervalTable1dDefs::WEIGHT_DEGEN]
						* t[IntervalTable1dDefs::WEIGHT_DEGEN]
						* p[IntervalTable1dDefs::WEIGHT_DEGEN];
					IntervalWeights.push_back(std::move(it));
				}
			}

			void ddWeightsDDSCAT::getIntervalTable(IntervalTable3d &res) const
			{
				res = IntervalWeights;
			}

			double ddWeightsDDSCAT::getWeight(double beta, double theta, double phi) const
			{
				return wBetas.weightBase(beta)
					* wThetas.weightBase(theta)
					* wPhis.weightBase(phi);
			}

			double ddWeightsDDSCAT::getWeight(double theta, double phi) const
			{
				return wThetas.weightBase(theta)
					* wPhis.weightBase(phi);
			}

			double ddWeightsDDSCAT::getWeight(double beta) const
			{
				return wBetas.weightBase(beta);
			}

			void ddWeightsDDSCAT::getIntervalBounds(
				double beta, double theta, double phi,
				dp &intBeta, dp &intTheta, dp &intPhi) const
			{
				double junk;
				wBetas.interval(beta, intBeta.first, intBeta.second, junk);
				wThetas.interval(theta, intTheta.first, intTheta.second, junk);
				wPhis.interval(phi, intPhi.first, intPhi.second, junk);
			}


			OrientationWeights1d::~OrientationWeights1d()
			{
			}

			OrientationWeights1d::OrientationWeights1d(bool cyclic)
				: cyclic(cyclic), min(0), max(0), span(0)
			{
			}

			void OrientationWeights1d::calcSpan(const ddWeights& w, double &min, double &max, double &span) const
			{
				IntervalTable intervals;
				w.getIntervals(intervals);
				if (intervals.size() == 0) throw debug::xArrayOutOfBounds();
				min = intervals.begin()->second.first;
				max = intervals.rbegin()->second.second;
				span = max - min;
			}
			
			void OrientationWeights1d::getWeights(weightTable &weights) const
			{
				weights = this->weights;
			}

			Uniform1dWeights::Uniform1dWeights(const ddWeights& dw)
				: OrientationWeights1d(true)
			{
				// Calculate the overall span
				calcSpan(dw,min,max,span);

				IntervalTable intervals;
				dw.getIntervals(intervals);

				for (const auto i : intervals)
				{
					// Note: DDSCAT does not present any intervals which go past the zero degree point.
					weights.insert(std::pair<double,double>
						(i.second.first, (i.second.second - i.second.first) / span) );
				}
			}

			double VonMisesWeights::VonMisesPDF(double x, double mu, double kappa)
			{
				//const double span = max - min;
				const double pi = boost::math::constants::pi<double>();
				//const double scale = span / (2. * pi);

				const double mb1 = boost::math::cyl_bessel_i(0, kappa);

				double pdf = exp(kappa * cos(x - mu) ) / (2. * pi * mb1);
				return pdf;
			}

			double VonMisesWeights::VonMisesCDF(double x, double mu, double kappa)
			{
				const double pi = boost::math::constants::pi<double>();

				double res = 0;
				double resprev = 1.0;
				for (size_t i=1; abs((res-resprev)/resprev) > 0.0001 || i > 100; ++i)
				{
					resprev = res;

					double j = (double) i;
					res += boost::math::cyl_bessel_i(j, kappa) * sin(j*(x-mu)) / j;
				}

				res *= 2. / boost::math::cyl_bessel_i(0, kappa);
				res += x;
				res /= 2. * pi;
				return res;
			}

			VonMisesWeights::VonMisesWeights(const ddWeights& dw, double mean, double kappa)
				: mean(mean), kappa(kappa), OrientationWeights1d(true)
			{
				const double pi = boost::math::constants::pi<double>();
				// Calculate the overall span
				calcSpan(dw,min,max,span);

				if ( abs(span - ( 360 ) ) > 0.001)
					throw debug::xArrayOutOfBounds();

				IntervalTable intervals;
				dw.getIntervals(intervals);

				for (const auto i : intervals)
				{
					double weight = VonMisesCDF(i.second.second * pi / 180., mean * pi / 180., kappa * pi / 180.) 
						- VonMisesCDF(i.second.first * pi / 180., mean * pi / 180., kappa * pi / 180.);
					// Note: DDSCAT does not present any intervals which go past the zero degree point.
					weights.insert(std::pair<double,double>(i.second.first, weight ));
				}
			}

			OrientationWeights3d::OrientationWeights3d()
			{
			}

			OrientationWeights3d::~OrientationWeights3d()
			{
			}

			void OrientationWeights3d::getWeights(IntervalTable3d &weights) const
			{
				weights = this->weights;
			}

			VonMisesFisherWeights::VonMisesFisherWeights(const ddWeightsDDSCAT& dw, double muT, double muP, double kappa)
				: meanTheta(muT), meanPhi(muP), kappa(kappa), OrientationWeights3d()
			{
				// For now, assume that the weights will all sum to unity (ddscat has calculated using the usual 
				// rotation bounds).

				// Assumes that beta orientation weighting is uniform, and that the only variation occurs in 
				// theta and phi.

				IntervalTable3d intervals;
				dw.getIntervalTable(intervals);

				auto toRad = [](double val) -> double
				{
					const double pi = boost::math::constants::pi<double>();
					return val * pi / 180.;
				};

				for (const auto i : intervals)
				{
					/// \todo There is a bug here. Fix it!
					// TODO: Fix bug here. CDF function does not take angles directly. It needs 
					// conversion of thetas, phis and mus to a different coordinate system.



					double weight = VonMisesFisherCDF(
						i[IntervalTable3dDefs::THETA_MIN],i[IntervalTable3dDefs::THETA_MAX],
						i[IntervalTable3dDefs::PHI_MIN],i[IntervalTable3dDefs::PHI_MAX],
						muT, muP, kappa);
					weight /= static_cast<double>(dw.numBetas());

					IntervalTable3dEntry ie = i;
					ie[IntervalTable3dDefs::WEIGHT] = weight;

					weights.push_back(std::move(ie));
				}
			}

			double VonMisesFisherWeights::VonMisesFisherPDF(double xT, double xP, double muT, double muP, double kappa)
			{
				const double pi = boost::math::constants::pi<double>();
				const double C3 = kappa / ( 2.*pi * (exp(kappa) - exp(-kappa) ) );
				double vp = (xT * muT) + (xP * muP);
				double f = C3 * exp(kappa * vp);
				return f;
			}

			double VonMisesFisherWeights::VonMisesFisherCDF(double xT1, double xT2, double xP1, double xP2, double muT, double muP, double kappa)
			{
				const double pi = boost::math::constants::pi<double>();
				const double C3 = kappa / ( 2.*pi * (exp(kappa) - exp(-kappa) ) );
				
				double res = C3 / pow(kappa,2.);
				res *= (1./muT) * ( exp(kappa*muT*xT2) - exp(kappa*muT*xT1) );
				res *= (1./muP) * ( exp(kappa*muP*xP2) - exp(kappa*muP*xP1) );
				return res;
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
}
