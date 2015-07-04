#include "Stdafx-ddscat.h"
#include <cmath>
#include <ctime>
#include <mutex>
#include <atomic>
#include <boost/math/distributions/normal.hpp>
#include <boost/math/special_functions/bessel.hpp>
#include <boost/math/special_functions/erf.hpp>
#include <boost/random/random_device.hpp>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_on_sphere.hpp>
#include <boost/random/uniform_real_distribution.hpp>
#include <boost/random/variate_generator.hpp>
#include <Ryan_Debug/splitSet.h>
#include <Ryan_Debug/error.h>
#include "../rtmath/ddscat/ddweights.h"
#include "../rtmath/ddscat/rotations.h"


/// Namespace contains random number generator information
namespace
{
	typedef boost::random::mt19937 gen_type;
	gen_type rand_gen;
	std::mutex rand_mutex;
	std::atomic<bool> inited(false);
	void init_rand()
	{
		if (inited) return;
		std::lock_guard<std::mutex> lock(rand_mutex);

		rand_gen.seed(static_cast<unsigned int>(std::time(0)));
		inited = true;
	}
}

namespace rtmath {
	namespace ddscat {
		namespace weights {
			ddWeights::ddWeights(double start, double end, size_t n)
				: start(start), end(end), n(n) {}

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

			size_t ddWeights::getIndex(double point) const
			{
				size_t index = 0;
				double start = 0, end = 0, pivot = 0;
				interval(point, start, end, pivot);
				auto it = std::find_if(weights.cbegin(), weights.cend(), [&](const std::pair<double,double> &p)
				{
					index++;
					if (abs((pivot - p.first) / p.first) < 0.0001) return true;
					if (abs(pivot - p.first) < 0.0001) return true; // Near-zero value
					return false;
				});
				index--;
				if (it != weights.cend())
					return index;
				return 0;
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
				: ddWeights(start, end, n)
			{
				double dn = (double) n;
				std::set<double> pts;
				Ryan_Debug::splitSet::splitSet(start, end, dn, "lin", pts);
				for (auto &pt : pts)
					weights[pt] = 1. / (double) pts.size();

				auto mp = [](double a,double b) -> double
				{
					return ( ( a / 2.) + (b / 2.) );
				};
				fillIntervalTable(start, end, mp);
			}

			ddWeightsCosInt::ddWeightsCosInt(double start, double end, size_t n)
				: ddWeights(start, end, n)
			{
				double dn = (double) n;
				std::set<double> pts;
				Ryan_Debug::splitSet::splitSet(start, end, dn, "cos", pts);
				if (pts.size() % 2 == 0) // even n
				{
					for (auto &pt : pts)
						weights[pt] = 1. / dn;
				} else { // odd n
					// First (0) and last (pts.size()-1) points have weighting of 1 / pts.size()
					// Points 1, 3, 5, ... have weights of 4 / pts.size()
					// Points 2, 4, 6, ... have weights of 2 / pts.size()
					size_t i=0;
					double wtb = 1. / (3. * (dn - 1.));
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

			/*
			double ddWeightsLinInt::getRandomPoint() const
			{
				init_rand();
				std::lock_guard<std::mutex> lock(rand_mutex);
				boost::random::uniform_real_distribution<double> dist(start,end);
				double val = dist(rand_gen);
				return val;
			}

			void ddWeightsLinInt::getRandomPoints(size_t n, std::vector<double> &out) const
			{
				init_rand();
				out.clear();
				out.reserve(n);
				
				std::lock_guard<std::mutex> lock(rand_mutex);
				boost::random::uniform_real_distribution<double> dist(start,end);
				for (size_t i=0; i<n; i++)
				{
					double val = dist(rand_gen);
					out.push_back(val);
				}
			}

			double ddWeightsCosInt::getRandomPoint() const
			{
				init_rand();
				std::lock_guard<std::mutex> lock(rand_mutex);
				boost::random::uniform_on_sphere<double> dist(1);
				boost::variate_generator<gen_type&, boost::uniform_on_sphere<double> >
					random_on_sphere(rand_gen, dist);
				std::vector<double> random_sphere_point = random_on_sphere();
				return random_sphere_point.at(0);
			}

			void ddWeightsCosInt::getRandomPoints(size_t n, std::vector<double> &out) const
			{
				out.clear();
				out.reserve(n);

				init_rand();
				std::lock_guard<std::mutex> lock(rand_mutex);
				boost::random::uniform_on_sphere<double> dist(1);
				boost::variate_generator<gen_type&, boost::uniform_on_sphere<double> >
					random_on_sphere(rand_gen, dist);
				for (size_t i=0; i<n; i++)
				{
					std::vector<double> random_sphere_point = random_on_sphere();
					out.push_back(random_sphere_point.at(0));
				}
			}
			*/

			ddWeightsDDSCAT::ddWeightsDDSCAT(
				double bMin, double bMax, size_t nB,
				double tMin, double tMax, size_t nT,
				double pMin, double pMax, size_t nP) :
			wThetas(tMin, tMax, nT),
				wBetas(bMin, bMax, nB),
				wPhis(pMin, pMax, nP)
			{ _init(); }

			ddWeightsDDSCAT::ddWeightsDDSCAT(const rotations& rots) :
				wThetas(rots.tMin(), rots.tMax(), rots.tN()),
				wBetas(rots.bMin(), rots.bMax(), rots.bN()),
				wPhis(rots.pMin(), rots.pMax(), rots.pN())
			{ _init(); }

			void ddWeightsDDSCAT::_init()
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
				if (intervals.size() == 0) RDthrow(Ryan_Debug::error::xArrayOutOfBounds())
					<< Ryan_Debug::error::otherErrorText("No intervals can be "
					"constructed from the parameters passed to "
					"this function.");
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
					RDthrow(Ryan_Debug::error::xArrayOutOfBounds());

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

			double OrientationWeights3d::getWeight(double beta, double theta, double phi) const
			{
				for (auto it = this->weights.cbegin(); it != this->weights.cend(); ++it)
				{
					double bmin = it->at(IntervalTable3dDefs::BETA_MIN),
						bmax = it->at(IntervalTable3dDefs::BETA_MAX),
						tmin = it->at(IntervalTable3dDefs::THETA_MIN),
						tmax = it->at(IntervalTable3dDefs::THETA_MAX),
						pmin = it->at(IntervalTable3dDefs::PHI_MIN),
						pmax = it->at(IntervalTable3dDefs::PHI_MAX);
					if (beta < bmin || beta > bmax) continue;
					if (theta < tmin || theta > tmax) continue;
					if (phi < pmin || phi > pmax) continue;

					/*
					std::cerr << it->at(IntervalTable3dDefs::BETA_MIN) <<
						"\t" << it->at(IntervalTable3dDefs::BETA_MAX) <<
						"\t" << it->at(IntervalTable3dDefs::THETA_MIN) <<
						"\t" << it->at(IntervalTable3dDefs::THETA_MAX) <<
						"\t" << it->at(IntervalTable3dDefs::PHI_MIN) <<
						"\t" << it->at(IntervalTable3dDefs::PHI_MAX) << 
						"\t" << it->at(IntervalTable3dDefs::WEIGHT) <<
						std::endl;
					*/

					double weight = it->at(IntervalTable3dDefs::WEIGHT);
					return weight;
				}
				return 0;
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
				auto aToRad = [](size_t n, double *vals, double *res)
				{
					const double pi = boost::math::constants::pi<double>();
					for (size_t i=0;i<n;++i)
						res[i] = vals[i] * pi / 180.;
				};

				const double pi = boost::math::constants::pi<double>();
				const size_t degree = 3;

				for (const auto i : intervals)
				{
					/// \todo There is a bug here. Fix it!
					// TODO: Fix bug here. CDF function does not take angles directly. It needs 
					// conversion of thetas, phis and mus to a different coordinate system.

					double start_deg[degree-1] = {
						i[IntervalTable3dDefs::THETA_MIN],
						i[IntervalTable3dDefs::PHI_MIN] };

					double end_deg[degree-1] = {
						i[IntervalTable3dDefs::THETA_MAX],
						i[IntervalTable3dDefs::PHI_MAX] };

					double mid_deg[degree-1] = {
						i[IntervalTable3dDefs::THETA_PIVOT],
						i[IntervalTable3dDefs::PHI_PIVOT] };

					double mus_deg[degree-1] = { muT, muP };

					double start_rad[degree], end_rad[degree], mid_rad[degree], mus_rad[degree];
					start_rad[0] = 1; end_rad[0] = 1; mid_rad[0] = 1; mus_rad[0] = 1;
					aToRad(2, start_deg, start_rad+1);
					aToRad(2, end_deg, end_rad+1);
					aToRad(2, mid_deg, mid_rad+1);
					aToRad(2, mus_deg, mus_rad+1);

					double start_pol[degree], end_pol[degree], mid_pol[degree], mus_pol[degree];
					radSphToCrt(degree, start_rad, start_pol);
					radSphToCrt(degree, end_rad, end_pol);
					radSphToCrt(degree, mid_rad, mid_pol);
					radSphToCrt(degree, mus_rad, mus_pol);

					double kappa_rad = toRad(kappa);

					auto SA2S = [](const double *start_rad, const double *end_rad) -> double
					{
						// Omega = int int sin(theta) dtheta dphi
						// First is theta, then phi
						//double res = end_rad[1] - start_rad[1];
						//res *= (cos(start_rad[0])) - (cos(end_rad[0]));

						double sa = sin((end_rad[0] - start_rad[0]) / 2.);
						double sb = sin((end_rad[1] - start_rad[1]) / 2.);
						double res = 4. * asin( sa * sb );
						return abs(res);
					};

					double weight = VonMisesFisherPDF(degree, mid_pol, mus_pol, kappa_rad);
					//weight *= SA2S(start_rad, end_rad); // Scale based on the sphere solid angle
					//weight *= 4. * pi; // Scale based on sphere surface area
					weight /= static_cast<double>(dw.numBetas() * dw.numThetas() * dw.numPhis()); // Account for multiple betas here.
					weight *= 4. * pi;

					IntervalTable3dEntry ie = i;
					ie[IntervalTable3dDefs::WEIGHT] = abs(weight);

					weights.push_back(std::move(ie));
				}
			}

			double VonMisesFisherWeights::VonMisesFisherPDF(size_t degree, const double *x, const double *mu, double kappa)
			{
				const double pi = boost::math::constants::pi<double>();
				const double C = pow(kappa,( static_cast<double>(degree)/2.)-1.)
					/ (pow(2.*pi,static_cast<double>(degree)/2.) * boost::math::cyl_bessel_i(( static_cast<double>(degree)/2.)-1, kappa));
				//const double C = kappa / ( 2.*pi * (exp(kappa) - exp(-kappa) ) );
				double vp = 0;
				for (size_t i=0; i<degree; ++i)
					vp += x[i] * mu[i];
				double f = C * exp(kappa * vp);

				//std::cerr << "PDF: deg:" << degree << " f:" << f << " C:" << C << " k:" << kappa << " vp:" << vp << std::endl;
				return f;
			}

			double VonMisesFisherWeights::VonMisesFisherCDF(size_t degree, const double *x1, const double *x2, const double *mu, double kappa)
			{
				// Note: the Jacobian is r^2 * sin(theta). r is 1 here.
				// Note: surface area of a sphere is 4pi r^2. Will divide to yield the proper cdf.
				const double pi = boost::math::constants::pi<double>();
				double C = kappa / ( 2.*pi * (exp(kappa) - exp(-kappa) ) ); // degree = 3 special case
				if (degree != 3)
					C = pow(kappa,( static_cast<double>(degree)/2.)-1.)
					/ (pow(2.*pi,static_cast<double>(degree)/2.) * boost::math::cyl_bessel_i(( static_cast<double>(degree)/2.)-1, kappa));

				double res = C;
				double denom = 0;
				for (size_t i=0; i<degree; ++i)
				{
					res *= ( exp(kappa*mu[i]*x2[i]) - exp(kappa*mu[i]*x1[i]) ) / (mu[i] * kappa);
					denom += mu[i] * kappa;
				}
				res /= denom;
				return res;
			}

			BimodalVonMisesFisherWeights::BimodalVonMisesFisherWeights(const ddWeightsDDSCAT& dw, double muT, double muP, double kappa)
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
				auto aToRad = [](size_t n, double *vals, double *res)
				{
					const double pi = boost::math::constants::pi<double>();
					for (size_t i=0;i<n;++i)
						res[i] = vals[i] * pi / 180.;
				};

				const double pi = boost::math::constants::pi<double>();
				const size_t degree = 3;

				for (const auto i : intervals)
				{
					/// \todo There is a bug here. Fix it!
					// TODO: Fix bug here. CDF function does not take angles directly. It needs 
					// conversion of thetas, phis and mus to a different coordinate system.

					double start_deg[degree-1] = {
						i[IntervalTable3dDefs::THETA_MIN],
						i[IntervalTable3dDefs::PHI_MIN] };

					double end_deg[degree-1] = {
						i[IntervalTable3dDefs::THETA_MAX],
						i[IntervalTable3dDefs::PHI_MAX] };

					double mid_deg[degree-1] = {
						i[IntervalTable3dDefs::THETA_PIVOT],
						i[IntervalTable3dDefs::PHI_PIVOT] };

					double mus_deg[degree-1] = { muT, muP };

					double mus_deg_b[degree-1] = { 180. - muT, muP + 180. };

					double start_rad[degree], end_rad[degree], mid_rad[degree], mus_rad[degree], mus_b_rad[degree];
					start_rad[0] = 1; end_rad[0] = 1; mid_rad[0] = 1; mus_rad[0] = 1; mus_b_rad[0] = 1;
					aToRad(2, start_deg, start_rad+1);
					aToRad(2, end_deg, end_rad+1);
					aToRad(2, mid_deg, mid_rad+1);
					aToRad(2, mus_deg, mus_rad+1);
					aToRad(2, mus_deg_b, mus_b_rad+1);

					double start_pol[degree], end_pol[degree], mid_pol[degree], mus_pol[degree], mus_b_pol[degree];
					radSphToCrt(degree, start_rad, start_pol);
					radSphToCrt(degree, end_rad, end_pol);
					radSphToCrt(degree, mid_rad, mid_pol);
					radSphToCrt(degree, mus_rad, mus_pol);

					radSphToCrt(degree, mus_b_rad, mus_b_pol);

					double kappa_rad = toRad(kappa);

					auto SA2S = [](const double *start_rad, const double *end_rad) -> double
					{
						// Omega = int int sin(theta) dtheta dphi
						// First is theta, then phi
						//double res = end_rad[1] - start_rad[1];
						//res *= (cos(start_rad[0])) - (cos(end_rad[0]));

						double sa = sin((end_rad[0] - start_rad[0]) / 2.);
						double sb = sin((end_rad[1] - start_rad[1]) / 2.);
						double res = 4. * asin( sa * sb );
						return abs(res);
					};

					double weight_a = VonMisesFisherWeights::VonMisesFisherPDF(degree, mid_pol, mus_pol, kappa_rad);
					double weight_b = VonMisesFisherWeights::VonMisesFisherPDF(degree, mid_pol, mus_b_pol, kappa_rad);
					//weight_a /= static_cast<double>(dw.numBetas() * dw.numThetas() * dw.numPhis()); // Account for multiple betas here.
					//weight_a *= 4. * pi;
					//weight_b /= static_cast<double>(dw.numBetas() * dw.numThetas() * dw.numPhis()); // Account for multiple betas here.
					//weight_b *= 4. * pi;
					double weight = sqrt((weight_a * weight_a) + (weight_b * weight_b));
					weight /= static_cast<double>(dw.numBetas() * dw.numThetas() * dw.numPhis()); // Account for multiple betas here.
					weight *= 4. * pi / sqrt(2);
					//std::cerr << "w: " << weight << " wa:" << weight_a << " wb:" << weight_b << std::endl;
					IntervalTable3dEntry ie = i;
					if (weight <0) weight *= -1.0; // oddly, abs(weight) gives zeros on linux gcc 4.8.1...
					ie[IntervalTable3dDefs::WEIGHT] = weight;

					//for (auto a = ie.begin(); a != ie.end(); ++a)
					//	std::cerr << " " << *a;
					//std::cerr << " " << weight << std::endl;

					weights.push_back(std::move(ie));
				}
			}

			DDSCAT3dWeights::DDSCAT3dWeights(const ddWeightsDDSCAT& dw)
			{
				IntervalTable3d intervals;
				dw.getIntervalTable(intervals);

				for (const auto i : intervals)
				{
					IntervalTable3dEntry ie = i;
					//ie[IntervalTable3dDefs::WEIGHT] = abs(weight);
					weights.push_back(std::move(ie));
				}
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
			if (*it < 0) RDthrow rtmath::debug::xAssert("Point < 0");
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
