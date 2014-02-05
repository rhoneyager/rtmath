#include "Stdafx-ddscat.h"
#pragma warning( disable : 4244 ) // annoying double to float issues in boost

#include <functional>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics.hpp>
#include <boost/accumulators/statistics/covariance.hpp>
#include <boost/accumulators/statistics/density.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/kurtosis.hpp>
#include <boost/accumulators/statistics/max.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <boost/accumulators/statistics/min.hpp>
#include <boost/accumulators/statistics/moment.hpp>
#include <boost/accumulators/statistics/skewness.hpp>
#include <boost/accumulators/statistics/variance.hpp>
#include <boost/accumulators/statistics/variates/covariate.hpp>
#include <boost/math/constants/constants.hpp>
#include <Eigen/Dense>

#include "../rtmath/ddscat/shapestatsRotated.h"
#include "../rtmath/ddscat/shapestats.h"
#include "../rtmath/ddscat/rotations.h"

namespace rtmath {
	namespace ddscat {

		namespace stats {

			const unsigned int shapeFileStatsRotated::_maxVersion = SHAPESTATS_ROTATED_VERSION;

			bool shapeFileStatsRotated::needsUpgrade() const
			{
				// Standard case
				if (this->_currVersion >= 0 && this->_currVersion < _maxVersion) return true;
				return false;
			}

			boost::shared_ptr<const shapeFileStatsRotated> shapeFileStatsBase::calcStatsRot(double beta, double theta, double phi)
			{
				boost::shared_ptr<shapeFileStatsRotated> pres(new shapeFileStatsRotated);
				shapeFileStatsRotated &res = *pres;

				res.beta = beta;
				res.theta = theta;
				res.phi = phi;
				// Please note that the set rotations has a special comparator to allow for 
				// rotational ordering intrusive to a shared pointer, which allows for the 
				// avoidance of a map.
				if (rotations.count(pres)) // Already calculated
				{
					auto it = rotations.find(pres);
					return *it;
				}

				Eigen::Matrix3f Roteff;
				rotationMatrix<float>( (float) theta,(float) phi,(float) beta,Roteff);
				// Normally, Reff = RyRxRz. But, the rotation is a1,a2-dependent, 
				// which are specified in the file. Apply effective rotation matrix also.

				using namespace boost::accumulators;

				// Figure out potential energy
				// Using moment<1> to rescale value by dividing by the total number of points.
				// I'm currently ignoring the mass term, so am making the ansatz that the whole flake 
				// has unit mass.
				accumulator_set<float, boost::accumulators::stats<tag::min, tag::max, tag::moment<1> > > abs_x, abs_y, abs_z;
				// Tried http://stackoverflow.com/questions/4316716/is-it-possible-to-use-boost-accumulators-with-vectors?rq=1
				// with accumulator_set<vector<double>, ...), but it does not compile on msvc 2010
				typedef std::vector<accumulator_set<float, boost::accumulators::stats<
					tag::min,
					tag::max, 
					tag::moment<1>,
					tag::moment<2>,
					tag::sum,
					tag::skewness,
					tag::kurtosis,
					// Covariances are special
					tag::variance,
					tag::covariance<float, tag::covariate1>,
					tag::covariance<float, tag::covariate2>
				> > > acc_type;
				acc_type acc_x, acc_y, acc_z, acc_r;
				//acc(std::vector<double>(3)); //acc_x, acc_y, acc_z;

				const size_t nV = _shp->Dielectrics.size()+1;
				acc_x.resize(nV); // Assumes that the dielectrics start at 1.
				acc_y.resize(nV); // Will probably crash if not.
				acc_z.resize(nV);
				acc_r.resize(nV);
				res.PE.resize(nV, 4);
				res.mom1.resize(nV, 4);
				res.mom2.resize(nV, 4);
				res.mominert.resize(nV);
				res.covariance.resize(nV);

				res.min.resize(nV,4);
				res.max.resize(nV,4);
				res.sum.resize(nV,4);
				res.skewness.resize(nV,4);
				res.kurtosis.resize(nV,4);

				//for (auto it = _shp->latticePtsStd.begin(), ot = _shp->latticePtsRi.begin(); 
				//	it != _shp->latticePtsStd.end(); ++it, ++ot)
				for (size_t i = 0; i < _shp->numPoints; i++)
				{
					auto iit = _shp->latticePts.block<1,3>(i,0);
					auto iot = _shp->latticePtsRi.block<1,3>(i,0);
					auto it = &iit; // Laziness when adapting the algorithm
					auto ot = &iot;

					const Eigen::Vector3f s = (*it).transpose() - b_mean;
					const Eigen::Vector3f pt = Roteff * s;
					const double x = pt(0);
					const double y = pt(1);
					const double z = pt(2);
					const size_t diel = (const size_t) (*ot)(0);

					acc_x[0](x, covariate1 = y, covariate2 = z);
					acc_y[0](y, covariate1 = x, covariate2 = z);
					acc_z[0](z, covariate1 = x, covariate2 = y);
					acc_r[0](sqrt((x*x)+(y*y)+(z*z)), covariate1 = x, covariate2 = y);
					acc_x[diel](x, covariate1 = y, covariate2 = z);
					acc_y[diel](y, covariate1 = x, covariate2 = z);
					acc_z[diel](z, covariate1 = x, covariate2 = y);
					acc_r[diel](sqrt((x*x)+(y*y)+(z*z)), covariate1 = x, covariate2 = y);
					abs_x(abs(x));
					abs_y(abs(y));
					abs_z(abs(z));

					// Accumulators are in TF frame? Check against Holly code
				}

				// Are other quantities needed?
				/*
				auto makeMapXYZR = [&](size_t index, Eigen::Matrix<float, Eigen::Dynamic, 4>& m, std::function<float(acc_type::value_type)>& f)
				//decltype(boost::accumulators::m
				//auto makeMapXYZR = [&](size_t index, Eigen::Matrix<float, Eigen::Dynamic, 4>& m, decltype(boost::accumulators::min)& f)
				{
				m(index,0) = f(acc_x[index]);
				m(index,1) = f(acc_y[index]);
				m(index,2) = f(acc_z[index]);
				m(index,3) = f(acc_r[index]);
				};

				makeMapXYZR(0, res.min, boost::accumulators::min);
				makeMapXYZR(0, res.max, boost::accumulators::max);
				*/

				// Absolue value-dependent quantities

				res.abs_min(0) = boost::accumulators::min(abs_x); // abs, not acc here
				res.abs_min(1) = boost::accumulators::min(abs_y);
				res.abs_min(2) = boost::accumulators::min(abs_z);

				res.abs_max(0) = boost::accumulators::max(abs_x); // abs, not acc here
				res.abs_max(1) = boost::accumulators::max(abs_y);
				res.abs_max(2) = boost::accumulators::max(abs_z);

				res.abs_mean(0) = boost::accumulators::moment<1>(abs_x); // abs, not acc here
				res.abs_mean(1) = boost::accumulators::moment<1>(abs_y);
				res.abs_mean(2) = boost::accumulators::moment<1>(abs_z);

				// Aspect ratios
				// Absolute aspect ratio
				res.as_abs(0,0) = 1;
				res.as_abs(0,1) = boost::accumulators::max(abs_x)/boost::accumulators::max(abs_y);
				res.as_abs(0,2) = boost::accumulators::max(abs_x)/boost::accumulators::max(abs_z);
				res.as_abs(1,0) = boost::accumulators::max(abs_y)/boost::accumulators::max(abs_x);
				res.as_abs(1,1) = 1;
				res.as_abs(1,2) = boost::accumulators::max(abs_y)/boost::accumulators::max(abs_z);
				res.as_abs(2,0) = boost::accumulators::max(abs_z)/boost::accumulators::max(abs_x);
				res.as_abs(2,1) = boost::accumulators::max(abs_z)/boost::accumulators::max(abs_y);
				res.as_abs(2,2) = 1;

				// Absolute mean
				res.as_abs_mean(0,0) = 1;
				res.as_abs_mean(0,1) = boost::accumulators::moment<1>(abs_x)/boost::accumulators::moment<1>(abs_y);
				res.as_abs_mean(0,2) = boost::accumulators::moment<1>(abs_x)/boost::accumulators::moment<1>(abs_z);
				res.as_abs_mean(1,0) = boost::accumulators::moment<1>(abs_y)/boost::accumulators::moment<1>(abs_x);
				res.as_abs_mean(1,1) = 1;
				res.as_abs_mean(1,2) = boost::accumulators::moment<1>(abs_y)/boost::accumulators::moment<1>(abs_z);
				res.as_abs_mean(2,0) = boost::accumulators::moment<1>(abs_z)/boost::accumulators::moment<1>(abs_x);
				res.as_abs_mean(2,1) = boost::accumulators::moment<1>(abs_z)/boost::accumulators::moment<1>(abs_y);
				res.as_abs_mean(2,2) = 1;

				// Absolute rms
				res.as_rms(0,0) = 1;
				res.as_rms(0,1) = sqrt(boost::accumulators::moment<2>(acc_x[0])/boost::accumulators::moment<2>(acc_y[0]));
				res.as_rms(0,2) = sqrt(boost::accumulators::moment<2>(acc_x[0])/boost::accumulators::moment<2>(acc_z[0]));
				res.as_rms(1,0) = sqrt(boost::accumulators::moment<2>(acc_y[0])/boost::accumulators::moment<2>(acc_x[0]));
				res.as_rms(1,1) = 1;
				res.as_rms(1,2) = sqrt(boost::accumulators::moment<2>(acc_y[0])/boost::accumulators::moment<2>(acc_z[0]));
				res.as_rms(2,0) = sqrt(boost::accumulators::moment<2>(acc_z[0])/boost::accumulators::moment<2>(acc_x[0]));
				res.as_rms(2,1) = sqrt(boost::accumulators::moment<2>(acc_z[0])/boost::accumulators::moment<2>(acc_y[0]));
				res.as_rms(2,2) = 1;

				// RMS accumulators - rms is the square root of the 2nd moment
				res.rms_mean(0) = sqrt(boost::accumulators::variance(acc_x[0]));
				res.rms_mean(1) = sqrt(boost::accumulators::variance(acc_y[0]));
				res.rms_mean(2) = sqrt(boost::accumulators::variance(acc_z[0]));
				res.rms_mean(3) = sqrt(boost::accumulators::variance(acc_r[0]));

				const size_t _N = _shp->numPoints;
				const Eigen::Array3f &d = _shp->d;
				const float dxdydz = d(0) * d(1) * d(2);

				for (size_t i=0; i<nV; i++)
				{
					res.min(i,0) = boost::accumulators::min(acc_x[i]);
					res.min(i,1) = boost::accumulators::min(acc_y[i]);
					res.min(i,2) = boost::accumulators::min(acc_z[i]);
					res.min(i,3) = boost::accumulators::min(acc_r[i]);

					res.max(i,0) = boost::accumulators::max(acc_x[i]);
					res.max(i,1) = boost::accumulators::max(acc_y[i]);
					res.max(i,2) = boost::accumulators::max(acc_z[i]);
					res.max(i,3) = boost::accumulators::max(acc_r[i]);

					res.sum(i,0) = boost::accumulators::sum(acc_x[i]);
					res.sum(i,1) = boost::accumulators::sum(acc_y[i]);
					res.sum(i,2) = boost::accumulators::sum(acc_z[i]);
					res.sum(i,3) = boost::accumulators::sum(acc_r[i]);

					res.skewness(i,0) = boost::accumulators::skewness(acc_x[i]);
					res.skewness(i,1) = boost::accumulators::skewness(acc_y[i]);
					res.skewness(i,2) = boost::accumulators::skewness(acc_z[i]);
					res.skewness(i,3) = boost::accumulators::skewness(acc_r[i]);

					res.kurtosis(i,0) = boost::accumulators::kurtosis(acc_x[i]);
					res.kurtosis(i,1) = boost::accumulators::kurtosis(acc_y[i]);
					res.kurtosis(i,2) = boost::accumulators::kurtosis(acc_z[i]);
					res.kurtosis(i,3) = boost::accumulators::kurtosis(acc_r[i]);

					res.mom1(i,0) = boost::accumulators::moment<1>(acc_x[i]);
					res.mom1(i,1) = boost::accumulators::moment<1>(acc_y[i]);
					res.mom1(i,2) = boost::accumulators::moment<1>(acc_z[i]);
					res.mom1(i,3) = boost::accumulators::moment<1>(acc_r[i]);

					res.mom2(i,0) = boost::accumulators::moment<2>(acc_x[i]);
					res.mom2(i,1) = boost::accumulators::moment<2>(acc_y[i]);
					res.mom2(i,2) = boost::accumulators::moment<2>(acc_z[i]);
					res.mom2(i,3) = boost::accumulators::moment<2>(acc_r[i]);

					//covariance
					res.covariance[i](0,0) = boost::accumulators::variance(acc_x[i]);
					//boost::accumulators::covariance(acc_x, covariate1);
					//boost::accumulators::covariance(acc_x, covariate2);
					res.covariance[i](0,1) = boost::accumulators::covariance(acc_x[i], covariate1);
					res.covariance[i](0,2) = boost::accumulators::covariance(acc_x[i], covariate2);
					res.covariance[i](1,0) = boost::accumulators::covariance(acc_y[i], covariate1);
					res.covariance[i](1,1) = boost::accumulators::variance(acc_y[i]);
					res.covariance[i](1,2) = boost::accumulators::covariance(acc_y[i], covariate2);
					res.covariance[i](2,0) = boost::accumulators::covariance(acc_z[i], covariate1);
					res.covariance[i](2,1) = boost::accumulators::covariance(acc_z[i], covariate2);
					res.covariance[i](2,2) = boost::accumulators::variance(acc_z[i]);

					// Calculate moments of inertia
					float val = 0;

					// See derivation in Summer 2012 notebook

					// All of these are the partial moments. They need to be scaled by the cell _density_,
					// as multiplying by dxdydz is a volume.
					// I_xx
					val = boost::accumulators::moment<2>(acc_y[i]) + boost::accumulators::moment<2>(acc_z[i]);
					val *= _N * dxdydz;
					res.mominert[i](0,0) = val;

					// I_yy
					val = boost::accumulators::moment<2>(acc_x[i]) + boost::accumulators::moment<2>(acc_z[i]);
					val *= _N * dxdydz;
					res.mominert[i](1,1) = val;

					// I_zz
					val = boost::accumulators::moment<2>(acc_x[i]) + boost::accumulators::moment<2>(acc_y[i]);
					val *= _N * dxdydz;
					res.mominert[i](2,2) = val;

					// I_xy and I_yx
					val = res.covariance[i](1,0) + (boost::accumulators::moment<1>(acc_x[i]) * boost::accumulators::moment<1>(acc_y[i]) );
					val *= -1.0f * _N * dxdydz;
					res.mominert[i](0,1) = val;
					res.mominert[i](1,0) = val;

					// I_xz and I_zx
					val = res.covariance[i](2,0) + (boost::accumulators::moment<1>(acc_x[i]) * boost::accumulators::moment<1>(acc_z[i]) );
					val *= -1.0f * _N * dxdydz;
					res.mominert[i](0,2) = val;
					res.mominert[i](2,0) = val;

					// I_yz and I_zy
					val = res.covariance[i](2,1) + (boost::accumulators::moment<1>(acc_y[i]) * boost::accumulators::moment<1>(acc_z[i]) );
					val *= -1.0f * _N * dxdydz;
					res.mominert[i](2,1) = val;
					res.mominert[i](1,2) = val;

					// Calculate the potential energy
					const float g = 9.80665f; // m/s^2
					// I do not need the partial mass means to be zero here
					res.PE(i,0) = g*boost::accumulators::sum(acc_x[i]);
					res.PE(i,1) = g*boost::accumulators::sum(acc_y[i]);
					res.PE(i,2) = g*boost::accumulators::sum(acc_z[i]);
					res.PE(i,3) = 0; // Writing dir_mat_t, but with three elements instead of four.
				}

				res.areas(0) = 0;
				res.areas(1) = 0;
				res.areas(2) = 0;

				// Use std move to insert into set
				//boost::shared_ptr<const shapeFileStatsRotated> cpres
				//	= boost::const_pointer_cast<
				rotations.insert(pres);
				return pres;
			}


			shapeFileStatsRotated::shapeFileStatsRotated() : 
				beta(0), theta(0), phi(0), _currVersion(-1) { }

			shapeFileStatsRotated::~shapeFileStatsRotated() { }

			bool shapeFileStatsRotated::operator<(const shapeFileStatsRotated &rhs) const
			{
				if (beta!=rhs.beta) return beta<rhs.beta;
				if (theta!=rhs.theta) return theta<rhs.theta;
				if (phi!=rhs.phi) return phi<rhs.phi;
				return false;
			}

		}
	}
}
