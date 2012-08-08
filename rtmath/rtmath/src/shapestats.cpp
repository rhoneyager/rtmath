#include "../rtmath/Stdafx.h"
/*
#include <vector>
#include <map>
#include <set>
#include <complex>

//#include <boost/accumulators/accumulators.hpp>
//#include <boost/accumulators/statistics.hpp>

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
#include <boost/make_shared.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <boost/shared_ptr.hpp>

#include <cmath>
*/
#include "../rtmath/matrixop.h"
#include "../rtmath/error/error.h"
#include "../rtmath/ddscat/shapefile.h"
#include "../rtmath/ddscat/shapestats.h"
#include "../rtmath/ddscat/hulls.h"

namespace rtmath {
	namespace ddscat {

		const unsigned int shapeFileStatsBase::_maxVersion = 1;

		shapeFileStats::shapeFileStats(const shapefile &shp)
		{
			_shp = boost::shared_ptr<shapefile>(new shapefile(shp));
		}

		shapeFileStats::shapeFileStats(const boost::shared_ptr<const shapefile> &shp)
		{
			_shp = boost::shared_ptr<shapefile>(new shapefile(*shp));
		}

		shapeFileStats::shapeFileStats()
		{
		}

		shapeFileStatsBase::shapeFileStatsBase()
			: b_min(2,3,1), b_max(2,3,1), b_mean(2,3,1), rot(2,3,3), invrot(2,3,3)
		{
			beta = 0;
			theta = 0;
			phi = 0;
			V_cell_const = 0;
			V_dipoles_const = 0;
			aeff_dipoles_const = 0;
			max_distance = 0;

			a_circum_sphere = 0;
			V_circum_sphere = 0;
			SA_circum_sphere = 0;
			V_convex_hull = 0;
			aeff_V_convex_hull = 0;
			SA_convex_hull = 0;
			aeff_SA_convex_hull = 0;

			V_ellipsoid_max = 0;
			aeff_ellipsoid_max = 0;
			V_ellipsoid_rms = 0;
			aeff_ellipsoid_rms = 0;

			f_circum_sphere = 0;
			f_convex_hull = 0;
			f_ellipsoid_max = 0;
			f_ellipsoid_rms = 0;

			rho_basic = 1;
			rho_circum_sphere = 0;
			rho_convex = 0;
			rho_ellipsoid_max = 0;
			rho_ellipsoid_rms = 0;


			_currVersion = _maxVersion;
			_valid = false;

			// Need to fill with something for serialization to work with
			//_shp = boost::shared_ptr<shapefile>(new shapefile);
		}

		shapeFileStatsBase::~shapeFileStatsBase()
		{
		}

		bool shapeFileStatsBase::load()
		{
			// Return true if shape is loaded or can be loaded (and load it)
			// Return false if shape CANNOT be loaded
			if (_shp->_latticePts.size() ) return true;

			std::string fname = _shp->_filename;
			if (boost::filesystem::exists(fname))
			{
				boost::shared_ptr<shapefile> nshp(new shapefile(fname));
				_shp = nshp;
				return true;
			} else {
				return false;
			}
		}

		void shapeFileStatsBase::calcStatsRot(double beta, double theta, double phi)
		{
			shapeFileStatsRotated res;
			res.beta = beta;
			res.theta = theta;
			res.phi = phi;
			if (rotations.count(res)) return; // Already calculated

			const double drconv = 2.0*boost::math::constants::pi<double>()/180.0;
			double cb = cos(beta*drconv);
			double ct = cos(theta*drconv);
			double cp = cos(phi*drconv);
			double sb = sin(beta*drconv);
			double st = sin(theta*drconv);
			double sp = sin(phi*drconv);
			// Do right-handed rotation
			// It's just easier to express the overall rotation as the multiplication of
			// the component Gimbal matrices.
			matrixop Rx(2,3,3), Ry(2,3,3), Rz(2,3,3);

			Rx.set(1,2,0,0);
			Rx.set(cp,2,1,1);
			Rx.set(cp,2,2,2);
			Rx.set(sp,2,2,1);
			Rx.set(-sp,2,1,2);

			Ry.set(cb,2,0,0);
			Ry.set(1 ,2,1,1);
			Ry.set(cb,2,2,2);
			Ry.set(sb,2,0,2);
			Ry.set(-sb,2,2,0);

			Rz.set(ct,2,0,0);
			Rz.set(ct,2,1,1);
			Rz.set(1,2,2,2);
			Rz.set(st,2,1,0);
			Rz.set(-st,2,0,1);

			// Normally, Reff = RyRxRz. But, the rotation is a1,a2-dependent, 
			// which are specified in the file. Apply effective rotation matrix also.
			matrixop Roteff = Ry*Rx*Rz*rot;
			
			using namespace boost::accumulators;

			// Figure out potential energy
			// Using moment<1> to rescale value by dividing by the total number of points.
			// I'm currently ignoring the mass term, so am making the ansatz that the whole flake 
			// has unit mass.
			accumulator_set<double, stats<tag::min, tag::max, tag::moment<1> > > abs_x, abs_y, abs_z;
			// Tried http://stackoverflow.com/questions/4316716/is-it-possible-to-use-boost-accumulators-with-vectors?rq=1
			// with accumulator_set<vector<double>, ...), but it does not compile on msvc 2010
			accumulator_set<double, stats<
				tag::min,
				tag::max, 
				tag::moment<1>,
				tag::moment<2>,
				tag::sum,
				tag::skewness,
				tag::kurtosis,
				// Covariances are special
				tag::covariance<double, tag::covariate1>,
				tag::covariance<double, tag::covariate2>
				> > acc_x, acc_y, acc_z; //acc(std::vector<double>(3)); //acc_x, acc_y, acc_z;
				
			for (auto it = _shp->_latticePtsStd.begin(); it != _shp->_latticePtsStd.end(); it++)
			{
				// it->first is the points id. it->second is its matrixop coords (1x3 matrix)
				// Mult by rotaion matrix to get 3x1 rotated matrix
				
				matrixop pt = Roteff * (it->transpose() - b_mean);
				double x = pt.get(2,0,0);
				double y = pt.get(2,1,0);
				double z = pt.get(2,2,0);
				//vector<double> vpt(3);
				//pt.to<std::vector<double> >(vpt);
				acc_x(x, covariate1 = y, covariate2 = z);
				acc_y(y, covariate1 = x, covariate2 = z);
				acc_z(z, covariate1 = x, covariate2 = y);
				abs_x(abs(x));
				abs_y(abs(y));
				abs_z(abs(z));

				// Accumulators are in TF frame? Check against Holly code
			}

			// Are other quantities needed?

			// Export to class matrixops
			res.min.set(boost::accumulators::min(acc_x),2,0,0);
			res.min.set(boost::accumulators::min(acc_y),2,1,0);
			res.min.set(boost::accumulators::min(acc_z),2,2,0);

			res.max.set(boost::accumulators::max(acc_x),2,0,0);
			res.max.set(boost::accumulators::max(acc_y),2,1,0);
			res.max.set(boost::accumulators::max(acc_z),2,2,0);

			res.sum.set(boost::accumulators::sum(acc_x),2,0,0);
			res.sum.set(boost::accumulators::sum(acc_y),2,1,0);
			res.sum.set(boost::accumulators::sum(acc_z),2,2,0);

			res.skewness.set(boost::accumulators::skewness(acc_x),2,0,0);
			res.skewness.set(boost::accumulators::skewness(acc_y),2,1,0);
			res.skewness.set(boost::accumulators::skewness(acc_z),2,2,0);

			res.kurtosis.set(boost::accumulators::kurtosis(acc_x),2,0,0);
			res.kurtosis.set(boost::accumulators::kurtosis(acc_y),2,1,0);
			res.kurtosis.set(boost::accumulators::kurtosis(acc_z),2,2,0);

			res.mom1.set(boost::accumulators::moment<1>(acc_x),2,0,0);
			res.mom1.set(boost::accumulators::moment<1>(acc_y),2,1,0);
			res.mom1.set(boost::accumulators::moment<1>(acc_z),2,2,0);

			res.mom2.set(boost::accumulators::moment<2>(acc_x),2,0,0);
			res.mom2.set(boost::accumulators::moment<2>(acc_y),2,1,0);
			res.mom2.set(boost::accumulators::moment<2>(acc_z),2,2,0);

			// Absolue value-dependent quantities

			res.abs_min.set(boost::accumulators::min(abs_x),2,0,0); // abs, not acc here
			res.abs_min.set(boost::accumulators::min(abs_y),2,1,0);
			res.abs_min.set(boost::accumulators::min(abs_z),2,2,0);

			res.abs_max.set(boost::accumulators::max(abs_x),2,0,0); // abs, not acc here
			res.abs_max.set(boost::accumulators::max(abs_y),2,1,0);
			res.abs_max.set(boost::accumulators::max(abs_z),2,2,0);

			res.abs_mean.set(boost::accumulators::moment<1>(abs_x),2,0,0); // abs, not acc here
			res.abs_mean.set(boost::accumulators::moment<1>(abs_y),2,1,0);
			res.abs_mean.set(boost::accumulators::moment<1>(abs_z),2,2,0);
			res.PE = res.abs_mean;


			// Aspect ratios
			// Absolute aspect ratio
			res.as_abs.set(1,2,0,0);
			res.as_abs.set(boost::accumulators::max(abs_x)/boost::accumulators::max(abs_y),2,0,1);
			res.as_abs.set(boost::accumulators::max(abs_x)/boost::accumulators::max(abs_z),2,0,2);
			res.as_abs.set(boost::accumulators::max(abs_y)/boost::accumulators::max(abs_x),2,1,0);
			res.as_abs.set(1,2,1,1);
			res.as_abs.set(boost::accumulators::max(abs_y)/boost::accumulators::max(abs_z),2,1,2);
			res.as_abs.set(boost::accumulators::max(abs_z)/boost::accumulators::max(abs_x),2,2,0);
			res.as_abs.set(boost::accumulators::max(abs_z)/boost::accumulators::max(abs_y),2,2,1);
			res.as_abs.set(1,2,2,2);

			// Absolute mean
			res.as_abs_mean.set(1,2,0,0);
			res.as_abs_mean.set(boost::accumulators::moment<1>(abs_x)/boost::accumulators::moment<1>(abs_y),2,0,1);
			res.as_abs_mean.set(boost::accumulators::moment<1>(abs_x)/boost::accumulators::moment<1>(abs_z),2,0,2);
			res.as_abs_mean.set(boost::accumulators::moment<1>(abs_y)/boost::accumulators::moment<1>(abs_x),2,1,0);
			res.as_abs_mean.set(1,2,1,1);
			res.as_abs_mean.set(boost::accumulators::moment<1>(abs_y)/boost::accumulators::moment<1>(abs_z),2,1,2);
			res.as_abs_mean.set(boost::accumulators::moment<1>(abs_z)/boost::accumulators::moment<1>(abs_x),2,2,0);
			res.as_abs_mean.set(boost::accumulators::moment<1>(abs_z)/boost::accumulators::moment<1>(abs_y),2,2,1);
			res.as_abs_mean.set(1,2,2,2);

			// Absolute rms
			res.as_rms.set(1,2,0,0);
			res.as_rms.set(boost::accumulators::moment<2>(acc_x)/boost::accumulators::moment<2>(acc_y),2,0,1);
			res.as_rms.set(boost::accumulators::moment<2>(acc_x)/boost::accumulators::moment<2>(acc_z),2,0,2);
			res.as_rms.set(boost::accumulators::moment<2>(acc_y)/boost::accumulators::moment<2>(acc_x),2,1,0);
			res.as_rms.set(1,2,1,1);
			res.as_rms.set(boost::accumulators::moment<2>(acc_y)/boost::accumulators::moment<2>(acc_z),2,1,2);
			res.as_rms.set(boost::accumulators::moment<2>(acc_z)/boost::accumulators::moment<2>(acc_x),2,2,0);
			res.as_rms.set(boost::accumulators::moment<2>(acc_z)/boost::accumulators::moment<2>(acc_y),2,2,1);
			res.as_rms.set(1,2,2,2);

			// RMS accumulators - rms is the square root of the 2nd moment
			res.rms_mean.set(sqrt(boost::accumulators::moment<2>(acc_x)),2,0,0);
			res.rms_mean.set(sqrt(boost::accumulators::moment<2>(acc_y)),2,1,0);
			res.rms_mean.set(sqrt(boost::accumulators::moment<2>(acc_z)),2,2,0);

			const size_t _N = _shp->_numPoints;

			//covariance
			res.covariance.set(_N*boost::accumulators::moment<2>(acc_x),2,0,0);
			//boost::accumulators::covariance(acc_x, covariate1);
			//boost::accumulators::covariance(acc_x, covariate2);
			res.covariance.set(boost::accumulators::covariance(acc_x, covariate1),2,0,1);
			res.covariance.set(boost::accumulators::covariance(acc_x, covariate2),2,0,2);
			res.covariance.set(boost::accumulators::covariance(acc_y, covariate1),2,1,0);
			res.covariance.set(_N*boost::accumulators::moment<2>(acc_y),2,1,1);
			res.covariance.set(boost::accumulators::covariance(acc_y, covariate2),2,1,2);
			res.covariance.set(boost::accumulators::covariance(acc_z, covariate1),2,2,0);
			res.covariance.set(boost::accumulators::covariance(acc_z, covariate2),2,2,1);
			res.covariance.set(_N*boost::accumulators::moment<2>(acc_z),2,2,2);

			// Calculate moments of inertia
			{
				double val = 0;
				// All wrong. Need to redo.
				// I_xx
				val = boost::accumulators::moment<2>(acc_y) + boost::accumulators::moment<2>(acc_z);
				res.mominert.set(val,2,0,0);

				// I_yy
				val = boost::accumulators::moment<2>(acc_x) + boost::accumulators::moment<2>(acc_z);
				res.mominert.set(val,2,1,1);

				// I_zz
				val = boost::accumulators::moment<2>(acc_x) + boost::accumulators::moment<2>(acc_y);
				res.mominert.set(val,2,2,2);

				// I_xy and I_yx
				val = -1.0 * res.covariance.get(2,1,0);
				val /= _N;
				res.mominert.set(val,2,0,1);
				res.mominert.set(val,2,1,0);

				// I_xz and I_zx
				val = -1.0 * res.covariance.get(2,2,0);
				val /= _N;
				res.mominert.set(val,2,0,2);
				res.mominert.set(val,2,2,0);

				// I_yz and I_zy
				val = -1.0 * res.covariance.get(2,2,1);
				val /= _N;
				res.mominert.set(val,2,2,1);
				res.mominert.set(val,2,1,2);
			}

			// Are other quantities needed?


			// Use std move to insert into set
			rotations.insert(std::move(res));
		}

		void shapeFileStatsBase::calcStatsBase()
		{
			using namespace std;
			// Do calculations of the center of mass, the tensor quantities, and other stuff
			// The functions called here are all indep. of the initial state, as mass, density,
			// volume and everything else have been calculated already.

			// Define the accumulators that we want
			// For each axis, get min, max and the other statistics about the distribution

			// Iterate accumulator as function of radial distance from center of mass

			// Pull in some vars from the shapefile
			const size_t _N = _shp->_numPoints;
			
			if (!_N)
			{
				GETOBJKEY();
				throw rtmath::debug::xBadInput("Stats cannot be calculated because the shapefile is not loaded.");
			}

			// Calculate volume elements
			const matrixop &d = _shp->_d;
			double dxdydz = d.get(2,0,0) * d.get(2,0,1) * d.get(2,0,2);
			V_cell_const = dxdydz;
			V_dipoles_const = dxdydz * _N;
			aeff_dipoles_const = (3./(4.*boost::math::constants::pi<double>()))*pow(dxdydz,1./3.);

			const matrixop &a1 = _shp->_a1;
			const matrixop &a2 = _shp->_a2;
			const matrixop &a3 = _shp->_a3;
			//_a1 = a1;
			//_a2 = a2;
			// Figure out the base rotation from a1 = <1,0,0>, a2 = <0,1,0> that 
			// gives the current a1, a2.

			double thetar, betar, phir;
			// Theta is the angle between a1 and xlf
			// From dot product, theta = acos(a1.xlf)
			double dp = a1.get(2,0,0); // need only x component, as xlf is the unit vector in +x
			thetar = acos(dp);

			// From a1 = x_lf*cos(theta) + y_lf*sin(theta)*cos(phi) + z_lf*sin(theta)*sin(phi),
			// can use either y_lf or z_lf components to get phi
			double stheta = sin(thetar);
			if (thetar)
			{
				double acphi = a1.get(2,0,1) / stheta;
				phir = acos(acphi);

				// Finally, a2_x = -sin(theta)cos(beta)
				double cbeta = a2.get(2,0,0) / stheta * -1;
				betar = acos(cbeta);
			} else {
				// theta is zero, so gimbal locking occurs. assume phi = 0.
				phir = 0;
				// must use alternate definition to get beta
				double cosbeta = a2.get(2,0,1);
				betar = acos(cosbeta);
			}

			// thetar, betar, phir are in radians
			// convert to degrees
			{
				double scale = 180.0/(2.0*boost::math::constants::pi<double>());
				beta = betar * scale;
				theta = thetar * scale;
				phi = phir * scale;
			}

			// And figure out the effective rotation matrix of the existing file
			{
				double cb = cos(beta);
				double ct = cos(theta);
				double cp = cos(phi);
				double sb = sin(beta);
				double st = sin(theta);
				double sp = sin(phi);
				matrixop Rx(2,3,3), Ry(2,3,3), Rz(2,3,3);

				Rx.set(1,2,0,0);
				Rx.set(cp,2,1,1);
				Rx.set(cp,2,2,2);
				Rx.set(sp,2,2,1);
				Rx.set(-sp,2,1,2);

				Ry.set(cb,2,0,0);
				Ry.set(1 ,2,1,1);
				Ry.set(cb,2,2,2);
				Ry.set(sb,2,0,2);
				Ry.set(-sb,2,2,0);

				Rz.set(ct,2,0,0);
				Rz.set(ct,2,1,1);
				Rz.set(1,2,2,2);
				Rz.set(st,2,1,0);
				Rz.set(-st,2,0,1);

				rot = Ry*Rx*Rz;
				invrot = rot.inverse();
			}

			// Define statistics for max, min, mean, std dev, skewness, kurtosis, moment of inertia
			using namespace boost::accumulators;
			//using namespace boost::accumulators::tag;
			
			// Do two passes to be able to renormalize coordinates
			accumulator_set<double, stats<tag::mean, tag::min, tag::max> > m_x, m_y, m_z;
			for (auto it = _shp->_latticePtsStd.begin(); it != _shp->_latticePtsStd.end(); it++)
			{
				m_x(it->get(2,0,0));
				m_y(it->get(2,0,1));
				m_z(it->get(2,0,2));
			}

			b_min.set(boost::accumulators::min(m_x),2,0,0);
			b_min.set(boost::accumulators::min(m_y),2,1,0);
			b_min.set(boost::accumulators::min(m_z),2,2,0);

			b_max.set(boost::accumulators::max(m_x),2,0,0);
			b_max.set(boost::accumulators::max(m_y),2,1,0);
			b_max.set(boost::accumulators::max(m_z),2,2,0);

			b_mean.set(boost::accumulators::mean(m_x),2,0,0);
			b_mean.set(boost::accumulators::mean(m_y),2,1,0);
			b_mean.set(boost::accumulators::mean(m_z),2,2,0);


			// Figure out diameter of smallest circumscribing sphere
			convexHull cvHull(_shp->_latticePtsStd);
			cvHull.constructHull();
			max_distance = cvHull.maxDiameter();

			a_circum_sphere = max_distance / 2.0;
			V_circum_sphere = boost::math::constants::pi<double>() * 4.0 * pow(a_circum_sphere,3.0) / 3.0;
			SA_circum_sphere = boost::math::constants::pi<double>() * 4.0 * pow(a_circum_sphere,2.0);

			V_convex_hull = cvHull.volume();
			aeff_V_convex_hull = pow(3.0 * V_circum_sphere / (4.0 * boost::math::constants::pi<double>()),1./3.);
			SA_convex_hull = cvHull.surface_area();
			aeff_SA_convex_hull = pow(SA_convex_hull / (4.0 * boost::math::constants::pi<double>()),0.5);

			_currVersion = _maxVersion;
			_valid = true;


			// Calculate rotated stats to avoid having to duplicate code
			calcStatsRot(0,0,0);
			// From the 0,0,0 rotation,
			{
				// At beginning by default, as it is the only entry at this point!
				const shapeFileStatsRotated &dr = *(this->rotations.begin());
				V_ellipsoid_max = boost::math::constants::pi<double>() / 6.0;
				// Using diameters, and factor in prev line reflects this
				V_ellipsoid_max *= dr.max.get(2,0,0) - dr.min.get(2,0,0);
				V_ellipsoid_max *= dr.max.get(2,1,0) - dr.min.get(2,1,0);
				V_ellipsoid_max *= dr.max.get(2,2,0) - dr.min.get(2,2,0);

				V_ellipsoid_rms = 4.0 * boost::math::constants::pi<double>() / 3.0;
				V_ellipsoid_rms *= dr.rms_mean.get(2,0,0);
				V_ellipsoid_rms *= dr.rms_mean.get(2,1,0);
				V_ellipsoid_rms *= dr.rms_mean.get(2,2,0);

				aeff_ellipsoid_max = pow(3.0 * V_ellipsoid_max / (4.0 * boost::math::constants::pi<double>()),1./3.);
				aeff_ellipsoid_rms = pow(3.0 * V_ellipsoid_rms / (4.0 * boost::math::constants::pi<double>()),1./3.);
			}

			// Volume fractions
			f_circum_sphere = V_cell_const / V_circum_sphere;
			f_convex_hull = V_cell_const / V_convex_hull;
			f_ellipsoid_max = V_cell_const / V_ellipsoid_max;
			f_ellipsoid_rms = V_cell_const / V_ellipsoid_rms;

			// Densities
			rho_basic = 1.0;
			rho_circum_sphere = _N / V_circum_sphere;
			rho_convex = _N / V_convex_hull;
			rho_ellipsoid_max = _N / V_ellipsoid_max;
			rho_ellipsoid_rms = _N / V_ellipsoid_rms;


		}

		shapeFileStatsRotated::shapeFileStatsRotated()
			: min(2,3,1), max(2,3,1), sum(2,3,1), skewness(2,3,1), kurtosis(2,3,1), PE(2,3,1),
			mom1(2,3,1), mom2(2,3,1), mominert(2,3,3), covariance(2,3,3),
			abs_min(2,3,1), abs_max(2,3,1), abs_mean(2,3,1),
			as_abs(2,3,3), as_abs_mean(2,3,3), as_rms(2,3,3),
			rms_mean(2,3,1)
		{
			this->beta = 0;
			this->theta = 0;
			this->phi = 0;
		}

		shapeFileStatsRotated::~shapeFileStatsRotated()
		{
		}

		bool shapeFileStatsRotated::operator<(const shapeFileStatsRotated &rhs) const
		{
			if (beta!=rhs.beta) return beta<rhs.beta;
			if (theta!=rhs.theta) return theta<rhs.theta;
			if (phi!=rhs.phi) return phi<rhs.phi;
			return false;
		}

	}
}

//BOOST_CLASS_EXPORT_IMPLEMENT(rtmath::ddscat::shapeFileStatsRotated)
//BOOST_CLASS_EXPORT_IMPLEMENT(rtmath::ddscat::shapeFileStatsBase)
BOOST_CLASS_EXPORT_IMPLEMENT(rtmath::ddscat::shapeFileStats)
