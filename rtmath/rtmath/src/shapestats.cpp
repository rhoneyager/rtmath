#include "../rtmath/Stdafx.h"
#pragma warning( disable : 4996 ) // -D_SCL_SECURE_NO_WARNINGS
#pragma warning( disable : 4244 ) // annoying double to float issues in boost

#include <set>
#include <boost/math/constants/constants.hpp>
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
#include <boost/filesystem.hpp>
#include <Eigen/Dense>

//#include "../rtmath/matrixop.h"
#include "../rtmath/ddscat/rotations.h"
#include "../rtmath/ddscat/shapefile.h"
#include "../rtmath/ddscat/shapestats.h"
#include "../rtmath/ddscat/hulls.h"
#include "../rtmath/error/debug.h"
#include "../rtmath/error/error.h"

namespace {
	bool _doQhull = true;
}

namespace rtmath {
	namespace ddscat {

		const unsigned int shapeFileStatsBase::_maxVersion = 0;

		bool rotComp::operator()(const boost::shared_ptr<const shapeFileStatsRotated> &lhs,
			const boost::shared_ptr<const shapeFileStatsRotated> &rhs) const
		{
			if (lhs->beta != rhs->beta) return lhs->beta < rhs->beta;
			if (lhs->theta != rhs->theta) return lhs->theta < rhs->theta;
			if (lhs->phi != rhs->phi) return lhs->phi < rhs->phi;
			return false;
		}

		shapeFileStats::shapeFileStats(const shapefile &shp)
		{
			_shp = boost::shared_ptr<shapefile>(new shapefile(shp));
			calcStatsBase();
			calcStatsRot(0,0,0);
		}

		shapeFileStats::shapeFileStats(const boost::shared_ptr<const shapefile> &shp)
		{
			_shp = boost::shared_ptr<shapefile>(new shapefile(*shp));
			calcStatsBase();
			calcStatsRot(0,0,0);
		}

		bool shapeFileStats::doQhull()
		{
			return _doQhull;
		}

		void shapeFileStats::doQhull(bool val)
		{
			_doQhull = val;
		}

		shapeFileStats::shapeFileStats() { }

		shapeFileStatsBase::shapeFileStatsBase()
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

			_currVersion = _maxVersion;
			_valid = false;
			qhull_enabled = shapeFileStats::doQhull();

			// Need to fill with something for serialization to work with
			//_shp = boost::shared_ptr<shapefile>(new shapefile);
		}

		shapeFileStatsBase::~shapeFileStatsBase() { }

		bool shapeFileStatsBase::load()
		{
			// Return true if shape is loaded or can be loaded (and load it)
			// Return false if shape CANNOT be loaded
			if (_shp->latticePts.size() ) return true;

			std::string fname = _shp->filename;
			if (boost::filesystem::exists(boost::filesystem::path(fname)))
			{
				boost::shared_ptr<shapefile> nshp(new shapefile(fname));
				_shp = nshp;
				return true;
			} else {
				return false;
			}
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
			const size_t _N = _shp->numPoints;
			
			if (!_N)
			{
				throw rtmath::debug::xBadInput("Stats cannot be calculated because the shapefile is not loaded.");
			}

			// Calculate volume elements
			float dxdydz = _shp->d(0) * _shp->d(1) * _shp->d(2);
			V_cell_const = dxdydz;
			V_dipoles_const = dxdydz * _N;
			aeff_dipoles_const = pow(V_dipoles_const*3.f/(4.f*boost::math::constants::pi<float>()),1.f/3.f);

			const Eigen::Array3f &a1 = _shp->a1;
			const Eigen::Array3f &a2 = _shp->a2;
			const Eigen::Array3f &a3 = _shp->a3;
			// Figure out the base rotation from a1 = <1,0,0>, a2 = <0,1,0> that 
			// gives the current a1, a2.

			float thetar, betar, phir;
			// Theta is the angle between a1 and xlf
			// From dot product, theta = acos(a1.xlf)
			float dp = a1(0); // need only x component, as xlf is the unit vector in +x
			thetar = acos(dp);

			// From a1 = x_lf*cos(theta) + y_lf*sin(theta)*cos(phi) + z_lf*sin(theta)*sin(phi),
			// can use either y_lf or z_lf components to get phi
			float stheta = sin(thetar);
			if (thetar)
			{
				float acphi = a1(1) / stheta;
				phir = acos(acphi);

				// Finally, a2_x = -sin(theta)cos(beta)
				float cbeta = a2(0) / stheta * -1.f;
				betar = acos(cbeta);
			} else {
				// theta is zero, so gimbal locking occurs. assume phi = 0.
				phir = 0;
				// must use alternate definition to get beta
				float cosbeta = a2(1);
				betar = acos(cosbeta);
			}

			// thetar, betar, phir are in radians
			// convert to degrees
			{
				double scale = 180.0/(boost::math::constants::pi<double>());
				beta = betar * scale;
				theta = thetar * scale;
				phi = phir * scale;
			}

			rotationMatrix<float>(theta, phi, beta, rot);
			invrot = rot.inverse();

			// Define statistics for max, min, mean, std dev, skewness, kurtosis, moment of inertia
			using namespace boost::accumulators;
			//using namespace boost::accumulators::tag;
			
			// Do two passes to be able to renormalize coordinates
			accumulator_set<float, stats<tag::mean, tag::min, tag::max> > m_x, m_y, m_z;
			//for (auto it = _shp->latticePtsStd.begin(); it != _shp->latticePtsStd.end(); ++it)
			for (size_t i = 0; i < _shp->numPoints; i++)
			{
				auto it = _shp->latticePts.block<1,3>(i,0);
				m_x((it)(0));
				m_y((it)(1));
				m_z((it)(2));
			}

			b_min(0) = boost::accumulators::min(m_x);
			b_min(1) = boost::accumulators::min(m_y);
			b_min(2) = boost::accumulators::min(m_z);

			b_max(0) = boost::accumulators::max(m_x);
			b_max(1) = boost::accumulators::max(m_y);
			b_max(2) = boost::accumulators::max(m_z);

			b_mean(0) = boost::accumulators::mean(m_x);
			b_mean(1) = boost::accumulators::mean(m_y);
			b_mean(2) = boost::accumulators::mean(m_z);

			// Figure out diameter of smallest circumscribing sphere
			convexHull cvHull(_shp->latticePtsStd);
			if (qhull_enabled)
			{
				cvHull.constructHull();
			} else {
				cvHull.hullEnabled(false);
			}
			max_distance = cvHull.maxDiameter();

			a_circum_sphere = max_distance / 2.0;
			V_circum_sphere = boost::math::constants::pi<float>() * 4.0f * pow(a_circum_sphere,3.0f) / 3.0f;
			SA_circum_sphere = boost::math::constants::pi<float>() * 4.0f * pow(a_circum_sphere,2.0f);

			V_convex_hull = cvHull.volume();
			aeff_V_convex_hull = pow(3.0 * V_convex_hull / (4.0f * boost::math::constants::pi<float>()),1.f/3.f);
			SA_convex_hull = cvHull.surfaceArea();
			aeff_SA_convex_hull = pow(SA_convex_hull / (4.0f * boost::math::constants::pi<float>()),0.5);

			_currVersion = _maxVersion;
			_valid = true;


			// Calculate rotated stats to avoid having to duplicate code
			// From the 0,0,0 rotation,
			{
				// At beginning by default, as it is the only entry at this point!
				auto pdr = calcStatsRot(0,0,0);
				V_ellipsoid_max = boost::math::constants::pi<float>() / 6.0f;
				// Using diameters, and factor in prev line reflects this
				V_ellipsoid_max *= pdr->max(0) - pdr->min(0);
				V_ellipsoid_max *= pdr->max(1) - pdr->min(1);
				V_ellipsoid_max *= pdr->max(2) - pdr->min(2);

				V_ellipsoid_rms = 4.0f * boost::math::constants::pi<float>() / 3.0f;
				V_ellipsoid_max *= pdr->max(0) - pdr->min(0);
				V_ellipsoid_max *= pdr->max(1) - pdr->min(1);
				V_ellipsoid_max *= pdr->max(2) - pdr->min(2);

				aeff_ellipsoid_max = pow(3.0f * V_ellipsoid_max / (4.0f * boost::math::constants::pi<float>()),1.f/3.f);
				aeff_ellipsoid_rms = pow(3.0f * V_ellipsoid_rms / (4.0f * boost::math::constants::pi<float>()),1.f/3.f);
			}

			// Volume fractions
			f_circum_sphere = V_dipoles_const / V_circum_sphere;
			f_convex_hull = V_dipoles_const / V_convex_hull;
			f_ellipsoid_max = V_dipoles_const / V_ellipsoid_max;
			f_ellipsoid_rms = V_dipoles_const / V_ellipsoid_rms;

		}

	}
}

