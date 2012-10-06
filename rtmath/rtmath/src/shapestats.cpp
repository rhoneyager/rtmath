#include "../rtmath/Stdafx.h"

#include "../rtmath/matrixop.h"
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
			const boost::shared_ptr<const shapeFileStatsRotated> &rhs)
		{
			if (lhs->beta != rhs->beta) return lhs->beta < rhs->beta;
			if (lhs->theta != rhs->theta) return lhs->theta < rhs->theta;
			if (lhs->phi != rhs->phi) return lhs->phi < rhs->phi;
			return false;
		}

		shapeFileStats::shapeFileStats(const shapefile &shp)
		{
			_shp = boost::shared_ptr<shapefile>(new shapefile(shp));
		}

		shapeFileStats::shapeFileStats(const boost::shared_ptr<const shapefile> &shp)
		{
			_shp = boost::shared_ptr<shapefile>(new shapefile(*shp));
		}

		bool shapeFileStats::doQhull()
		{
			return _doQhull;
		}

		void shapeFileStats::doQhull(bool val)
		{
			_doQhull = val;
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

			_currVersion = _maxVersion;
			_valid = false;
			qhull_enabled = shapeFileStats::doQhull();

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
			const size_t _N = _shp->_numPoints;
			
			if (!_N)
			{
				throw rtmath::debug::xBadInput("Stats cannot be calculated because the shapefile is not loaded.");
			}

			// Calculate volume elements
			const matrixop &d = _shp->_d;
			double dxdydz = d.get(2,0,0) * d.get(2,0,1) * d.get(2,0,2);
			V_cell_const = dxdydz;
			V_dipoles_const = dxdydz * _N;
			aeff_dipoles_const = pow(V_dipoles_const*3./(4.*boost::math::constants::pi<double>()),1./3.);

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
				double scale = 180.0/(boost::math::constants::pi<double>());
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
			if (qhull_enabled)
			{
				cvHull.constructHull();
			} else {
				cvHull.hull_enabled = false;
			}
			max_distance = cvHull.maxDiameter();

			a_circum_sphere = max_distance / 2.0;
			V_circum_sphere = boost::math::constants::pi<double>() * 4.0 * pow(a_circum_sphere,3.0) / 3.0;
			SA_circum_sphere = boost::math::constants::pi<double>() * 4.0 * pow(a_circum_sphere,2.0);

			V_convex_hull = cvHull.volume();
			aeff_V_convex_hull = pow(3.0 * V_convex_hull / (4.0 * boost::math::constants::pi<double>()),1./3.);
			SA_convex_hull = cvHull.surface_area();
			aeff_SA_convex_hull = pow(SA_convex_hull / (4.0 * boost::math::constants::pi<double>()),0.5);

			_currVersion = _maxVersion;
			_valid = true;


			// Calculate rotated stats to avoid having to duplicate code
			// From the 0,0,0 rotation,
			{
				// At beginning by default, as it is the only entry at this point!
				auto pdr = calcStatsRot(0,0,0);
				V_ellipsoid_max = boost::math::constants::pi<double>() / 6.0;
				// Using diameters, and factor in prev line reflects this
				V_ellipsoid_max *= pdr->max.get(2,0,0) - pdr->min.get(2,0,0);
				V_ellipsoid_max *= pdr->max.get(2,1,0) - pdr->min.get(2,1,0);
				V_ellipsoid_max *= pdr->max.get(2,2,0) - pdr->min.get(2,2,0);

				V_ellipsoid_rms = 4.0 * boost::math::constants::pi<double>() / 3.0;
				V_ellipsoid_rms *= pdr->rms_mean.get(2,0,0);
				V_ellipsoid_rms *= pdr->rms_mean.get(2,1,0);
				V_ellipsoid_rms *= pdr->rms_mean.get(2,2,0);

				aeff_ellipsoid_max = pow(3.0 * V_ellipsoid_max / (4.0 * boost::math::constants::pi<double>()),1./3.);
				aeff_ellipsoid_rms = pow(3.0 * V_ellipsoid_rms / (4.0 * boost::math::constants::pi<double>()),1./3.);
			}

			// Volume fractions
			f_circum_sphere = V_dipoles_const / V_circum_sphere;
			f_convex_hull = V_dipoles_const / V_convex_hull;
			f_ellipsoid_max = V_dipoles_const / V_ellipsoid_max;
			f_ellipsoid_rms = V_dipoles_const / V_ellipsoid_rms;

		}

	}
}

