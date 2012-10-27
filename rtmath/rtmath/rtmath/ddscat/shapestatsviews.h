#pragma once

// Contains the defs for converting shape statistics into physical units

#include <vector>
#include <map>
#include <set>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/StdVector>
#include "shapestats.h"

// Using macro expansion here to define names
#define D_SCALED(varname, power) \
	double varname() const {\
		double ft = pow(_d, (double) power); \
		double val = _base->varname; \
		return val * ft; }
#define D_MAT_SCALED(varname, power) \
	matrixop varname() const {\
		double d = pow(_d, (double) power); \
		matrixop val = _base->varname; \
		matrixop res = val * d; \
		return res; }
#define D_MAT_SCALED_INDEXED(varname, power) \
	matrixop varname(size_t index) const {\
		double d = pow(_d, (double) power); \
		matrixop val = _base->varname[index]; \
		matrixop res = val * d; \
		return res; }
#define D_MAT_SCALED_INDEXED_OTHER(varname, power, multname) \
	matrixop varname(size_t index) const {\
		double d = pow(_d, (double) power); \
		matrixop val = _base->varname[index]; \
		matrixop res = val * d * multname[index]; \
		return res; }

namespace rtmath {
	namespace ddscat {

		class shapeFileStatsRotatedView : public shapeFileStatsRotated
		{
		public:
			shapeFileStatsRotatedView(const boost::shared_ptr<const shapeFileStatsRotated> &base, double dSpacing)
				: _base(base), _d(dSpacing) {}
			double getScale() const {return _d; }
			void setScale(double d) {_d = d;}
			void setMasses(const std::vector<double> &m) { _masses = m; }
			void getMasses(std::vector<double> &m) { m = _masses; }
			void setDensities(const std::vector<double> &den) { _densities = den; }
			void getDensities(std::vector<double> &den) { den = _densities; }

			// Accessor functions
			D_SCALED(beta, 0);
			D_SCALED(theta, 0);
			D_SCALED(phi, 0);

			// These are divided by constituent objects (distinct materials with 
			// each having its own mass and density).
			// mom1, mom2, covariance are fine by themselves
			D_MAT_SCALED_INDEXED(mom1, 1);
			D_MAT_SCALED_INDEXED(mom2, 2);
			D_MAT_SCALED_INDEXED(covariance, 2);
			// Moment of inertia requires material density
			D_MAT_SCALED_INDEXED_OTHER(mominert, 3, _densities);
			// PE requires multiplication by material mass
			D_MAT_SCALED_INDEXED_OTHER(PE, 1, _masses);

			D_MAT_SCALED(min, 1);
			D_MAT_SCALED(max, 1);
			D_MAT_SCALED(sum, 1);

			D_MAT_SCALED(abs_min, 1);
			D_MAT_SCALED(abs_max, 1);
			D_MAT_SCALED(abs_mean, 1);
			D_MAT_SCALED(rms_mean, 1);

			D_MAT_SCALED(as_abs, 0);
			D_MAT_SCALED(as_abs_mean, 0);
			D_MAT_SCALED(as_rms, 0);

		private:
			double _d;
			const boost::shared_ptr<const shapeFileStatsRotated> _base;
			std::vector<double> _masses, _densities;
		};

		class shapeFileStatsDipoleView : public shapeFileStatsBase
		{
		public:
			shapeFileStatsDipoleView(const boost::shared_ptr<shapeFileStatsBase> &base, double dSpacing)
				: _base(base), _d(dSpacing) {}
			double getScale() const { return _d; }
			void setScale(double d) { _d = d; }

			// Accessor functions
//			rotations getRotDefault() const;
			
			D_SCALED(beta, 0);
			D_SCALED(theta, 0);
			D_SCALED(phi, 0);

			D_SCALED(V_cell_const, 3);
			D_SCALED(aeff_dipoles_const, 1);
			D_SCALED(V_dipoles_const, 3);
			D_SCALED(max_distance, 1);
			D_SCALED(a_circum_sphere, 1);
			D_SCALED(V_circum_sphere, 3);
			D_SCALED(SA_circum_sphere, 2);
			D_SCALED(V_convex_hull, 3);
			D_SCALED(aeff_V_convex_hull, 1);
			D_SCALED(SA_convex_hull, 2);
			D_SCALED(aeff_SA_convex_hull, 1);
			D_SCALED(V_ellipsoid_max, 3);
			D_SCALED(aeff_ellipsoid_max, 1);
			D_SCALED(V_ellipsoid_rms, 3);
			D_SCALED(aeff_ellipsoid_rms, 1);
			D_SCALED(f_circum_sphere, 0);
			D_SCALED(f_convex_hull, 0);
			D_SCALED(f_ellipsoid_max, 0);
			D_SCALED(f_ellipsoid_rms, 0);
		private:
			double _d;
			const boost::shared_ptr<shapeFileStatsBase> _base;
		};

	}
}

#undef D_SCALED
#undef D_MAT_SCALED
#undef D_MAT_SCALED_INDEXED
#undef D_MAT_SCALED_INDEXED_OTHER
