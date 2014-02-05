#pragma once

// Contains the defs for converting shape statistics into physical units

#include <vector>
#include <map>
#include <set>
#include <Eigen/Core>
#include "shapestats.h"

// MSVC 2010 and 2012 do not implement the full decltype definition. They rely on an 
// older version, where I cannot use the :: scope operator.
#if _MSC_VER <= 1700
#include <utility>
#define decltype(...) \
	std::identity<decltype(__VA_ARGS__)>::type
#endif

// Macro expansion defines a function varname() that accesses the base object 
// and performs appropriate scaling. Works for default types and Eigen templates.
#define D_SCALED(varname, power) \
	decltype(_base->varname) varname() const {\
	float d = pow(_d, (float) power); \
	return _base->varname * d; }
// More complex template that can work with vectors of matrices 
#define D_SCALED_INDEXED(varname, power) \
	decltype(_base->varname)::value_type varname(size_t index) const {\
	float d = pow(_d, (float) power); \
	return _base->varname[index] * d; }
#define D_MAT_SCALED_INDEXED_OTHER(varname, power, multname) \
	matrixop varname(size_t index) const {\
	float d = pow(_d, (double) power); \
	matrixop val = _base->varname[index]; \
	matrixop res = val * d * multname[index]; \
	return res; }



namespace rtmath {
	namespace ddscat {
		namespace stats {

			class shapeFileStatsRotatedView //: public shapeFileStatsRotated
			{
			private: // moved to top of class for the MSVC parser for _base
				double _d;
				const boost::shared_ptr<const shapeFileStatsRotated> _base;
				std::vector<double> _masses, _densities;
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

				D_SCALED(mom1, 1);
				D_SCALED(mom2, 2);
				D_SCALED_INDEXED(covariance, 2);
				/*
				// Moment of inertia requires material density
				D_MAT_SCALED_INDEXED_OTHER(mominert, 3, _densities);
				// PE requires multiplication by material mass
				D_MAT_SCALED_INDEXED_OTHER(PE, 1, _masses);
				*/

				D_SCALED(min, 1);
				D_SCALED(max, 1);
				D_SCALED(sum, 1);

				D_SCALED(abs_min, 1);
				D_SCALED(abs_max, 1);
				D_SCALED(abs_mean, 1);
				D_SCALED(rms_mean, 1);

				D_SCALED(as_abs, 0);
				D_SCALED(as_abs_mean, 0);
				D_SCALED(as_rms, 0);

			};

			class shapeFileStatsDipoleView //: public shapeFileStatsBase
			{
			private:
				double _d;
				const boost::shared_ptr<shapeFileStatsBase> _base;
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
			};
		}
	}
}

#undef D_SCALED
#undef D_SCALED_INDEXED
#undef D_MAT_SCALED_INDEXED_OTHER
#undef decltype // undefine the bug fix

