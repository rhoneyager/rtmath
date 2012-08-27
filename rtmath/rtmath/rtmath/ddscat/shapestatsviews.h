#pragma once

// Contains the defs for converting shape statistics into physical units

#include <vector>
#include <map>
#include <set>
#include "../matrixop.h"
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


namespace rtmath {
	namespace ddscat {

		class shapeFileStatsRotatedView : public shapeFileStatsRotated
		{
		public:
			shapeFileStatsRotatedView(const boost::shared_ptr<const shapeFileStatsRotated> &base, double dSpacing)
				: _base(base), _d(dSpacing) {}
			double getScale() const {return _d; }
			void setScale(double d) {_d = d;}

			// Accessor functions
			D_SCALED(beta, 0);
			D_SCALED(theta, 0);
			D_SCALED(phi, 0);

			// D_MAT_SCALED(mom1, 1);
			// D_MAT_SCALED(mom2, 1);
			// D_MAT_SCALED(mominert, 1);
//			D_MAT_SCALED(PE, 2); // TODO: check

			D_MAT_SCALED(min, 1);
			D_MAT_SCALED(max, 1);
			D_MAT_SCALED(sum, 1);

			D_MAT_SCALED(abs_min, 1);
			D_MAT_SCALED(abs_max, 1);
			D_MAT_SCALED(abs_mean, 1);
			D_MAT_SCALED(rms_mean, 1);

			D_MAT_SCALED(as_abs, 0);
			D_MAT_SCALED(as_abs_mean, 0);
			D_MAT_SCALED(as_abs_rms, 0);
		};

		class shapeFileStatsDipoleView : public shapeFileStatsBase
		{
		public:
			shapeFileStatsDipoleView(const boost::shared_ptr<const shapeFileStatsBase> &base, double dSpacing)
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
			D_SCALED(rho_basic, 0);
			D_SCALED(rho_circum_sphere, 0);
			D_SCALED(rho_convex, 0);
			D_SCALED(rho_ellipsoid_max, 0);
			D_SCALED(rho_ellipsoid_rms, 0);
		private:
			double _d;
			const boost::shared_ptr<const shapeFileStatsBase> _base;
		};

	}
}

#undef D_SCALED
#undef D_MAT_SCALED

