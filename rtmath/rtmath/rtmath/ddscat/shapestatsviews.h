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


namespace rtmath {
	namespace ddscat {

		class shapeFileStatsDipoleView : public shapeFileStatsBase
		{
		public:
			shapeFileStatsDipoleView(const boost::shared_ptr<const shapeFileStatsBase> &base, double dSpacing)
				: _base(base), _d(dSpacing) {}
			void getScale(double &d) { return _d; }
			void setScale(double d) { _d = d; }

			// Accessor functions
//			rotations getRotDefault() const;
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

