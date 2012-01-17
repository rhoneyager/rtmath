#pragma once

/* atmoslayer - represents an atmospheric layer. Used by atmos, and uses absorber class. */
#include <vector>
#include <map>
#include "error/debug.h"
#include "da/daLayer.h"
#include "matrixop.h"
#include "da/damatrix.h"
#include <string>
#include <memory>

namespace rtmath {
	//class daLayer; // List it here for reference
	//namespace lbl { class lbllayer; }; // Listed for reference

	namespace atmos {

		class atmoslayer;
		class absorber;

		class atmoslayer
		{
		public:
			atmoslayer() { _init(); }
			atmoslayer(double p, double T, double dz);
			virtual ~atmoslayer() {};
			// Need special assignment and copy constructors due to
			// the absorbers set being irregular (use of unique_ptr)
			atmoslayer(const atmoslayer & rhs);
			atmoslayer & operator = (const atmoslayer&);
			//atmoslayer* clone() const;

			// Other functions
			inline double p() const { return _p; }
			inline void p(double newp) { _p = newp; }
			inline double T() const { return _T; }
			inline void T(double newT) { _T = newT; }
			inline double dz() const { return _dz; }
			inline void dz(double newdz) { _dz = newdz; }
			double tau(double nu) const; // Calculates tau of the layer
			// absorbers needs to be kept as a pointer because it it pure virtual
			// I don't want to allow layers to be arbitrarily copied, as this screws
			// with the absorber psfrac parameter.
			std::set<std::shared_ptr<absorber> > absorbers;
		protected:
			double _p;
			double _T;
			double _dz;
			void _init();
		private:
			friend class absorber;
		};

	}; // end namespace atmos

}; // end namespace rtmath
