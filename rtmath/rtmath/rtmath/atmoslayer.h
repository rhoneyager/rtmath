#pragma once

/* atmoslayer - represents an atmospheric layer. Used by atmos, and uses absorber class. */
#include <vector>
#include <map>
#include <string>
#include <memory>
//#include "error/debug.h"
#include "da/daLayer.h"
#include "matrixop.h"
#include "da/damatrix.h"
#include "ddscat/ddLoader.h"

namespace rtmath {

	namespace atmos {

		class atmoslayer;
		class absorber;

		class atmoslayer
		{
		public:
			atmoslayer() { _init(); }
			atmoslayer(double p, double T, double z, double dz, double rho);
			virtual ~atmoslayer() {};
			// Need special assignment and copy constructors due to
			// the absorbers set being irregular (use of unique_ptr)
			atmoslayer(const atmoslayer & rhs);
			atmoslayer & operator = (const atmoslayer&);
			//atmoslayer* clone() const;

			// Other optical depth-based functions
			inline double p() const { return _p; }
			inline void p(double newp) { _p = newp; }
			// Will think of a better name to avoid conflicts of underatanding with
			// doubling-adding stuff
			inline double T() const { return _T; } 
			inline void T(double newT) { _T = newT; }
			inline double z() const { return _z; }
			inline void z(double newz) { _z = newz; }
			inline double dz() const { return _dz; }
			inline void dz(double newdz) { _dz = newdz; }
			// rho is number density (# of gas molecules per volume)
			inline double rho() const { return _rho; }
			inline void rho(double newrho) { _rho = newrho; }
			// wvden is density of water vapor (g/m^3)
			inline double wvden() const { return _wvden; }
			inline void wvden(double newwvden) { _wvden = newwvden; }

			double tau(double f) const; // Calculates tau of the layer
			// absorbers needs to be kept as a pointer because it it pure virtual
			// I don't want to allow layers to be arbitrarily copied, as this screws
			// with the absorber psfrac parameter.
			std::set<std::shared_ptr<absorber> > absorbers;
		protected:
			double _p;
			double _T;
			double _z;
			double _dz;
			double _rho;
			double _wvden;
			void _init();

			// The doubling-adding functions (FUN)
		public:
			// Calculate transmission and reflection matrices
			void da_T(std::shared_ptr<const damatrix> &res) const;
			void da_R(std::shared_ptr<const damatrix> &res) const;


			// Allow for setting of pf loader
			void setPfLoader(std::unique_ptr<rtmath::ddLoader> newLoader);
			// Reading pf and related quantities - these alias to _pfLoader's functions!
			void pf(std::shared_ptr<const damatrix> &pF) const;
			void K(std::shared_ptr<const damatrix> &k) const;
			void emV(std::shared_ptr<const damatrix> &emv) const;


		protected:
			std::shared_ptr<const damatrix> _baseP, _baseK, _baseEmV;
			std::shared_ptr<const damatrix> _effT, _effR;
			std::unique_ptr<rtmath::ddLoader> _pfLoader;
		private:
			friend class absorber;
		};

	}; // end namespace atmos

}; // end namespace rtmath
