#pragma once
/* atmos.h - the class definitions for an atmosphere
   An atmosphere is a collection of layers from which doubling-adding
   calculations may give properties such as intensity, flux, 
   reflectance, transmittance, heating rate, ...

   A vector holds the atmospheric layers in order. Each layer is generated 
   at a specified wavelength (sets tau and single scattering albedo), and 
   the layers are added together using the adding method. See Liou for 
   several descriptions of the adding method (he repeats himself constantly)
   */

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

		class absorber
		{
		public:
			// The constructors act either as a dummy constructor, or they assign
			// the appropriate molecule id for the gas. 
			// Isoconc is overridable to allow for selection of method (lbl, band model, 
			// or Liu's sources.
			absorber() { _init(); }
			absorber(int molnum);
			absorber(const std::string &molecule);
			absorber(const atmoslayer &layer, double psfrac);
			virtual ~absorber();
			virtual double deltaTau(double nu) const = 0;
			double p() const { return _p; }
			double psfrac() const { return _psfrac; }
			double T() const { return _T; }
			double dz() const { return _dz; }
			void p(double newp) { _p = newp; }
			void psfrac(double newpsfrac) {_psfrac = newpsfrac;}
			void T(double newt) {_T = newt;}
			void dz(double newdz) {_dz = newdz;}
		protected:
			double _p;
			double _T;
			double _dz;
			double _psfrac;
			std::string _molecule;
			int _molnum;
		private:
			void _init();
		public:
			// Let's be specific. _wvtofreq converts wavenumbers of form cm^-1
			// into frequency in GHz. _freqtowv does the reverse.
			static double _wvtofreq(double wvnum);
			static double _freqtowv(double f);
		};

		class atmoslayer
		{
		public:
			atmoslayer() { _init(); }
			virtual ~atmoslayer() {};
			inline double p() const { return _p; }
			inline void p(double newp) { _p = newp; }
			inline double T() const { return _T; }
			inline void T(double newT) { _T = newT; }
			inline double dz() const { return _dz; }
			inline void dz(double newdz) { _dz = newdz; }
			double tau(double nu); // Calculates tau of the layer
			// absorbers needs to be kept as a pointer because it it pure virtual
			std::set<std::shared_ptr<absorber> > absorbers;
		protected:
			double _p;
			double _T;
			double _dz;
			void _init();
		};

	}; // end namespace atmos

}; // end namespace rtmath
