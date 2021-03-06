#pragma once

#include <set>
#include <map>
#include <string>
#include <vector>
#include <boost/shared_ptr.hpp>
//#include "error/debug.h"
#include "atmos.h"

namespace rtmath {
	namespace atmos {

		/** 
		* \brief Base class for calculating atmospheric absorption 
		* due to water vapor, oxygen and nitrogen. 
		*
		* This is an adaptation of Dr. Liu's code (in mwrt3/sub0/absorb.f),
		* which considers absorption in a bulk atmosphere. Since my code needs 
		* the absorption in a small layer, his code requires some adaptation.
		**/
		class absorber
		{
		public:
			// The constructors act either as a dummy constructor, or they assign
			// the appropriate molecule id for the gas. 
			// Isoconc is overridable to allow for selection of method (lbl, band model, 
			// or Liu's sources.
			absorber() { _init(); }
			//absorber(int molnum);
			//absorber(const std::string &molecule);
			// This is the preferred initializer, as it sets p, T, etc.:
			absorber(const atmoslayer &layer);
			virtual ~absorber();
			virtual absorber* clone() const = 0; // Annoying, but necessary
			virtual double deltaTau(double f) const = 0;
			double p() const { return *_p; }
			//double psfrac() const { return _psfrac; }
			double T() const { return *_T; }
			double dz() const { return *_dz; }
			double numConc() const { return _numConc; }
			virtual void numConc(double newnumConc) { _numConc = newnumConc; }
			inline void getName(std::string &name) { name = _molecule; }
			//void psfrac(double newpsfrac) {_psfrac = newpsfrac;}
			void setLayer(const atmoslayer &layer);
			// For O2 only, need water vapor density (in g/m^3)
			// because of O2 line broadening calculation
			void wvden(double newwvden) { _wvden = newwvden; } 
			double wvden() const { return _wvden; }
		protected:
			const double *_p;
			const double *_T;
			const double *_dz;
			const atmoslayer *_layer;
			std::string _molecule;
			int _molnum;
			double _numConc;
			double _wvden;
			/// Frequency range
			double _fr[2];
		private:
			void _init();
		public:
			/// Function converting relative humidity (%) to density (in g/m^3)
			static double _Vden(double T, double RH);
			/// Function converting density (g/m^3) to relative humidity (%)
			static double _RH(double T, double rhowat);
			/// Function calculating saturation vapor pressure (in hPa)
			static double _ewsat(double P, double T);
			/// Function calculating number density (number of molecules / m^3)
			static double _rho(double P, double T, double ew);
			/// Create an appropriate absorption class based on molecule and
			/// target frequency range
			/// \returns indicates success or failure
			static bool _findAbsorber (const std::string &molecule, 
				std::shared_ptr<absorber> &res, double frequency = 0);
		};

		// Might as well set some functions that handle absorption
		// b.n.: These functions are member functions because they 
		// have virtual class inheritance. However, though they are 
		// member functions, their classes have no member data. The
		// initialization is inline, so it doesn't matter too much.


		/** \brief Collision-induced absorption by Pardo et al. (2000)
		*
		* Pardo, J. R., E. Serabyn, and J. Cernicharo, 
		*  Submillimeter atmospheric transmission measurements
		* on Mauna Kea during extremely dry El Nino
		* consitions: Implications for broadband opacity 
		* contributions. J.Q.S.R.T., 67, 169-180, 2000.
		* 1.29 times of N2-N2 collision absorption to account
		* for N2-O2 and O2-O2 collisions.
		**/
		class collide : public absorber {
		public: 
			collide() { _molecule = "COLLIDE"; }
			virtual double deltaTau(double f) const;
			virtual collide* clone() const
			{
				return new collide(*this);
			}
		};

		/// N2 collision absorption
		/// P. Rosenkranz (1998)
		class abs_N2 : public absorber {
		public: 
			abs_N2() { _molecule = "N2"; }
			virtual double deltaTau(double f) const;
			virtual abs_N2* clone() const
			{
				return new abs_N2(*this);
			}
		};

		/// H2O absorption
		/// Rosenkranz (1998)
		class abs_H2O : public absorber {
		public: 
			abs_H2O() { _molecule = "H2O"; }
			virtual double deltaTau(double f) const;
			virtual abs_H2O* clone() const
			{
				return new abs_H2O(*this);
			}
		};

		/// O2 absorption
		/// Rosenkranz (1995)
		class abs_O2 : public absorber {
		public: 
			abs_O2() { _molecule = "O2"; }
			virtual double deltaTau(double f) const;
			virtual abs_O2* clone() const
			{
				return new abs_O2(*this);
			}
		};

	}
}



