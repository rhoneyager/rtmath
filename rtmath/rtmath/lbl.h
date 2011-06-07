#pragma once
/* lbl.h - header file for line-by-line reading of HITRAN files into a structure and determining
 * the appropriate optical depths for a set of layers at a given wavenumber.
 * This part of the code requires the HITRAN database, including HITRAN08.par, parsum.dat and 
 * molparam.txt. These files, or subsets thereof, give the necessary line information.
 *
 * HITAN08.par contins the individual line information.
 * parsum.dat contains the partition function (Q) for each isotope at each temperature.
 * molparam.txt provides the isotope abundances.
 */

#include <set>
#include <map>
#include <string>
#include <boost/shared_ptr.hpp>

namespace rtmath {

	namespace lbl {
		// I want the lbl code to be distinct from the rest of the rtmath code

		class Qselector {
		public:
			Qselector(unsigned int molecnum, unsigned int isonum, double tempK)
			{
				_molecnum = molecnum;
				_isonum = isonum;
				_tempK = tempK;
			}
			inline unsigned int molecnum() { return _molecnum; }
			inline unsigned int isonum() { return _isonum; }
			inline double tempK() { return _tempK; }
		protected:
			unsigned int _molecnum;
			unsigned int _isonum;
			double _tempK;
		};

		class isoselector {
		public:
			isoselector(unsigned int molecnum, unsigned int isonum)
			{
				_molecnum = molecnum;
				_isonum = isonum;
			}
			inline unsigned int molecnum() { return _molecnum; }
			inline unsigned int isonum() { return _isonum; }
		protected:
			unsigned int _molecnum;
			unsigned int _isonum;
		};

		// Don't use a full class, as reading is too slow
		class specline {
		public:
/*
			specline() {}
			specline(unsigned int molecnum, unsigned int isonum,
				double nu, double S, double gamAir, double gamSelf,
				double nAir, double deltaAir, double Eb);
			void set(std::string &parLine);
			specline(std::string &parLine) { set(parLine); }
			~specline();
			inline double nu() { return _nu; }
			inline unsigned int molecnum() { return _molecnum; }
			inline unsigned int isonum() { return _isonum; }
			inline double S() { return _S; }
			inline double gamAir() {return _gamAir; }
			inline double gamSelf() { return _gamSelf; }
			inline double nAir() { return _nAir; }
			inline double deltaAir() { return _deltaAir; }
			inline double Eb() { return _Eb; }
		protected:
*/
		public:
			double _nu;
			unsigned int _molecnum;
			unsigned int _isonum;
			double _S;
			double _gamAir;
			double _gamSelf;
			double _nAir;
			double _deltaAir;
			double _Eb;
		public:
			static void loadlines(const char* hitranpar, 
				const char* molparam, const char* parsum);
			//static std::set<specline*> lines;
			static std::map<Qselector, double> Qmap;
			static std::map<isoselector, double> abundanceMap;
			inline static double pRef() { return _pRef; }
		protected:
			static double _pRef;
			static void _loadHITRAN(const char* hitranpar);
			static void _loadMolparam(const char* molparam);
			static void _loadParsum(const char* parsum);
		};

	}; // end namespace lbl


}; // end namespace rtmath
