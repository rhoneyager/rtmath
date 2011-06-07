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
#include <vector>
#include <boost/shared_ptr.hpp>

namespace rtmath {

	namespace lbl {
		// I want the lbl code to be distinct from the rest of the rtmath code

		class isoselector {
		public:
			isoselector(unsigned int molecnum, unsigned int isonum, double abundance)
			{
				_molecnum = molecnum;
				_isonum = isonum;
				_abundance = abundance;
			}
			inline unsigned int molecnum() { return _molecnum; }
			inline unsigned int isonum() { return _isonum; }
			inline double abundance() { return _abundance; }
		protected:
			unsigned int _molecnum;
			unsigned int _isonum;
			double _abundance;
		};

		// Don't use a full class, as reading is too slow
		class specline {
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
			//static std::map<Qselector, double> Qmap;
			static std::vector< std::map<double, double> > Qmap;
			static std::vector< std::string> QmapNames;
			//static std::vector
			static std::vector<isoselector> abundanceMap;
			//static std::map<isoselector, double> abundanceMap;
			inline static double pRef() { return _pRef; }
		protected:
			static double _pRef;
			static void _loadHITRAN(const char* hitranpar);
			static void _loadMolparam(const char* molparam);
			static void _loadParsum(const char* parsum);
		};

	}; // end namespace lbl


}; // end namespace rtmath
