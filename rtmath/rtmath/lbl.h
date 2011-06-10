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

		class isodata;
		class isoconc;

		class isoselector {
		public:
			isoselector(std::string &name, unsigned int molecnum, unsigned int isonum, double abundance)
			{
				_molecnum = molecnum;
				_isonum = isonum;
				_abundance = abundance;
				_molecname = name;
			}
			inline unsigned int molecnum() { return _molecnum; }
			inline unsigned int isonum() { return _isonum; }
			inline double abundance() { return _abundance; }
			void molecname(std::string &target) { target = _molecname; }
		protected:
			unsigned int _molecnum;
			unsigned int _isonum;
			double _abundance;
			std::string _molecname;
		};

		// Don't use a full class, as reading is too slow
		class specline {
		public: // Member vars
			// For speed, leave these in the open. Inlining has no point.
			double _nu;
			unsigned int _molecnum;
			unsigned int _isonum;
			double _S;
			double _gamAir;
			double _gamSelf;
			double _nAir;
			double _deltaAir;
			double _Eb;
		public: // Member functions
			double gamma(double p, double ps, double T);
			double nuShifted(double p);
			double f(double nu, double p, double ps, double T);
			double S(double T, std::map<double,double> *Q);
			double k(double nu, double p, double ps, double T, std::map<double,double> *Q);
			double deltaTau(double nu, double p, double ps, 
				double T, double abun, std::map<double,double> *Q, double dz);
		public: // Static vars and functions
			static void loadlines(const char* hitranpar, 
				const char* molparam, const char* parsum);
			//static std::map<Qselector, double> Qmap;
			static std::vector< std::map<double, double> > Qmap;
			static std::vector< std::string> QmapNames;
			static specline *lines;
			static const unsigned int numrecs;
			static std::set<isodata*> linemappings;
			//static std::vector
			static std::vector<isoselector> abundanceMap;
			//static std::map<isoselector, double> abundanceMap;
			inline static double pRef() { return _pRef; }
			inline static double TRef() { return _TRef; }
		protected:
			static double _pRef;
			static double _TRef;
			static void _loadHITRAN(const char* hitranpar);
			static void _loadMolparam(const char* molparam);
			static void _loadParsum(const char* parsum);
			static void _doMappings();
		};

		class isodata {
		public:
			// Blank class constructer used by specline
			isodata() {}
			// Standard constructor used by others, which pulls from specline
			// TODO: implement
			isodata(unsigned int atom, unsigned int isoorder); 
			~isodata() {}
			inline double abundance() const { return _abundance; }
			inline double Q(double T) 
			{ 
				int _T = (int) T; // Truncation necessary as Q is every int
				return _Q->at((double) _T);
			}
			std::set<specline*> lines;
			inline const char* molecule() const {return _molecule.c_str();}
			inline unsigned int molnum() const { return _molnum;}
			inline unsigned int isotope() const { return _isotope;}
			inline unsigned int isoorder() const { return _isoorder;}
		private:
			double _abundance;
			std::map<double,double> *_Q;
			std::string _molecule;
			unsigned int _isotope;
			unsigned int _molnum;
			unsigned int _isoorder;
			friend class specline;
			friend class lbllayer;
			friend class isoconc;
		};

		class isoconc 
		{
		public:
			isoconc() {}
			~isoconc() {}
			inline double ps() const { return _ps; }
			inline void ps(double newps) { _ps = newps; }
			inline double p() const { return _p; }
			inline void p(double newp) { _p = newp; }
			inline double T() const { return _T; }
			inline void T(double newT) { _T = newT; }
			inline double abun() const { return _abun; }
			inline void abun(double newabun) { _abun = newabun; }
			inline double dz() const { return _dz; }
			inline void dz(double newdz) { _dz = newdz; }
			std::set<isodata*> isotopes;
			double deltaTau(double nu) const;
		private:
			double _ps;
			double _p;
			double _T;
			std::map<double,double> *_Q;
			double _abun;
			double _dz;
		};

		class lbllayer {
		// This is a layer of the atmosphere.
		// It corresponds exactly with a doubling-adding layer.
		// This layer overlay provides the optical depth at the 
		// desired frequency. The atmos class gives it the 
		// appropriate lines and concentrations of gases.
		public:
			lbllayer();
			~lbllayer();
			// Isotopes contains the spectral lines of each isotope, 
			// and the molecular data. It does not, however, have the 
			// concentrations at a given level
			//std::set<isodata*> isotopes;
			std::set<isoconc> isoconcentrations;
			inline double p() const { return _p; }
			inline void p(double newp) { _p = newp; }
			inline double T() const { return _T; }
			inline void T(double newT) { _T = newT; }
			inline double dz() const { return _dz; }
			inline void dz(double newdz) { _dz = newdz; }
			double tau(double nu); // Calculates tau of the layer
		private:
		protected:
			double _p;
			double _T;
			double _dz;
		}; 

	}; // end namespace lbl


}; // end namespace rtmath
