#pragma once

#include <set>
#include <map>
#include <string>
#include <vector>
#include <boost/shared_ptr.hpp>
#include "../error/debug.h"
//#include "atmos.h"

namespace rtmath {

	namespace atmos {
		class atmos;

		/// Namespace for line-by-line spectral models
		namespace lbl {
			
			/**
			* \brief Line-by-line reading of HITRAN files into a structure and determining
			* the appropriate optical depths for a set of layers at a given wavenumber.
			*
			* This part of the code requires the HITRAN database, including HITRAN##.par, parsum.dat and 
			* molparam.txt. These files, or subsets thereof, give the necessary line information.
			*
			* HITAN08.par and HITRAN12.par contain the individual line information.
			* parsum.dat contains the partition function (Q) for each isotope at each temperature.
			* molparam.txt provides the isotope abundances.
			**/
			namespace hitran
			{
				class isodata;
				class isoconc;

				namespace lineshape {
					enum shape {
						LORENTZIAN,
						VLECK_WEISSKOPF,
						VOIGHT,
						GAUSSIAN,
						NUM_SHAPES
					};
				}; // end namespace lineshape

				class isoselector {
				public:
					isoselector(std::string &name, unsigned int molecnum, unsigned int isonum, double abundance)
					{
						_molecnum = molecnum;
						_isonum = isonum;
						_abundance = abundance;
						_molecname = name;
					}
					inline int molecnum() { return _molecnum; }
					inline int isonum() { return _isonum; }
					inline double abundance() { return _abundance; }
					void molecname(std::string &target) { target = _molecname; }
				protected:
					int _molecnum;
					int _isonum;
					double _abundance;
					std::string _molecname;
				};

				// Don't use a full class, as reading is too slow
				class specline {
					//#include "debug_mem_class.h" // Do special specline debugging
				public: // Member vars
					// For speed, leave these in the open. Inlining has no point.
					double _nu;
					int _molecnum;
					int _isonum;
					double _S;
					double _gamAir;
					double _gamSelf;
					double _nAir;
					double _deltaAir;
					double _Eb;
				public: // Member functions
					/*
					double gamma(double p, double ps, double T);
					double nuShifted(double p);
					double f(double nu, double p, double ps, double T);
					double S(double T, unsigned int Qcol);
					double k(double nu, double p, double ps, double T, unsigned int Qcol);
					double deltaTau(double nu, double p, double ps, 
					double T, double abun, unsigned int Qcol, double dz);
					*/
				public: // Static vars and functions
					static void loadlines(const char* hitranpar, 
						const char* molparam, const char* parsum);
					//static std::map<Qselector, double> Qmap;
					//static std::vector< std::map<double, double> > Qmap;
					static double *Qmatrix;
					static unsigned int QTlow, QThigh;
					static unsigned int QnumRecords, QnumIsos;
					static std::vector< std::string> QmapNames;
					static specline *lines;
					static const unsigned int numrecs;
					static std::set<isodata*> linemappings;
					static lineshape::shape speclineShape;
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
					//#undef new
				};

				class isodata {
				public:
					// Blank class constructer used by specline
					isodata() {lines = NULL; valid = true;}
					// See linemappings for predefined molecule-isotope maps of lines
					//  these are generated by _doMappings() in specline.
					isodata(const std::string &molecule);
					~isodata() {}
					inline double abundance() const { return _abundance; }
					/*inline double Q(double T) 
					{ 
					int _T = (int) T; // Truncation necessary as Q is every int
					return _Q->at((double) _T);
					}*/
					//std::set<specline*> lines;
					specline* lines;
					bool valid;
					inline unsigned int numLines() const { return _numLines; }
					inline const char* molecule() const {return _molecule.c_str();}
					inline int molnum() const { return _molnum;}
					inline int isotope() const { return _isotope;}
					inline int isoorder() const { return _isoorder;}
				private:
					double _abundance;
					//std::map<double,double> *_Q;
					unsigned int _Qcol;
					std::string _molecule;
					int _isotope;
					int _molnum;
					int _isoorder;
					unsigned int _numLines;
					friend class specline;
					friend class lbllayer;
					friend class isoconc;
				};

				class isoconc 
				{
				public:
					isoconc() {}
					~isoconc() {}
					// Two convenient constructors to act as selectors
					isoconc(int molnum);
					isoconc(std::string &molecule);

					int molnumsrc;

					inline double psfrac() const { return _psfrac; }
					inline void psfrac(double newps) { _psfrac = newps; }
					inline double p() const { return *_p; }
					inline void p(double &newp) { _p = &newp; }
					inline double T() const { return *_T; }
					inline void T(double &newT) { _T = &newT; }
					//inline double abun() const { return _abun; }
					//inline void abun(double newabun) { _abun = newabun; }
					inline double dz() const { return *_dz; }
					inline void dz(double &newdz) { _dz = &newdz; }
					std::set<isodata*> isotopes;
					double deltaTau(double nu) const;
				private:
					double _psfrac;
					double *_p;
					double *_T;
					//unsigned int _Qcol;
					//std::map<double,double> *_Q;
					//double _abun;
					double *_dz;
				};

				class lbllayer {
					// This is a layer of the atmosphere.
					// It corresponds exactly with a doubling-adding layer.
					// This layer overlay provides the optical depth at the 
					// desired frequency. The atmos class gives it the 
					// appropriate lines and concentrations of gases.
				public:
					lbllayer() {}
					~lbllayer() {}
					// Isotopes contains the spectral lines of each isotope, 
					// and the molecular data. It does not, however, have the 
					// concentrations at a given level
					//std::set<isodata*> isotopes;
					std::set<isoconc*> isoconcentrations;
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
					//friend class rtmath::atmos; // Direct private var access necessary for
					// layer-initialization functions
				}; 

			}
		}
	}
}
