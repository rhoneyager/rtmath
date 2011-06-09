#include "Stdafx.h"
#include <set>
#include <map>
#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include <cstdlib>
#include <cstdio>
#include <boost/shared_ptr.hpp>
#define _USE_MATH_DEFINES
#include <math.h>
#include <cmath>
#include "lbl.h"

namespace rtmath {
	namespace lbl {
		// Define the static variables here
		double specline::_pRef = 1.0;
		double specline::_TRef = 296.0;
		//std::set<specline*> specline::lines;
		//std::map<Qselector, double> specline::Qmap;
		std::vector< std::map<double, double> > specline::Qmap;
		std::vector< std::string> specline::QmapNames;
		std::vector<isoselector> specline::abundanceMap;
		std::set<isodata*> specline::linemappings;
		specline* specline::lines = NULL;
		const unsigned int specline::numrecs = 2713968;

		double specline::gamma(double p, double ps, double T)
		{
			double Tscale = pow(_TRef/T,_nAir);
			double gair = _gamAir * (p - ps);
			double gsel = _gamSelf * ps;
			double res = Tscale * (gair + gsel);
			return res;
		}

		double specline::nuShifted(double p)
		{
			return _nu + _deltaAir * p;
		}

		double specline::f(double nu, double p, double ps, double T)
		{
			// Assuming Lorentx Profile, for now
			// TODO: find where Voight/Lorentz shift occurs when 
			// looking over a large range of wavelengths
			double num = gamma(p,ps,T);
			double denom = pow(gamma(p,ps,T),2.0) + pow(nuShifted(p),2.0);
			double res = num / (M_PI * denom);
			return res;
		}

		double specline::S(double T, std::map<double,double> *Q)
		{
			double Qquo = Q->at(_TRef) / Q->at(T);
			double cb = -1.4388; // cm*K
			double Equo = exp(cb*_Eb/T) / exp(cb*_Eb/_TRef);
			double nuquo = (1.0 - exp(cb*_nu/T)) / 
				(1.0 - exp(cb*_nu/_TRef));
			double res = _S * Qquo * Equo * nuquo;
			return res;
		}

		double specline::k(double nu, double p, double ps, 
			double T, std::map<double,double> *Q)
		{
			double Sres = S(T,Q);
			double fres = f(nu,p,ps,T);
			double res = Sres * fres;
			return res;
		}

		double specline::deltaTau(double nu, double p, double ps, 
			double T, double abun, std::map<double,double> *Q, double dz)
		{
			double kres = k(nu,p,ps,T,Q);
			// Boltzmann's constant:
			// TODO: check units is ps in atm, T in K
			const double kb=1.3806503e-23; // m^2 kg s^-2 K^-1
			// Convert ps from atm to Pa for units
			double nai = abun * 101325 * ps / (kb*T);
			// kres has units of 1/(molecule cm^-2)
			double res = nai * kres * dz;
			throw;
			return res;
		}

		void specline::loadlines(const char* hitranpar, 
			const char* molparam, const char* parsum)
		{
			// If a string is null, skip that step
			if (hitranpar[0]) _loadHITRAN(hitranpar);
			if (molparam[0]) _loadMolparam(molparam);
			if (parsum[0]) _loadParsum(parsum);
			if (lines) _doMappings();
		}

		void specline::_loadHITRAN(const char* hitranpar)
		{
			// This algorithm is too slow at first
			// Preallocate memory to speed it up
			//const unsigned int numrecs = 2713968; // Now a static var
			specline *inlines = new specline[numrecs];
			lines = inlines;
			// Also, use low-level C functions like fread
			// getchar locks the io stream each time it is called
			// fread only does one locking

			// Slow point is atof, atoi, but these are needed, as the records 
			// have numbers in exp notation (3.02e48)
			// Time spent in routine is ~50 seconds on windows, which is acceptable
			// If only doing atoi for mol and isotop number, takes just 9 secs.
			// With no assignments, takes only 6 secs.

			// This algorithm will do a continuous read of 400 records
			// at a time. Uses one fread call to fill up almost 64 kb.
			// 64 kb is a boundary as many systems have a page fault past here, 
			// so it slows down and gives an equivalent timed result

			const unsigned int nRecsinread = 400;
			// Size is 161 because of line feed character
			const unsigned int recSize = 161;

			// Calc the critical read number (for special treatment)
			const unsigned int numFullReads = numrecs/nRecsinread;
			const unsigned int recRem = numrecs - numFullReads * nRecsinread;

			// Open the hitran file
			using namespace std;

			ifstream indata(hitranpar);
			if (indata.good() == false) throw;
			if (indata.eof()) throw;
			if (indata.bad()) throw;
			if (indata.fail()) throw;

			// The line containing the data
			char inset[recSize*nRecsinread];
			char *record = inset;
			//char newvalue[20];
			specline *linep = inlines;
			unsigned int k=0;
			unsigned int numcurrRecs = nRecsinread;
			while (indata.eof() == false)
			{
				// Read in the first increment
				// If eof is reached, the read stops there
				// Note: no null character gets appended
				indata.read(inset,recSize*nRecsinread);
				
				// On last read, if near end of file, change number 
				// of records to insert
				k += numcurrRecs;
				// Logic check to break loop
				if (k == numrecs) break;
				if (k/numcurrRecs >= numFullReads) 
					numcurrRecs = numrecs % k; // get a remainder
				// omp parallel for requires integer iterator
#pragma omp parallel for private(record,linep)
				for (int i=0;i< (int) numcurrRecs;i++)
				{
					linep = &inlines[k+i];
					// Do the actual record parsing
					record = &inset[i*recSize];
					char newvalue[20];
					// Copy values for conversion into newvalue[20]
					// Ensure that they are null-terminated
					// Then, call atof or atoi and insert into array
					
					// I am casting record to allow pointer addition
					
					// Molecule number
					strncpy(newvalue,((char*) record) ,2);
					newvalue[2] = '\0';
					linep->_molecnum = atoi(newvalue);

					// Isotopologue number
					strncpy(newvalue,((char*) record) + 2,1);
					newvalue[1] = '\0';
					linep->_isonum = atoi(newvalue);

					// Vacuum wavenumber
					strncpy(newvalue,((char*) record) + 3,12);
					newvalue[12] = '\0';
					linep->_nu = atof(newvalue);

					// S
					strncpy(newvalue,((char*) record) + 15,10);
					newvalue[10] = '\0';
					linep->_S = atof(newvalue);

					// Gamma_air
					strncpy(newvalue,((char*) record) + 35,5);
					newvalue[5] = '\0';
					linep->_gamAir = atof(newvalue);

					// Gamma_self
					strncpy(newvalue,((char*) record) + 40,5);
					newvalue[5] = '\0';
					linep->_gamSelf = atof(newvalue);

					// E"
					strncpy(newvalue,((char*) record) + 45,10);
					newvalue[10] = '\0';
					linep->_Eb = atof(newvalue);

					// n_air
					strncpy(newvalue,((char*) record) + 55,4);
					newvalue[4] = '\0';
					linep->_nAir = atof(newvalue);

					// delta_air
					strncpy(newvalue,((char*) record) + 59,8);
					newvalue[8] = '\0';
					linep->_deltaAir = atof(newvalue);

					//linep++;
				}

			}

			indata.close();
			//std::cout << "Records read: " << j << std::endl;
		}

		void specline::_loadMolparam(const char* molparam)
		{
			using namespace std;
			ifstream indata(molparam);
			// molparam.txt is oddly-formatted
			// it has one line for a header
			// the next line, prefixed by three spaces, is the molecule, with number
			// the next several lines have records as:
			//  iso#, abundance, Q(296K), gj, Molar mass (g)
			// Then, a new molecule's entry begins
			// All I need are abundances, but I might as well keep the other data
			// NOTE: the isotope numbers are not the same as for PARSUM.
			string linein;
			// Skip the header
			getline(indata,linein);

			string molname;
			unsigned int molid;
			unsigned int isoid;
			double abundance;
			//double Q;
			//int gj;
			//double mmass;

			do 
			{
				getline(indata,linein);
				// Check for new molecule
				if (linein.size() < 5) continue;
				if (linein[5] != ' ')
				{
					// New molecule found. Loading id and number
					molname = linein.substr(0,7);
					// Trim molname
					molname.erase(molname.find_last_not_of(' ')+1,molname.size());
					molname.erase(0,molname.find_first_not_of(' '));
					molid = atoi( linein.substr(8).c_str() );
				} else {
					// Check for new isotope
					if (linein.size() < 11) continue;
					if (linein[11] != ' ')
					{
						// New isotope found
						isoid = atoi( linein.substr(8,4).c_str() );
						abundance = atof( linein.substr(14,10).c_str() );
						//Q = atof( linein.substr(29,9).c_str() );
						//gj = atoi( linein.substr(42,2).c_str() );
						//mmass = atof( linein.substr(49,8).c_str() );

						// Add the isotope to the list
						abundanceMap.push_back(isoselector(molname, molid,isoid,abundance));
						//abundanceMap[isoselector(molid,isoid)] = abundance;
						// Yeah, I know that the rest is ignored.
					}
				}
			} while (linein.size());
			indata.close();
		}

		void specline::_loadParsum(const char* parsum)
		{
			// This is a big spreadsheet of data values
			// Contains T from 70K to 3000K
			// Contains all isotopes in molparam.txt
			// But of course, the isotopes are not quite the same as in 
			// molparams.txt. Crap.
			
			// _loadMolparam has already run, so that vector is done
			//Qmap.resize(abundanceMap.size());
			using namespace std;
			ifstream indata(parsum);

			string linein;
			// Parse the first line to get atoms and isotopes
			getline(indata,linein);
			std::istringstream parser(linein, istringstream::in);
			std::vector<string> molecisoids;
			string namein;
			indata >> namein; // Skip Temp(K) in file
			getline(indata,namein);
			parser.str(namein);
			// Expand around whitespace
			while (parser.good())
			{
				parser >> namein;
				molecisoids.push_back(namein);
			}

			Qmap.resize(molecisoids.size());
			QmapNames = molecisoids;

			// Expand Qmap names, so that molec/isotop lookup works
			// Using find and substr
			/*
			for (unsigned int i=0;i<QmapNames.size();i++)
			{
				size_t seploc = QmapNames[i].find('_');
				string mname = QmapNames[i].substr(0,seploc);
				unsigned int isotop = atoi( QmapNames[i].substr(seploc+1).c_str());
				// TODO: complete here if desired. The functionality is unused
			}
			*/

			// Iterate until eof, reading temperatures and appropriate values
			while ( indata.good())
			{
				double tempK;
				indata >> tempK;
				for (unsigned int i=0;i<abundanceMap.size();i++)
				{
					double Q;
					indata >> Q;
					Qmap.at(i)[tempK] = Q;
				}
			}

			indata.close();
		}

		void specline::_doMappings()
		{
			// TODO: optimize this, since it's important and takes a while
			// Function creates the mappings between molparams, parsum and the lines.
			// This is necessary because each file uses different identifiers for the data.
			// This function assumes that molparam has the complete set of interesting 
			// molecules (it is essential for the others to work).
			// It finds the matching parsum entry, and selects the appropriate lines 
			//std::set<isodata*> linemappings;
			// Each isodata entry contains the isotope data for one isotope of a molecule
			// It contains the mapping of partition function values, the abundance, and 
			// the set of spectral lines. Pointers here are essential.

			unsigned int pmolec=0,piso=0;
			// Loop through the abundance map
			for (unsigned int i=0;i<abundanceMap.size();i++)
			{
				// Create a new isodata
				isodata *newiso = new isodata();

				// Set the basic parameters
				// Note: loading molparams gives isodata from greatest to least abundances!
				if (pmolec != abundanceMap[i].molecnum())
				{
					// New molecule
					pmolec = abundanceMap[i].molecnum();
					piso = 1;
				} else {
					// Same molecule. Increment isonum
					piso++;
				}
				newiso->_abundance = abundanceMap[i].abundance();
				abundanceMap[i].molecname(newiso->_molecule);
				newiso->_isotope = abundanceMap[i].isonum();
				newiso->_molnum = abundanceMap[i].molecnum();
				newiso->_isoorder = piso; // Set order for line lookup

				// Select the appropriate Q parameter set
				std::ostringstream qsstr; // I'm combining an int with a string
				std::string qstr;
				qsstr << newiso->_molecule << "_" << newiso->_isotope;
				qstr = qsstr.str(); // Make stringstream into string
				// Search for the entry
				unsigned int target = 0;
				for (unsigned int j=0;j<QmapNames.size();j++)
				{
					if (QmapNames[j] == qstr) target = j;
				}
				// Link _Q with Q[target]
				newiso->_Q = (&Qmap[target]);

				// Select the appropriate HITRAN lines
				// Must iterate over all lines, so code should be FAST!
				for (unsigned int k=0;k<numrecs;k++)
				{
					if (lines[k]._molecnum == newiso->_molnum)
					{
						if (lines[k]._isonum == newiso->_isoorder)
						{
							// This is a line that fits the criteria
							newiso->lines.insert(&lines[k]);
						}
					}
				}
				// And this abundance entry is completed!
				linemappings.insert(newiso);
			}
			// We've looped through all isotopes, so the mappings are complete.
		}

	}; // end lbl


}; //end rtmath


