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
#include <string.h>
#include "lbl.h"
#include "debug.h"
#include "../rtmath-base/macros.h"

namespace rtmath {
	namespace lbl {
		// Define the static variables here
		double specline::_pRef = 1.0;
		double specline::_TRef = 296.0;
		//std::set<specline*> specline::lines;
		//std::map<Qselector, double> specline::Qmap;
		double *specline::Qmatrix = NULL;
		unsigned int specline::QTlow = 0, specline::QThigh = 0, 
			specline::QnumIsos = 0, specline::QnumRecords = 0;
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

		double specline::S(double T, unsigned int Qcol)
		{
			// Approximate T to the truncated int due to roundoff of parsum
			unsigned int _T = (unsigned int) T;
			// Find the Tref and T Q values
			double Qref, Q;
			Qref = Qmatrix[(QnumIsos+1)*(296-QTlow) + Qcol + 1];
			Q = Qmatrix[(QnumIsos+1)*(_T-QTlow) + Qcol + 1];
			double Qquo = Qref / Q;
			double cb = -1.4388; // cm*K // TODO: unit check
			double Equo = exp(cb*_Eb/T) / exp(cb*_Eb/_TRef);
			double nuquo = (1.0 - exp(cb*_nu/T)) / 
				(1.0 - exp(cb*_nu/_TRef));
			double res = _S * Qquo * Equo * nuquo;
			//throw; // Unit check must be done
			return res;
		}

		double specline::k(double nu, double p, double ps, 
			double T, unsigned int Qcol)
		{
			double Sres = S(T,Qcol);
			double fres = f(nu,p,ps,T);
			double res = Sres * fres;
			return res;
		}

		double specline::deltaTau(double nu, double p, double ps, 
			double T, double abun, unsigned int Qcol, double dz)
		{
			double kres = k(nu,p,ps,T,Qcol);
			// Boltzmann's constant:
			// TODO: check units is ps in atm, T in K
			const double kb=1.3806503e-23; // m^2 kg s^-2 K^-1
			// Convert ps from atm to Pa for units
			double nai = abun * 101325 * ps / (kb*T);
			// kres has units of 1/(molecule cm^-2)
			double res = nai * kres * dz;
			//throw; // Unit check must be done first
			return res;
		}

		double isoconc::deltaTau(double nu) const
		{
			// Calculate deltaTau caused by a set of isotope lines
			// at a certain frequency, temp, pressure, concentration, 
			// etc. Sum them all up and return.
			double res = 0.0;
			std::set<isodata*>::iterator it;
			std::set<specline*>::iterator line;
			for (it = isotopes.begin(); it != isotopes.end(); it++)
			{
				// Isoconc provides lines
				// Isodata does not provide tau - it's useless, and 
				// isoconc calls the lines directly
				double abun = (*it)->abundance();

				for(line = (*it)->lines.begin(); line != (*it)->lines.end(); line++)
				{
					res += (*line)->deltaTau(nu, *_p, *_p * _psfrac, *_T, abun, (*it)->_Qcol, *_dz);
				}
			}
			return res;
		}

		double lbllayer::tau(double nu)
		{
			double res = 0.0;
			std::set<isoconc*>::iterator it;
			for (it = isoconcentrations.begin(); 
				it != isoconcentrations.end(); it++)
			{
				res += (*it)->deltaTau(nu);
			}

			return res;
		}

		void specline::loadlines(const char* hitranpar, 
			const char* molparam, const char* parsum)
		{
			// If a string is null, skip that step
			debug::timestamp(false);
			if (hitranpar[0]) _loadHITRAN(hitranpar);
			debug::timestamp(true);
			if (molparam[0]) _loadMolparam(molparam);
			debug::timestamp(true);
			if (parsum[0]) _loadParsum(parsum);
			debug::timestamp(true);
			if (lines) _doMappings();
			debug::timestamp(true);
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

			// Slow point is M_ATOF, M_ATOI, but these are needed, as the records 
			// have numbers in exp notation (3.02e48)
			// Time spent in routine is ~50 seconds on windows, which is acceptable
			// If only doing M_ATOI for mol and isotop number, takes just 9 secs.
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
					// Then, call M_ATOF or M_ATOI and insert into array
					
					// I am casting record to allow pointer addition
					
					// Molecule number
					strncpy(newvalue,((char*) record) ,2);
					newvalue[2] = '\0';
					linep->_molecnum = M_ATOI(newvalue);

					// Isotopologue number
					strncpy(newvalue,((char*) record) + 2,1);
					newvalue[1] = '\0';
					linep->_isonum = M_ATOI(newvalue);

					// Vacuum wavenumber
					strncpy(newvalue,((char*) record) + 3,12);
					newvalue[12] = '\0';
					linep->_nu = M_ATOF(newvalue);

					// S
					strncpy(newvalue,((char*) record) + 15,10);
					newvalue[10] = '\0';
					linep->_S = M_ATOF(newvalue);

					// Gamma_air
					strncpy(newvalue,((char*) record) + 35,5);
					newvalue[5] = '\0';
					linep->_gamAir = M_ATOF(newvalue);

					// Gamma_self
					strncpy(newvalue,((char*) record) + 40,5);
					newvalue[5] = '\0';
					linep->_gamSelf = M_ATOF(newvalue);

					// E"
					strncpy(newvalue,((char*) record) + 45,10);
					newvalue[10] = '\0';
					linep->_Eb = M_ATOF(newvalue);

					// n_air
					strncpy(newvalue,((char*) record) + 55,4);
					newvalue[4] = '\0';
					linep->_nAir = M_ATOF(newvalue);

					// delta_air
					strncpy(newvalue,((char*) record) + 59,8);
					newvalue[8] = '\0';
					linep->_deltaAir = M_ATOF(newvalue);

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
					molid = M_ATOI( linein.substr(8).c_str() );
				} else {
					// Check for new isotope
					if (linein.size() < 11) continue;
					if (linein[11] != ' ')
					{
						// New isotope found
						isoid = M_ATOI( linein.substr(8,4).c_str() );
						abundance = M_ATOF( linein.substr(14,10).c_str() );
						//Q = M_ATOF( linein.substr(29,9).c_str() );
						//gj = M_ATOI( linein.substr(42,2).c_str() );
						//mmass = M_ATOF( linein.substr(49,8).c_str() );

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

			// Note: Original routine was too slow. Using similar process as hitran08.par load
			static const unsigned int numParsumEntries = 3000-70+1; // TODO: allow for dynamic setting
			// Each record line is 2916 characters long, including the end line
			static const unsigned int linelength = 2916;
			// Split reads into intervals of 21 lines each, so a read length of 61236 < 64k
			//unsigned int numBlockLines = 21;
			unsigned int numBlockLines = numParsumEntries;
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
			//indata >> namein; // Skip Temp(K) in file
			//getline(indata,namein);
			//parser.str(namein);
			parser >> namein;
			// Expand around whitespace
			while (parser.good())
			{
				parser >> namein;
				molecisoids.push_back(namein);
			}
			// Drop last molecisoids (it repeats the last read)
			molecisoids.pop_back();

			//Qmap.resize(molecisoids.size());
			QmapNames = molecisoids;

			unsigned int numIsos = molecisoids.size();

			// Read in the data values
			if (Qmatrix) delete[] Qmatrix;
			Qmatrix = new double[numParsumEntries*(1+numIsos )];
			char* blockin = new char[linelength*numBlockLines];
			unsigned int linesRead = 0, lineIndex = 0;
			while (lineIndex < numParsumEntries)
			{
				if (numBlockLines == 0) break;
				indata.read(blockin,linelength*numBlockLines);
				linesRead += numBlockLines;
				// If we've hit the end of file, with a partial block, change numBlockLines
				//if (numParsumEntries - linesRead < numBlockLines) numBlockLines = numParsumEntries - linesRead;
				// Iterate in parallel on the block, filling in the values
				// Q values are in 27-character-wide fields, T values are in the first six characters
				// I can just punch nulls in this white-space-filled block
				char* line;
				char* arr;
				unsigned int offstart;
#pragma omp parallel for private(line, arr, offstart)
				for (int i=0;i<(int)numBlockLines;i++)
				{
					line = &blockin[linelength*i];
					arr = line;
					// Read T
					arr[6] = '\0'; // Insert a null
					Qmatrix[(numIsos+1)*(lineIndex+i)] = M_ATOF(&arr[0]); // Start at zero and read until null, and return a double
					for (unsigned int j=0;j<numIsos;j++)
					{
						// Read the values
						if (j==0)
						{
							offstart = 7;
						} else {
							offstart = 26*(j)+1;
						}
						// Punch hole at end
						arr[26*(j+1)]='\0';
						// Pull in the double
						Qmatrix[(numIsos+1)*(lineIndex+i)+j+1] = M_ATOF(&arr[offstart]);
					}
				}
				lineIndex += numBlockLines;
			}
			QTlow = (unsigned int) Qmatrix[0];
			QnumIsos = numIsos;
			QThigh = (unsigned int) Qmatrix[(numIsos+1)*(numParsumEntries-1)];
			QnumRecords = numParsumEntries;
			indata.close();
			delete[] blockin;
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
				for (unsigned int j=0;j<QnumIsos;j++)
				{
					if (QmapNames[j] == qstr) target = j;
				}
				// Link _Q with Q[target]
				newiso->_Qcol = target;

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

		isoconc::isoconc(unsigned int molnum)
		{
			// Select the molecule number (not abundance order)
			// and import the appropriate isotopes
			std::set<isodata*>::iterator it;
			for (it = specline::linemappings.begin(); it != specline::linemappings.end(); it++)
			{
				if ((*it)->molnum() == molnum)
				{
					// Add the isotope
					isotopes.insert( (*it) );
				}
			}
		}

		isoconc::isoconc(std::string &molecule)
		{
			// See other constructor
			std::set<isodata*>::iterator it;
			for (it = specline::linemappings.begin(); it != specline::linemappings.end(); it++)
			{
				std::string a((*it)->molecule()), b(molecule);
				if (a == b)
				{
					// Add the isotope
					isotopes.insert( (*it) );
				}
			}
		}

	}; // end lbl


}; //end rtmath


