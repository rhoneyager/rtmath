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
#include "macros.h"
#include "debug.h"

#include "debug_mem.h"


namespace rtmath {
	namespace lbl {
		// Define the static variables here
		double specline::_pRef = 1013.0; // in hPa
		double specline::_TRef = 296.0; // in K
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
		/*
		const char* specline::__file__ = 0;
	size_t specline::__line__ = 0;
	const char* specline::__caller__ = 0;
	*/
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
				
				/* // The old-school method that was way too slow
				for(line = (*it)->lines.begin(); line != (*it)->lines.end(); line++)
				{
					res += (*line)->deltaTau(nu, *_p, *_p * _psfrac, *_T, abun, (*it)->_Qcol, *_dz);
				}
				*/
				// This is going to be one long function, since inlining makes my header too big
				// Plus, I can optimize and parallelize to my heart's content

				// The function is at the molecular level
				// So, each iterator is at the isotope level
				double abun = (*it)->abundance();
				unsigned int Qc = (*it)->_Qcol;
				// Each isotope (isodata) has a set of specdata* lines
				// Loop through them
#pragma omp parallel for
				for (int i=0;i<(int) (*it)->numLines();i++)
				{
					// Each line is at (*it)->lines[i]
					specline *line = &(*it)->lines[i];
					// Each line has delta tau: dt=n_a,i * knn'(nu,T,p)dz
					// Also need the isotope's Q
					TASSERT(line->_S <1e-10); // Debugging invalid line check
					TASSERT(line->_S > 0);

					static const double kb=1.3806503e-23; // Boltzmann's const in m^2 kg s^-2 K^-1
					// Convert ps from atm to Pa for units
					//  Remember that _p, _T are pointers to the values of the layer
					double ps = _psfrac * (*_p);						// in hPa
					// nai is the number density for isotope I (units of molecule/m^3)
					//   abun is the isotope abundance fraction (fraction of this iso over all isos of this molecule)
					//   abun is dimensionless. ps is in hPa. kb is above. T in K
					// So, nai has units of 100 / m^3 (much better than in atmospheres, with the 101325 factor)
					double nai = abun * ps / (kb * (*_T));				// in 100 molecules / m^3
					// To get dTnn, we must calculate k

					// But, k is dependent on several other things! 
					
					// Calculate gamma(p,ps,T)
					// _nAir is provided by iteration over spectral line
					// Tr/T is unitless. each gamma is HWHM at 296 K in cm^-1 atm^-1
					// p, ps in hPA, so gamma needs conversion
					const double hPadivAtm = 0.00098692327; // hPa / atm conversion factor (= 1/1013...)
					double gamma = pow(specline::TRef() / (*_T),line->_nAir) * 
						((line->_gamAir * hPadivAtm * (*_p - ps)) + (line->_gamSelf * hPadivAtm * ps) );
					// so, gamma has units of cm^-1

					// Calculate nushifted(p)
					// spectral line-dependent
					// _deltaAir has units of 1 / (cm * atm)
					// _nu is in cm^-1
					// I want to keep nu in wavenumber units (cm^-1), so the formula is:
					double nushifted = line->_nu + (line->_deltaAir * (*_p) * hPadivAtm); // nushifted is in cm^-1

					// Calculate f
					// Assume Lorentzian for now
					// TODO: add selector to enable choice
					// spectral line dependent
					double f;	// f = gamma/(gamma^2 + (nu-nushift)^2)
								// f has units of (1/cm)/((1/cm)^2 + (cm^-1)^2)
								// f has units of 0.01 * m = cm
					{
						double num = gamma;
						double denom = (gamma * gamma) + ((nu - nushifted) * (nu - nushifted));
						f = num / (M_PI * denom);	// f is in cm
					}

					// Calculate S(T,Q)
					// S is spectral line dependent
					double S;									// S is in cm / molecule
					{
						// Average partition function to get Q(T)
						double Qref, Q, Qa, Qb;
						Qref = specline::Qmatrix[(specline::QnumIsos+1)*(296-specline::QTlow) + Qc + 1];
						Qa = specline::Qmatrix[(specline::QnumIsos+1)*( ((unsigned int)(*_T))-specline::QTlow) + Qc + 1];
						Qb = specline::Qmatrix[(specline::QnumIsos+1)*( ((unsigned int)(*_T))+1-specline::QTlow) + Qc + 1];
						// If Tfrac = 1, then Qb, if Tfrac = 0, then Qa
						double Tfrac = (*_T) - (double) ((unsigned int) *_T); 
						Q = (Tfrac * Qb) + ( (1.0 - Tfrac) * Qa);
						double Qquo = Qref / Q;
						double cb = -1.4388; // cm*K
						double Equo = exp( cb*line->_Eb/(*_T) ) / exp(cb*line->_Eb/specline::TRef());
						double nuquo = (1.0 - exp( cb*line->_nu/(*_T) )) / 
							(1.0 - exp(cb*line->_nu/specline::TRef() ));
						// Units of _S are cm / molecule at 296 K
						// Qquo is unitless
						// Equo is unitless
						// nuquo is unitless
						S = line->_S * Qquo * Equo * nuquo;		// Units of S are cm / molecule
					}

					// Calculate k
					// k = S(T,Q) * f(nu,p,ps,T)
					// k is spectral line dependent
					// S is in cm / molecule. f is in cm
					// k has units of cm^2 / molecule
					double k = S * f;

					// Finally, dtnn may be calculated
					// nai has units of 100 / m^3
					// k is in cm^2 / molecule
					// dz is in m
					// dtnn has units of 0.01 (dimensionless)
					//     so multiply by 100 to get truly dimensionless units
					double dtnn = 100 * nai * k * (*_dz); // dimensionless
					TASSERT(dtnn>=0);
#pragma omp atomic
					res += dtnn;
				}
			}
			return res; // res is dimensionless, as expected
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
			//__Track(3,0,0,0,0,0);
		}

		void specline::_loadHITRAN(const char* hitranpar)
		{
			// This algorithm is too slow at first
			// Preallocate memory to speed it up
			//const unsigned int numrecs = 2713968; // Now a static var
			specline *inlines = new specline[numrecs];
			specline::lines = inlines;

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
#ifdef _WIN32
			// Size is 161 because of line feed character
			const unsigned int recSize = 161;
#else
			// Size is 162 because non-CRLF systems read CR and LF separately, 
			// whereas windows reads it as a single newline
			const unsigned int recSize = 162;
#endif

			// Calc the critical read number (for special treatment)
			unsigned int numReads = numrecs/nRecsinread;
			if (numrecs % nRecsinread) numReads++;

			//const unsigned int recRem = numrecs - numFullReads * nRecsinread; // Not used

			// Open the hitran file
			using namespace std;

			ifstream indata(hitranpar);
			if (indata.good() == false) throw rtmath::debug::xEmptyInputFile(hitranpar);
			if (indata.eof()) throw rtmath::debug::xEmptyInputFile(hitranpar);
			if (indata.bad()) throw rtmath::debug::xEmptyInputFile(hitranpar);
			if (indata.fail()) throw rtmath::debug::xEmptyInputFile(hitranpar);

			// The line containing the data
			char inset[recSize*nRecsinread];
			char *record = inset;
			//char newvalue[20];
			specline *linep = inlines;

			// Do block-by-block to enable parallelization
			for (int block=0; block < (int) numReads; block++)
			{
				// Read in the block
				indata.read(inset,recSize*nRecsinread);
				// Set the bounds for the iteration
				int blockstart = block * nRecsinread;
				int blockend = (block + 1) * nRecsinread;
				if (blockend > (int) numrecs) blockend = numrecs; // not -1 since i<blockend

				// Loop through the record ids
#pragma omp parallel for private(record,linep)
				for (int i=blockstart;i<blockend;i++)
				{
					linep = &inlines[i];
					int indataLine = i % nRecsinread;
					record = &inset[recSize*indataLine];
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
                                        
                                        TASSERT(linep->_molecnum >= 0);
                                        
				}
			}

#ifdef _DEBUG
			// Verify that the lines are populated
			for (int i=0; i<(int)numrecs; i++)
			{
				if (inlines[i]._molecnum < 0) throw;
				if (inlines[i]._molecnum > 60) throw;
			}
			std::cerr << "DEBUG: HITRAN08.par read lines verified\n";
#endif

			indata.close();
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
			unsigned int molid = 0;
			unsigned int isoid = 0;
			double abundance = 0;
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
						abundance = M_ATOF( linein.substr(14,11).c_str() );
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
#ifdef _WIN32
			static const unsigned int linelength = 2916;
#else
			static const unsigned int linelength = 2917;
#endif
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
			//if (Qmatrix) delete[] Qmatrix;
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
							offstart = 27*(j)+1;
						}
						// Punch hole at end, only if not at end of array
						if (j != numIsos-1) arr[27*(j+1)]='\0';

						//double test = M_ATOF(&arr[offstart]); // for debugging

						// Pull in the double
						// M_ATOF, upon reading an endline, stops parsing and returns, so we're safe here
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

			// Perform parsum verification
#ifdef _DEBUG
			int imax = (int) numParsumEntries*(1+numIsos );
			double Qv = 0;
			for (int i=0; i< imax; i++)
			{
				Qv = Qmatrix[i];
				if (Qv < 0) throw;
				if (Qv > 9.5e16) throw;
			}
			std::cerr << "DEBUG: parsum verified\n";
#endif
		}

		void specline::_doMappings()
		{
			// Function creates the mappings between molparams, parsum and the lines.
			// This is necessary because each file uses different identifiers for the data.
			// This function assumes that molparam has the complete set of interesting 
			// molecules (it is essential for the others to work).
			// It finds the matching parsum entry, and selects the appropriate lines 

			// Each isodata entry contains the isotope data for one isotope of a molecule
			// It contains the mapping of partition function values, the abundance, and 
			// the set of spectral lines. Pointers here are essential.

			// Two-pass solution for greater speed

			// Create bins for abundances
			unsigned int numIsos = abundanceMap.size();
			unsigned int *nALines = new unsigned int[numIsos];
			isodata *newIsos = new isodata[numIsos]; // Have this data persist (used in set linemappings)


			// Loop through isotopes and make a mapping table to isotope / hitran line identifiers
			int pmolec = 0, piso = 0;
//////#pragma omp parallel for private(pmolec,piso) // can't use a parallel loop because I need ordered data...
			for (int i=0;i<(int)numIsos;i++)
			{
				// Set the basic parameters
				nALines[i] = 0;

				// Here, populate the newIsos data with molecule information
				//  Eveything goes in except for the actual lines
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
				newIsos[i]._abundance = abundanceMap[i].abundance();
				abundanceMap[i].molecname(newIsos[i]._molecule);
				newIsos[i]._isotope = abundanceMap[i].isonum();
				newIsos[i]._molnum = abundanceMap[i].molecnum();
				newIsos[i]._isoorder = piso; // Set order for line lookup

				// Select the appropriate Q parameter set
				// No need to speed up, as this only done ~110 times
				std::ostringstream qsstr; // I'm combining an int with a string
				std::string qstr;
				qsstr << newIsos[i]._molecule << "_" << newIsos[i]._isotope;
				qstr = qsstr.str(); // Make stringstream into string
				// Search for the entry
				unsigned int target = 0;
				bool success = false;
				for (unsigned int j=0;j<QnumIsos;j++)
				{
					if (QmapNames[j] == qstr) 
					{
						target = j;
						success = true;
					}
					if (success) break;
				}
				// Link _Q with Q[target]
				newIsos[i]._Qcol = target;
				if (!success) 
				{
					newIsos[i].valid = false;
				} else {
					// Add isotope to specdata::linemappings. It's essential for later parts
					// isodata *iso = &newIsos[i]; // useful when debugging in VS
//#pragma omp critical // see above - loop requires ordered data, so no openmp
					specline::linemappings.insert(&newIsos[i]);
				}
			}
			
			// Do pass one of the lines
			// In this pass, just count the number of lines for each isotope
			
#pragma omp parallel for
			for (int k=0;k<(int)numrecs;k++)
			{
				specline *currline = &lines[k];
                                // Debug: lots of assertions slow the code, but find the bugs
                                TASSERT(currline->_isonum >= 0);
                                TASSERT(currline->_S < 1.e20);
                                TASSERT(currline->_S >= 0);
                                TASSERT(currline->_molecnum >= 0);
				bool done = false;
				// Iterate over each isotope
				for (int i=0;i< (int) numIsos;i++)
				{
					isodata *currIso = &newIsos[i];
					if (newIsos[i].valid == false) continue; // Skip it
					if (lines[k]._molecnum == newIsos[i]._molnum)
					{
						if (lines[k]._isonum == newIsos[i]._isoorder)
						{
#pragma omp atomic
							nALines[i]++;
							//newIsos[i].lines.insert(&lines[k]);
							done = true;
						}
					}
					if (done) break;
				}
			}
			
			// Pass 2 - Allocate memory for the line arrays and insert the lines
#pragma omp parallel for
			for (int i=0;i<(int)numIsos;i++)
			{
				if (newIsos[i].valid == false) continue; // skip it
				newIsos[i].lines = new specline[ nALines[i] ];
				newIsos[i]._numLines = nALines[i];
			}
			///*
#pragma omp parallel for
			for (int k=0;k<(int)numrecs;k++)
			{
				specline *tline = &lines[k];
				bool done = false;

				TASSERT(tline->_molecnum >= 0); // A throwable assertion for bug checking

				// Iterate over each isotope
				for (int i=0;i< (int) numIsos;i++)
				{
					if (newIsos[i].valid == false) continue; // skip it
					if (lines[k]._molecnum == newIsos[i]._molnum)
					{
						if (lines[k]._isonum == newIsos[i]._isoorder)
						{
							// Omitting the -1 brought pain and heartbreak.
							// It took two hours to find the bug.
							// Windows App Verifier and gflags and many linux heap overflow
							// debug utilities did NOT find it.
							// Also, the next TWO lines are critical TOGETHER. Else, a run condition occurs and
							// not all values are filled.
#pragma omp critical
							{
								(newIsos[i].lines[nALines[i] - 1]) = *tline;
								nALines[i]--;
							}
							done = true;
						}
					}
					if (done) break;
				}
			}
			//*/
			// We've looped through all isotopes, so the mappings are complete.
			// Free nALines
			delete[] nALines;
		}

		isoconc::isoconc(int molnum)
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


