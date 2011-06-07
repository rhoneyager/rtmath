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

#include "lbl.h"

namespace rtmath {
	namespace lbl {
		// Define the static variables here
		double specline::_pRef = 1.0;
		//std::set<specline*> specline::lines;
		//std::map<Qselector, double> specline::Qmap;
		std::vector< std::map<double, double> > specline::Qmap;
		std::vector< std::string> specline::QmapNames;
		std::vector<isoselector> specline::abundanceMap;

		void specline::loadlines(const char* hitranpar, 
			const char* molparam, const char* parsum)
		{
			// If a string is null, skip that step
			if (hitranpar[0]) _loadHITRAN(hitranpar);
			if (molparam[0]) _loadMolparam(molparam);
			if (parsum[0]) _loadParsum(parsum);
		}

		void specline::_loadHITRAN(const char* hitranpar)
		{
			// This algorithm is too slow at first
			// Preallocate memory to speed it up
			const unsigned int numrecs = 2713968;
			specline *inlines = new specline[numrecs];
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
					molname = linein.substr(0,5);
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
						abundanceMap.push_back(isoselector(molid,isoid,abundance));
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
			// Expand around whitespace
			while (parser.good())
			{
				indata >> namein;
				molecisoids.push_back(namein);
			}

			Qmap.resize(molecisoids.size());
			QmapNames = molecisoids;

			// Expand Qmap names, so that molec/isotop lookup works
			// Using find and substr
			for (unsigned int i=0;i<QmapNames.size();i++)
			{
				size_t seploc = QmapNames[i].find('_');
				string mname = QmapNames[i].substr(0,seploc);
				unsigned int isotop = atoi( QmapNames[i].substr(seploc+1));

			}

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

	}; // end lbl


}; //end rtmath


