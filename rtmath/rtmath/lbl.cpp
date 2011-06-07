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
		std::map<Qselector, double> specline::Qmap;
		std::map<isoselector, double> specline::abundanceMap;

		void specline::loadlines(const char* hitranpar, 
			const char* molparam, const char* parsum)
		{
			_loadHITRAN(hitranpar);
			_loadMolparam(molparam);
			_loadParsum(parsum);
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
			/*
			using namespace std;
			ifstream indata(molparam);

			indata.close();
			*/
		}

		void specline::_loadParsum(const char* parsum)
		{
		}

	}; // end lbl


}; //end rtmath


