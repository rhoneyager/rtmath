#include "Stdafx.h"
#include "atmos.h"
#include <vector>
#include <map>
#include "layer.h"
#include "../rtmath-base/matrixop.h"
#include "damatrix.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include "lbl.h"

#include "debug_mem.h"

namespace rtmath {

	atmos::atmos()
	{
	}

	atmos::~atmos()
	{
	}

	double atmos::tau()
	{
		// for each lbl layer, compute tau
		// add the taus together to get the overall optical depth
		double res = 0.0;
		double newtau = 0.0;
		for (unsigned int i=0; i<lbllayers.size(); i++)
		{
			newtau = lbllayers[i].tau(_wvnum);
			std::cout << i << "\t" << newtau << std::endl;
			res += newtau;
		}

		return res;
	}

	void atmos::loadProfile(const char* filename)
	{
		//throw;
		// open the file
		using namespace std;
		ifstream in(filename);

		// These variables are in the file
		std::vector<double> zlevs, plevs, tlevs, dlevs;
		unsigned int numgases;
		// For gases, the unsigned int is the gas id, the double is 
		// the concentration in ppmv - matches input file
		std::vector< std::vector<double> > conc;
		std::vector<string> gasnames;

		string buffer;
		istringstream parser(istringstream::in);
		getline(in,buffer); // Read a full line
		parser.str(buffer);
		parser >> buffer; // Discard word 'Profile'
		parser >> buffer; // The name of the profile
		name = buffer;

		getline(in,buffer); // Get next line, which lists the var names
		parser.str(buffer); // I'm doing this to avoid passing line boundaries
		while (parser.good())
		{
			parser >> buffer; 
			if (buffer == "Altitude") continue;
			if (buffer == "Pres") continue;
			if (buffer == "Temp") continue;
			if (buffer == "Density") continue;
			// Otherwise, add gas to conc. list
			gasnames.push_back(buffer);
		}
		numgases = gasnames.size();

		// Skip next line
		getline(in,buffer);

		// Read in the profile values
		double zn, pn, tn, dn;
		double *gn = new double[numgases];
		conc.resize(numgases);

		while(in.good())
		{
			//getline(in,buffer);
			//parser.str(buffer);
			// Do formatted read
			//cerr << parser.str();
			in >> zn;
			in >> pn;
			in >> tn;
			in >> dn;
			zlevs.push_back(zn);
			plevs.push_back(pn / 1013.0);
			tlevs.push_back(tn);
			dlevs.push_back(dn);

			// Read in gases
			for (unsigned int i=0;i<numgases;i++)
			{
				in >> gn[i];
				conc[i].push_back(gn[i]);
			}

		}
		delete [] gn; // Note: I'm introducing a memory leak
		in.close();

		// Take the line data from hitran and select the useful lines
		// These lines will be a subset that corresponds to the 
		// measured gas concentrations that are mentioned in the profile

		

		// I've loaded the profile information, 
		// but now, what to do with it?

		// TODO: make more complex code to handle scattering cases too
		// For now, I'll assume that values are constant within a layer
		// and set the number of atmos layers equal to the number 
		// that were listed in the profile

		// TODO: add scattering case

		// Iterate from zero to the number of layers
		// Resize beforehand so I can use multiple threads here
		lbllayers.resize(zlevs.size());
		dalayers.resize(zlevs.size());

		lbl::lbllayer *layer;
		int j;
#pragma omp parallel for private(layer,j)
		for(int i=0;i<(int)zlevs.size()-1;i++)
		{
			// Fill in the information for this layer
			// TODO: add dalayer stuff
			layer = &lbllayers[i];
			// TODO: unit checks
			layer->dz(zlevs[i+1]-zlevs[i]);
			layer->p(plevs[i]);
			layer->T(tlevs[i]);
			
			// Loop through and add the necessary isotope concentrations
			//layer->isoconcentrations
			for (j=0;j<(int)numgases;j++)
			{
				// Create isoconc based on gasnames[j]
				lbl::isoconc *newgas= new lbl::isoconc(gasnames[j]);
				// newgas has dz, p and T set from the layer when it runs
				// TODO: change the structure so that pointers to the
				//  values are used. Prevents update errors.
				newgas->dz(layer->_dz);
				newgas->p(layer->_p);
				newgas->T(layer->_T);
				// need to calculate ps
				// ps is the partial pressure in atmospheres of the gas
				// so, the sum of all ps is one
				// but not all gases accounted for here, so < 1
				
				// I have P(mb or atm), density(cm^-3), and conc. (ppmv)
				// This is easy. Ps = conc * e-6 * P
				newgas->psfrac(1.0E-6 * conc[j].at(i));
				layer->isoconcentrations.insert(newgas);
			}
			// Gases inserted
		}
		// Layers are now fully constructed
		// Function routine is done
	}

	void atmos::_calcProps()
	{
		throw;
		// Have each layer generate its base properties
		// Then, add downward, then upward to get appropriate R and T
		// Next, calculate intensities and fluxes
		// Convert to get brightness temperatures
	}


};

