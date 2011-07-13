#include "Stdafx.h"
#include "atmos.h"
#include <vector>
#include <map>
#include "layer.h"
#include "matrixop.h"
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
		// If taus is filled, just return the sum.
		// If not, calculation is required

		// Holds the final value for tau
		double res = 0.0;
		if (_taus.size() == 0)
		{
			// for each lbl layer, compute tau
			// add the taus together to get the overall optical depth

			// The array of taus for each layer is _taus, as 
			//  they should be retained for scattering usage

#pragma omp parallel for
			for (int i=0; i<(int)lbllayers.size(); i++)
			{
				_taus[i] = lbllayers[i].tau(_wvnum);
				//std::cout << i << "\t" << pres[i] << std::endl;
			}
		}
		// Sum the partials
		for (int i=0;i<(int)_taus.size();i++)
			res += _taus[i];

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
#ifdef _WIN32
		numgases = gasnames.size(); 
#else // parser.good() repeats the last value. Aarg!
		numgases = gasnames.size() - 1;
#endif

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
			//TASSERT(zn>0); // Debug checking
			TASSERT(pn>0);
			TASSERT(tn>0);
			TASSERT(dn>0);

			zlevs.push_back(zn);
			plevs.push_back(pn);
			tlevs.push_back(tn);
			dlevs.push_back(dn);

			// Read in gases
			for (unsigned int i=0;i<numgases;i++)
			{
				in >> gn[i];
				conc[i].push_back(gn[i]);
			}

		}
		delete [] gn;
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
		lbllayers.resize(zlevs.size()-2);
		dalayers.resize(zlevs.size()-2);

		lbl::lbllayer *layer;
		int j;
		// Loop is to zlevs.size()-2 to avoid the second last read that in.good() loops
		// always have and to avoid calculating dz at the end of the atmosphere
#pragma omp parallel for private(layer,j)
		for(int i=0;i<(int)zlevs.size()-2;i++)
		{
			// Fill in the information for this layer
			// TODO: add dalayer stuff
			layer = &lbllayers[i];
			
			layer->dz(1000 * (zlevs[i+1]-zlevs[i]) ); // in m
			layer->p(plevs[i]); // in hPa
			layer->T(tlevs[i]); // in K
			TASSERT(layer->dz() > 0);
			
			// Loop through and add the necessary isotope concentrations
			//layer->isoconcentrations
			for (j=0;j<(int)numgases;j++)
			{
				// Create isoconc based on gasnames[j]
				lbl::isoconc *newgas= new lbl::isoconc(gasnames[j]);
				// newgas has dz, p and T set from the layer when it runs
				// TODO: change the structure so that pointers to the
				//  values are used. Prevents update errors.
				newgas->dz(layer->_dz); // in m
				newgas->p(layer->_p);   // in hPa
				newgas->T(layer->_T);   // in K
				// need to calculate ps
				// ps is the partial pressure in atmospheres of the gas
				// so, the sum of all ps is one
				// but not all gases accounted for here, so < 1
				
				// I have P(mb or atm), density(cm^-3), and conc. (ppmv)
				// This is easy. Ps = conc * e-6 * P
				newgas->psfrac(1.0E-6 * conc[j].at(i)); // Just a fraction (unitless)
				layer->isoconcentrations.insert(newgas);
			}
			// Gases inserted
		}
		// Layers are now fully constructed
		// Function routine is done
	}

	void atmos::_calcProps()
	{
		throw rtmath::debug::xUnimplementedFunction();
		// Have each layer generate its base properties
		// Then, add downward, then upward to get appropriate R and T
		// Next, calculate intensities and fluxes
		// Convert to get brightness temperatures
		using namespace rtmath;
		using namespace std;
		//vector<dalayer>::const_iterator it; // The iterator for the layer
		// Assume that the layers are already generated
		// Assume that the optical depths are already known (in _taus)

		// Use indexes because there is a 1:1:1 correspondance between 
		//  dayalers, lbllayers and _taus
		for (int i=0; i<(int)dalayers.size(); i++)
		{
			dalayers[i].tau(_taus[i]);
			//dalayers[i].generateLayer();
		}
	}

	void atmos::RTd(size_t low, size_t high, 
		boost::shared_ptr<damatrix> &Rres, boost::shared_ptr<damatrix> &Tres)
	{
		throw rtmath::debug::xUnimplementedFunction(); // Lots of debugging needed
		_calcProps(); // generate R and T for each layer
		// Add downwards from high to low
		// Use the standard adding method equations
		boost::shared_ptr<damatrix> startR = dalayers[high].getR();
		boost::shared_ptr<damatrix> startT = dalayers[high].getT();
		size_t it = high - 1;
		boost::shared_ptr<damatrix> Ra = startR;
		boost::shared_ptr<damatrix> Ta = startT;
		while (it >= low)
		{
			boost::shared_ptr<damatrix> Rb = dalayers[it].getR();
			boost::shared_ptr<damatrix> Tb = dalayers[it].getT();
			//boost::shared_ptr<damatrix> Rr = *Ra + (*Ta * *Rb * (1.0 - *Ra * *Rb).inverse() * Ta);
			//boost::shared_ptr<damatrix> Tr = *Tb * (1.0 - *Ra * *Rb).inverse() * *Ta;
			//Ra = Rr;
			//Ta = Tr;
			it--;
		}
		Rres = Ra;
		Tres = Ta;
	}

};

