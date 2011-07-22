#include "Stdafx.h"
#include <memory>
#include <vector>
#include <map>
#include <iostream>
#include <fstream>
#include <sstream>
#include "atmos.h"
#include "daLayer.h"
#include "matrixop.h"
#include "damatrix.h"
#include "lbl.h"
#include "daDiagonalMatrix.h"

namespace rtmath {
	atmos::atmos()
	{
		_wvnum = 0;
	}

	atmos::~atmos()
	{
	}

	double atmos::tau()
	{
		return tau(0,(unsigned int) lblLayers.size());
	}

	double atmos::tau(unsigned int layernum)
	{
		// Ask the layer directly
		return tau(layernum, layernum);
	}

	double atmos::tau(unsigned int layerlow, unsigned int layerhigh)
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
			for (int i=0; i<(int)lblLayers.size(); i++)
			{
				_taus[i] = lblLayers[i].tau(_wvnum);
				//std::cout << i << "\t" << pres[i] << std::endl;
			}
		}
		// Sum the partials
		for (int i=layerlow;i<(int)layerhigh;i++)
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
		numgases = (unsigned int) gasnames.size(); 
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
		lblLayers.resize(zlevs.size()-2);
		daLayers.resize(zlevs.size()-2);

		lbl::lbllayer *layer;
		int j;
		// Loop is to zlevs.size()-2 to avoid the second last read that in.good() loops
		// always have and to avoid calculating dz at the end of the atmosphere
#pragma omp parallel for private(layer,j)
		for(int i=0;i<(int)zlevs.size()-2;i++)
		{
			// Fill in the information for this layer
			// TODO: add dalayer stuff
			layer = &lblLayers[i];
			
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

	std::shared_ptr<damatrix> atmos::R()
	{
		// TODO: check if this notation is correct
		if (!_R ) _calcProps();
		return _R;
	}

	std::shared_ptr<damatrix> atmos::T()
	{
		// TODO: check if this notation is correct
		if (!_T ) _calcProps();
		return _T;
	}

	void atmos::_calcProps()
	{
		using namespace std;
		// Have each layer calculate its R and T matrix
		// Then, add the layers the same way as in daLayer

		// Do a check and calculate taus, populating _taus if empty
		tau();

		for (int i=0; i< (int) daLayers.size(); i++)
		{
			// daLayer tau is set at initialization
			// If it does not match lblLayer tau, then we it needs to be recalculated
			if (daLayers[i]->tau() != _taus[i])
				daLayers[i]->tau(_taus[i]); // Reset tau for daLayer
		}

		// Define the placeholders for the overall reflectivity and transmissivity
		std::shared_ptr<damatrix> Rn, Tn; // Current R and T (new layer)
		std::shared_ptr<damatrix> Ro, To; // Current R and T (old combined)
		
		std::shared_ptr<damatrix> D;	// Internal Downwelling
		std::shared_ptr<damatrix> U;	// Internal Upwelling
		std::shared_ptr<damatrix> S;	// Used to get D
		std::shared_ptr<damatrix> Qi;	// Calculate Q1
		std::shared_ptr<damatrix> Qc;	// Running multiplication to get Q^n 
		std::shared_ptr<damatrix> RA;	// Added layer reflectance
		std::shared_ptr<damatrix> TA;	// Added layer transmittance

		Ro = daLayers[0]->R;
		To = daLayers[0]->T;
		double tauBot = daLayers[0]->tau();

		for (int i=1; i< (int) daLayers.size(); i++)
		{
			Rn = daLayers[i]->R;
			Tn = daLayers[i]->T;

			// First, calculate Q1
			Qi = damatrix::op(Rn,Ro,MULT);
			S = Qi;
			Qc = Qi;
			// Then, recursively multiply to get Qn for a large n, say 10
			for (unsigned int j=0;j<10;j++)
			{
				// I'm doing this to get away with not implementing powers in my matrices
				Qc = damatrix::op(Qc,Qi,MULT);
				S = damatrix::op(S,Qc,ADD);
			}
			
			double tauTop = daLayers[i]->tau();

			// S has been calculated. Now to find D, U, RcD and TcD
			// Constrict the diagonal e^-tau/mu and mu0 matrices for this doubling
			std::shared_ptr<daDiagonalMatrix> dMu(new daDiagonalMatrix(tauTop, valmap_selector::MU));
			std::shared_ptr<daDiagonalMatrix> dMut(new daDiagonalMatrix(tauBot, valmap_selector::MU));
			std::shared_ptr<daDiagonalMatrix> dMun(new daDiagonalMatrix(tauTop, valmap_selector::MUN));
			std::shared_ptr<damatrix> dMuD = static_pointer_cast<damatrix>(dMu);
			std::shared_ptr<damatrix> dMutD = static_pointer_cast<damatrix>(dMut);
			std::shared_ptr<damatrix> dMunD = static_pointer_cast<damatrix>(dMun);

			std::shared_ptr<damatrix> D_a = damatrix::op(S,Tn,MULT);
			std::shared_ptr<damatrix> D_b = damatrix::op(S,dMunD,MULTNORMAL);
			std::shared_ptr<damatrix> D_c = damatrix::op(D_a,Tn,ADD);
			D = damatrix::op(D_c,D_b,ADD);
			// D has been found. This notation unfortunately cannot be simplified.

			std::shared_ptr<damatrix> U_a = damatrix::op(Ro,D,MULT);
			std::shared_ptr<damatrix> U_b = damatrix::op(Ro,dMunD,MULTNORMAL);
			U = damatrix::op(U_a,U_b,ADD);
			// U has been found.

			std::shared_ptr<damatrix> R_a = damatrix::op(dMuD,U,MULTNORMAL);
			std::shared_ptr<damatrix> R_b = damatrix::op(Tn,U,MULT);
			std::shared_ptr<damatrix> R_c = damatrix::op(R_a,R_b,ADD);
			RA = damatrix::op(Rn,R_c,ADD);
			// RA (the added layer's reflectance) has been found

			std::shared_ptr<damatrix> T_a = damatrix::op(To,D,MULT);
			std::shared_ptr<damatrix> T_b = damatrix::op(To,dMunD,MULTNORMAL);
			std::shared_ptr<damatrix> T_c = damatrix::op(dMutD,D,MULTNORMAL);
			std::shared_ptr<damatrix> T_d = damatrix::op(T_a,T_b,ADD);
			TA = damatrix::op(T_d,T_c,ADD);
			// TA (the added layer's diffuse transmission) has been found


			// Prepare for the next iteration of the loop
			To = TA;
			Ro = RA;
			tauBot += tauTop;
		}


		// Set the final transmission and reflection results
		_T = TA;
		_R = RA;
	}

}; // end namespace rtmath

