#include "../rtmath/Stdafx.h"
#include <memory>
#include <vector>
#include <map>
#include <iostream>
#include <fstream>
#include <sstream>
#include <boost/filesystem.hpp>
#include <boost/tokenizer.hpp>
#include "../rtmath/atmos.h"
#include "../rtmath/absorb.h"
#include "../rtmath/da/daDiagonalMatrix.h"
#include "../rtmath/error/error.h"

namespace rtmath {
	
	namespace atmos {

		atmos::atmos()
		{
		}

		atmos::~atmos()
		{
		}

		atmos::atmos(const std::string &filename)
		{
			loadProfile(filename);
		}

		double atmos::tau(double wvnum) const
		{
			return tau(wvnum,0,_layers.size());
		}

		double atmos::tau(double wvnum, size_t layernum) const
		{
			return tau(wvnum,layernum,layernum);
		}

		double atmos::tau(double wvnum, size_t layerLow, size_t layerHigh) const
		{
			throw rtmath::debug::xUnimplementedFunction();
			double res = 0;
			for (size_t i=layerLow; i<layerHigh; i++)
			{
				res += _layers[i].tau(wvnum);
			}
			return res;
		}

		void atmos::loadProfile(const std::string &filename)
		{
			loadProfileRyan(filename);
		}

		void atmos::loadProfileLiu(const std::string &filename)
		{
			// Liu's atmosphere structure is very dissimilar from my own
			// For now, all I'm doing is loading in the atmosphere, not 
			// the hydrology, which exists within a separate, unlinked file
			using namespace std;
			using namespace rtmath;
			using namespace boost::filesystem;
			// Verify file existence
			path p(filename);
			if (!exists(p)) throw debug::xMissingFile(filename.c_str());
			if (is_directory(p)) throw debug::xMissingFile(filename.c_str());
			// Open the file
			ifstream in(filename.c_str());
			if (!in) throw debug::xOtherError();
			if (!in.good()) throw debug::xEmptyInputFile(filename.c_str());

			// Liu's file structure gives the number of levels on the first line
			// After this, each line contains: the height in m, pressure (mb),
			// temperature (K) and the relative humidity (percent), starting with
			// the surface

			// For the hydrometeor profile, each record contains the cloud liquid,
			// cloud ice, rain, snow, graupel, starting from the lowest layer.
			// Units for cloud liquid and cloud ice are in g/m^3. Rain, snow and 
			// graupel are in g/m^3 or in mm/hr.
			// Therefore, the hydrometeor file is almost totally useless for my work

			size_t numLayers = 0;
			in >> numLayers;

			// TODO: fix layer toa issues
			_layers.resize(numLayers-1); // -1 to avoid dz at toa?
			// Loop through each layer and get information
			// As each layer is loaded, generate the appropriate atmoslayer and link
			// TODO: split logic between loading and implementing, as 
			// I need to calculate dz upon layer setup. Aargh.
			for (size_t i=0; i<numLayers-1;i++)
			{
				double h, p, t, rh;
				// Also, since I'm missing several gases here, I need to add
				// them by hand (O2, N2)
				in >> h >> p >> t >> rh; // Quick and easy input

				// Convert the relative humidity to a water vapor concentration
				// in ppmv? or a partial pressure?

				atmoslayer *layer = &_layers[i];
				layer->dz(); // dz needs layer above for setting
				layer->p(p);
				later->T(t);
			}
		}

		void atmos::loadProfileRyan(const std::string &filename)
		{
			// TODO: fix so that layers may start from toa or from ground
			using namespace std;
			using namespace rtmath;
			using namespace boost::filesystem;
			// Verify file existence
			path p(filename);
			if (!exists(p)) throw debug::xMissingFile(filename.c_str());
			if (is_directory(p)) throw debug::xMissingFile(filename.c_str());
			// Open the file
			ifstream in(filename.c_str());
			if (!in) throw debug::xOtherError();
			if (!in.good()) throw debug::xEmptyInputFile(filename.c_str());

			// Start reading
			vector<double> zlevs, plevs, tlevs, dlevs;
			unsigned int numgases;
			// For gases, the unsigned int is the gas id, the double is 
			// the concentration in ppmv - matches input file
			vector< vector<double> > conc;
			vector<string> gasnames;

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
				in >> zn;
				in >> pn;
				in >> tn;
				in >> dn;
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

			// I've read the file
			// Onto the processing!
			TASSERT(zlevs.size() > 2);
			// Resize beforehand for multithreading
			_layers.resize(zlevs.size()-2);

			atmoslayer *layer;
			int j;
			// Loop is to zlevs.size()-2 to avoid the second last read that in.good()
			// loops always seem to have and to avoid calculating dz at the end of 
			// the atmosphere
#pragma omp parallel for private(layer,j)
			for(int i=0;i<(int)zlevs.size()-2;i++)
			{
				// Fill in the information for this layer
				layer = &_layers[i];
				layer->dz( zlevs[i+1]-zlevs[i]); // in km
				layer->p(plevs[i]); // in hPa
				layer->T(tlevs[i]); // in K
				TASSERT(layer->dz() > 0);

				// Loop through and add the necessary gases to the layer
				for (j=0;j<(int)numgases;j++)
				{
					// Create absorber based on gas name in gasnames[j]
					absorber *newgas;
					if ("H2O" == gasnames[j]) newgas = new abs_H2O;
					else if ("O2" == gasnames[j]) newgas = new abs_O2;
					else if ("N2" == gasnames[j]) newgas = new abs_N2;
					else continue; // Skip if not found
					// I'll associate this with a layer, and I need to calculate psfrac
					// ps is the partial pressure of the gas in atmospheres (TODO: or mb?)
					// so, they all sum to one, but in reality <1 as I don't have all the gases
					// I have P(mb or atm), density(cm^-3) and conc. (ppmv)
					// This is easy. Ps = conc * e-6 * P
					newgas->setLayer(*layer, 1.0e-6*conc[j].at(i)); // Just a fraction (unitless)
					
					// Insert the gas into the layer
					// encapsulate into a pointer. Object already created, so no new needed
					std::shared_ptr<absorber> ptr(newgas); 
					layer->absorbers.insert(ptr);
				}
				// Add collision-induced broadening
				absorber *newgas = new collide;
				newgas->setLayer(*layer,0);
				std::shared_ptr<absorber> ptr(newgas); 
				layer->absorbers.insert(ptr);
			}
			// And, we're done!!!!!
		}

		void atmos::saveProfile(const std::string &filename) const
		{
			std::cerr << "saveProfile not yet implemented, so " << filename << 
				" will not be written.\n";
			throw rtmath::debug::xUnimplementedFunction();
		}

	}; // end namespace atmos

}; // end namespace rtmath

