#include "../rtmath/Stdafx.h"
#include <memory>
#include <vector>
#include <map>
#include <iostream>
#include <fstream>
#include <sstream>
#include <locale>
#include <boost/filesystem.hpp>
#include <boost/tokenizer.hpp>
#include <boost/units/systems/si.hpp> // Used for atmos readins!
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

		double atmos::tau(double f) const
		{
			return tau(f,0,_layers.size());
		}

		double atmos::tau(double f, size_t layernum) const
		{
			return tau(f,layernum,layernum);
		}

		double atmos::tau(double f, size_t layerLow, size_t layerHigh) const
		{
			double res = 0;
			for (size_t i=layerLow; i<layerHigh; i++)
			{
				double t = _layers[i].tau(f);
				res += t;
			}
			return res;
		}

		void atmos::loadProfile(const std::string &filename)
		{
			// Do some autodetection of file type
			// If the file begins with a number, it is one of Liu's. If
			// it starts with text ('Profile'), it is one of mine
			using namespace std;
			using namespace boost::filesystem;
			// Verify file existence
			path pt(filename);
			if (!exists(pt)) throw debug::xMissingFile(filename.c_str());
			if (is_directory(pt)) throw debug::xMissingFile(filename.c_str());

			// Open the file
			ifstream in(filename.c_str());
			if (!in) throw debug::xOtherError();
			if (!in.good()) throw debug::xEmptyInputFile(filename.c_str());

			// Look at first word.
			string buffer;
			in >> buffer;
			locale loc;

			//if (in == "Profile" || in == "Trace")
			if (buffer == "Profile")
				loadProfileRyan(filename);
			else if (isdigit(buffer[0],loc))
				loadProfileLiu(filename);
			else
				throw debug::xUnknownFileFormat(filename.c_str());
		}

		void atmos::loadProfileLiu(const std::string &filename)
		{
			// Liu's atmosphere structure is very dissimilar from my own
			// For now, all I'm doing is loading in the atmosphere, not 
			// the hydrology, which exists within a separate, unlinked file
			using namespace std;
			using namespace rtmath;
			using namespace boost::filesystem;

			// Open the file
			ifstream in(filename.c_str());

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
			_layers.resize(numLayers);
			// Loop through each layer and get information
			// As each layer is loaded, generate the appropriate atmoslayer and link
			// TODO: split logic between loading and implementing, as 
			// I need to calculate dz upon layer setup. Aargh.
			vector<double> h,p,t,rh;
			for (size_t i=0; i<numLayers;i++)
			{
				double ih, ip, it, irh;
				in >> ih >> ip >> it >> irh;
				h.push_back(ih / 1000.0); // convert m to km
				p.push_back(ip);
				t.push_back(it);
				rh.push_back(irh);
			}

			for(size_t i=0;i<numLayers;i++)
			{
				atmoslayer *layer;
				layer = &_layers[i];
				if (i<numLayers-1)
					layer->dz( h[i+1]-h[i]); // in km
				else
					layer->dz(0);
				layer->p(p[i]); // in hPa
				layer->T(t[i]); // in K
				TASSERT(layer->dz() >= 0);
				// Also, since I have relative humidity, call the appropriate functions
				double rhoWat = absorber::_Vden(t[i],rh[i]);

				// Add the absorbing gases
				// Add collision-induced broadening
				absorber *newgas = new collide;
				newgas->setLayer(*layer);
				newgas->wvden(rhoWat);
				std::shared_ptr<absorber> ptr(newgas); 
				layer->absorbers.insert(ptr);
				// Add N2
				newgas = new abs_N2;
				newgas->setLayer(*layer);
				newgas->wvden(rhoWat);
				std::shared_ptr<absorber> ptrb(newgas); 
				layer->absorbers.insert(ptrb);
				// Add O2
				newgas = new abs_O2;
				newgas->setLayer(*layer);
				newgas->wvden(rhoWat);
				std::shared_ptr<absorber> ptrc(newgas); 
				layer->absorbers.insert(ptrc);
				// Add H2O
				newgas = new abs_H2O;
				newgas->setLayer(*layer);
				newgas->wvden(rhoWat);
				std::shared_ptr<absorber> ptrd(newgas); 
				layer->absorbers.insert(ptrd);
			}
		}

		void atmos::loadProfileRyan(const std::string &filename)
		{
			using namespace std;
			using namespace rtmath;
			using namespace boost::filesystem;
			// Open the file
			ifstream in(filename.c_str());

			// Start reading
			vector<double> zlevs, plevs, tlevs, dlevs;
			size_t numgases;
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
			typedef boost::tokenizer<boost::char_separator<char> >
				tokenizer;
			boost::char_separator<char> sep(",\t");
			tokenizer tokens(buffer, sep);
			for (tokenizer::iterator it = tokens.begin();
				it != tokens.end(); ++it)
			{
				if (*it == "Altitude") continue;
				if (*it == "Pres") continue;
				if (*it == "Temp") continue;
				if (*it == "Density") continue;
				if (*it == "Number Density") continue;
				// Otherwise, add gas to conc. list, while avoiding duplicates at end
				gasnames.push_back(*it);
			}

			numgases = gasnames.size(); 

			// Get subheader line which gives the units for the columns
			getline(in,buffer);

			parser.str(buffer);
			std::string units_z, units_p, units_T, units_d;
			std::vector<std::string> units_gases;
			parser >> units_z;
			parser >> units_p;
			parser >> units_T;
			parser >> units_d;
			for (int i=0; i< (int)numgases; i++)
			{
				string j;
				parser >> j;
				units_gases.push_back(j);
			}

			// Work out any conversion factors
			// TODO

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
				for (size_t i=0;i<numgases;i++)
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
			_layers.resize(zlevs.size()-1);

			atmoslayer *layer;

			// Loop is to zlevs.size()-1 to avoid the second last read that in.good()
			// loops always seem to have and to avoid calculating dz at the end of 
			// the atmosphere
//#pragma omp parallel for private(layer,j)
			for(int i=0;i<(int)zlevs.size()-1;i++)
			{
				// Fill in the information for this layer
				layer = &_layers[i];
				if (i != (int) (zlevs.size()-2) )
					layer->dz( zlevs[i+1]-zlevs[i]); // in km
				else
					layer->dz(0); // For toa, set dz to zero
				layer->p(plevs[i]); // in hPa
				layer->T(tlevs[i]); // in K
				TASSERT(layer->dz() >= 0);

				bool hasH2O = false, hasO2 = false, hasN2 = false;
				double rhoWat = 0;
				// Loop through and add the necessary gases to the layer
				for (size_t j=0;j<numgases;j++)
				{
					// Create absorber based on gas name in gasnames[j]
					absorber *newgas;

					if ("H2O" == gasnames[j]) 
					{ 
						newgas = new abs_H2O; 
						hasH2O = true; 
						//psWV = psfrac; 
						const double Na = 6.022e23; // molecules / mol
						const double uH2O = 18.01528; // g/mol
						// dlevs is number density (molecules/cm^3)
						rhoWat = (dlevs[i] / Na) * uH2O * conc[j].at(i) * 1.0e-6;
						// rhoWat is now in g/m^3
					}
					else if ("O2" == gasnames[j]) { newgas = new abs_O2; hasO2 = true; }
					else if ("N2" == gasnames[j]) { newgas = new abs_N2; hasN2 = true; }
					else continue; // Skip if not found
					
					newgas->setLayer(*layer); 
					newgas->numConc(dlevs[i]); // Set number concentration. Will be used in lbl stuff.
					
					// Insert the gas into the layer
					// encapsulate into a pointer. Object already created, so no new needed
					std::shared_ptr<absorber> ptr(newgas); 
					layer->absorbers.insert(ptr);
				}
				// Add collision-induced broadening
				absorber *newgas = new collide;
				newgas->setLayer(*layer);
				std::shared_ptr<absorber> ptr(newgas); 
				layer->absorbers.insert(ptr);

				// Add and gases missing from the profile
				if (!hasH2O)
				{
					// Do nothing, as I can't find H2O concentration
				}

				if (!hasO2)
				{ // Note: O2 has trouble!!!
					absorber *newgas = new abs_O2;
					newgas->setLayer(*layer);
					std::shared_ptr<absorber> ptr(newgas); 
					layer->absorbers.insert(ptr);
				}

				if (!hasN2)
				{
					absorber *newgas = new abs_N2;
					newgas->setLayer(*layer);
					std::shared_ptr<absorber> ptr(newgas); 
					layer->absorbers.insert(ptr);
				}

				// Now, do another pass, recording water vapor density
				if (rhoWat) // Easily precalculated!
				{
					std::set<std::shared_ptr<absorber> >::iterator it;
					for (it = layer->absorbers.begin(); it != layer->absorbers.end(); it++)
					{
						(*it)->wvden(rhoWat);
					}
				}
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

