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
#include "../rtmath/units.h"
#include "../rtmath/da/daDiagonalMatrix.h"
#include "../rtmath/error/error.h"
#include "../rtmath/units.h"
#include "../rtmath/ddscat/ddLoader.h"

namespace rtmath {
	
	namespace atmos {

		atmos::atmos()
		{
			_init();
		}

		void atmos::_init()
		{
		}

		atmos::~atmos()
		{
		}

		atmos::atmos(const std::string &filename)
		{
			_init();
			loadProfile(filename);
		}

		atmos::atmos(const std::string &filename, const atmos &base)
		{
			_init();
			_layers = base._layers;
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
			if (buffer == "Profile" || buffer == "Overlay")
				loadProfileRyanB(filename);
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
			units::conv_alt convAlt("m", "km");
			for (size_t i=0; i<numLayers;i++)
			{
				double ih, ip, it, irh;
				in >> ih >> ip >> it >> irh;
				h.push_back(convAlt.convert(ih)); // convert m to km
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
				layer->z(h[i]); // in km
				layer->p(p[i]); // in hPa
				layer->T(t[i]); // in K
				TASSERT(layer->dz() >= 0);
				// Also, since I have relative humidity, call the appropriate functions
				double rhoWat = absorber::_Vden(t[i],rh[i]); // g/m^3
				layer->wvden(rhoWat);

				// Get conversion to number density for population of layer->rho
				double rho, ewsat, ew;
				ewsat = absorber::_ewsat(p[i],t[i]);
				ew = rh[i] * ewsat / 100.0;
				rho = absorber::_rho(p[i],t[i],ew); // in molecules / m^3
				units::conv_dens densconv("m^-3","cm^-3");
				layer->rho(densconv.convert(rho)); // in molecules / cm^3


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

		void atmos::loadProfileRyanB(const std::string &filename)
		{
			// Should read in the same data as former loadProfileRyanA (removed), but
			// performs the read and atmospheric creation in a different manner
			using namespace std;
			using namespace rtmath;
			using namespace boost::filesystem;

			// Check if expecting an overlay
			bool isOverlay = false;
			if (this->_layers.size()) isOverlay = true;

			// Open the file
			if (!exists(path(filename))) throw rtmath::debug::xMissingFile(filename.c_str());
			ifstream in(filename.c_str());
			string line;

			// First, read in the name of the profile
			// Found in first line, starting at the second word
			// The first word is always Profile!
			getline(in,line);
			line.erase( std::remove(line.begin(), line.end(), '\r'), line.end() );
			line.erase( std::remove(line.begin(), line.end(), '\t'), line.end() );
			name = line.substr(8); // Skip 'Profile '

			// Prepare the tokenizer
			// The rest of the lines in this file are csv or tsv
			typedef boost::tokenizer<boost::char_separator<char> >
				tokenizer;
			boost::char_separator<char> sep(",\t");


			// Read in header and subheader lines
			vector<string> header, subheader;
			getline(in,line);
			line.erase( std::remove(line.begin(), line.end(), '\r'), line.end() );
			tokenizer tokens(line,sep);
			for (tokenizer::iterator it = tokens.begin();
				it != tokens.end(); it++)
				header.push_back(*it);
			getline(in,line);
			line.erase( std::remove(line.begin(), line.end(), '\r'), line.end() );
			tokens.assign(line);
			for (tokenizer::iterator it = tokens.begin();
				it != tokens.end(); it++)
				subheader.push_back(*it);

			// Parse header and subheader
			// Header columns may include 'Alt/Altitude', 'P/Pres/Pressure', 
			// 'T/Temp/Temperature', 'D/Dens/Density,rho' (type of density  
			// based on units), 'Humidity', and the gas names

			// In general, a complete atmospheric description requires
			// each layer to have pressure/altitude, temp/density/humidity
			// Note: the necessary number of quantities MUST be given for the 
			// atmosphere load to succeed! I need to have a complete, working 
			// atmosphere at the end of the load.
			// Also, the hydrometeor profiles / phase functions for nongaseous
			// particles are not loaded in this file. This is incorporated through
			// an overlay file.

			size_t counter = 0;
			bool hAlt = false, hP = false, hT = false, hD = false, hPf = false;
			size_t cAlt = 0, cP = 0, cT = 0, cD = 0, cPf = 0;
			set<size_t> cGases;
			string convSubheader;
			map<size_t, std::unique_ptr<units::atmosConv> > converters;
			//std::unique_ptr<> pfConverter;
			// Iterate over the header and subheader. Use counter to count the numeric
			// column number. Can consider header and subheader together.
			for (auto it = header.begin(), ot = subheader.begin(); 
				it != header.end(); it++, ot++, counter++)
			{
				// Too bad I can't use a switch here
				// Take lower-case version of header to ease comparisons
				string sCol = *it; // Copy to preserve gas names!
				std::transform(sCol.begin(), sCol.end(), sCol.begin(), ::tolower);

				unique_ptr<units::atmosConv> cv;
				// Find key columns and use the subcolumn information to set up any 
				// necessary unit conversions
				if (sCol == "altitude" || sCol == "alt")
				{
					cAlt = counter;
					hAlt = true;
					// Want altitude in km
					// see rtmath-units
					// rtmath::converter convAlt(*ot,"km");
					cv = unique_ptr<units::atmosConv>(new units::conv_alt(*ot,"km"));
				} else if (sCol == "p" || sCol == "pres" || sCol == "pressure")
				{
					cP = counter;
					hP = true;
					// Want pressure in mb / hPa
					cv = unique_ptr<units::atmosConv>(new units::conv_pres(*ot,"hPa"));
				} else if (sCol == "t" || sCol == "temp" || sCol == "temperature")
				{
					cT = counter;
					hT = true;
					// Want temperature in K
					cv = unique_ptr<units::atmosConv>(new units::conv_temp(*ot,"K"));
				} else if (sCol == "d" || sCol == "dens" || sCol == "density" || sCol == "rho")
				{
					cD = counter;
					hD = true;
					// Want density in number / cm^-3
					cv = unique_ptr<units::atmosConv>(new units::conv_dens(*ot,"cm^-3"));
				} else if (sCol == "pfdata" || sCol == "pf" || 
						sCol == "phasefunction" || sCol == "phase function")
				{
					// Phase function information has been provided for this layer. This includes 
					// scattering and extinction matrices, or something that can generate these
					// This may be specified as a directory name, with spaces allowed since the 
					// separators are commas and tabs.
					// Additionally, this may be specified as a database id that maps to the appropriate 
					// phase function path. The id may be either numeric or may be a known key. 
					// The database entries may reference either postgresql code or a symlink undet a path 
					// known to rtmathconf. These may be distinguished from each other by the subheader fields.
					// Note: only one phase function may be specified at this stage, because otherwise it's too 
					// much coding / inconvenience. Effective pfs should be calculated first anyways.
					//
					// TODO: come up with a converter that does the appropriate mapping to an actual file!
					cPf = counter;
					convSubheader = *ot;
					hPf = true;
				} else {
					// Everything else should be a gas concentration
					// These should be in ppmv
					cv = unique_ptr<units::atmosConv>(new units::conv_dens(*ot,"ppmv"));
					cGases.insert(counter);
				}
				// Insert the converter into the appropriate structure
				converters[counter] = move(cv);
			}

			if (!isOverlay) // Quick check to end processing here
				if (!(hAlt && hP && hT && hD))
					throw rtmath::debug::xBadInput("Base atmosphere is incomplete.");
			if (isOverlay)
				if (!(hP)) // Altitude is possible, but harder to map
					throw rtmath::debug::xBadInput("Overlay does not contain pressure.");

			// The headers are loaded and the converters are in converters!
			map<size_t,vector<double> > vals;
			map<size_t,string> pfVals;
			// Begin processing the actual atmosphere
			while (in.good()) // Think up better reader that doesn't duplicate the last line
			{
				getline(in,line);
				line.erase( std::remove(line.begin(), line.end(), '\r'), line.end() );
				tokens.assign(line);
				counter = 0; // Keeps track of column number
				for (tokenizer::iterator it = tokens.begin();
					it != tokens.end(); it++, counter++)
				{
					if (counter == cPf && hPf)
					{
						// Special phase function handling!
						if (pfVals.count(counter) == 0)
							pfVals[counter] = *it;
					} else {
						// Use standard numeric converters
						istringstream ival(*it);
						double ivald;
						ival >> ivald;
						double quant = converters[counter]->convert( ivald );
						vector<double> test;
						if (vals.count(counter) == 0) vals[counter] = test; // Creation of value in map
						vals[counter].push_back(quant);
					}
				}
			}

			// Do final processing and modification / creation of atmoslayers
			// Do a loop over the altitude or the pressure
			//size_t loopCol = (hP) ? cP : cAlt; // This is the column that will be used in overlay comparisons
			// Pressure is preferred as layer stores dz only
			size_t loopCol = cP;
			double lastP = 0;
			for (size_t i=0;i<vals[loopCol].size();i++)
			{
				atmoslayer *layer = nullptr;
				if (vals[cP][i] == lastP) continue; // Safety check that skips duplicate reads of layers
				if (!isOverlay)
				{
					// Careful about the bounds!
					double dZ;
					if (i == vals[loopCol].size()-1)
						dZ = 0;
					else dZ= vals[cAlt][i+1]-vals[cAlt][i];
					// TODO: add support for computing number density from relative humidity
					atmoslayer nl(vals[cP][i],vals[cT][i],vals[cAlt][i],dZ, vals[cD][i]);
					_layers.push_back(nl);
					layer = &_layers[i];
				} else {
					// Scan layers for the matching one
					bool done = false;
					for (size_t j=0; j < _layers.size() && done == false; j++)
						if (_layers[j].p() == vals[cP][i])
						{
							done = true;
							layer = &_layers[j];
						}
					if (!done) throw rtmath::debug::xBadInput("Overlay does not match any existing pressure.");
				}


				// Insert any new gases. Replace existing gas if encountered.
				bool hasH2O = false, hasO2 = false, hasN2 = false;
				double rhoWat = 0; // Will be set once found.
				for (auto j = cGases.begin(); j != cGases.end(); j++)
				{
					string gasname = header[*j];
					// Scan for existing gas by this name
					bool duplicate = false;
					set<shared_ptr<absorber> >::const_iterator dupIt;
					for (auto ot = layer->absorbers.begin(); ot != layer->absorbers.end() && duplicate == false; ot++)
					{
						string aname;
						(*ot)->getName(aname);
						if (aname == gasname) { duplicate = true; dupIt = ot; }
					}
					if (duplicate)
					{
						// Remove existing gas entry. Assumes only one entry for each gas type.
						layer->absorbers.erase(dupIt);
					}
					absorber *newgas = nullptr; // encapsulates in shared pointer. no manual delete.
					if ("H2O" == gasname) 
					{ 
						newgas = new abs_H2O; 
						hasH2O = true; 
						//psWV = psfrac; 
						const double Na = 6.022e23; // molecules / mol
						const double uH2O = 18.01528; // g/mol
						// dlevs is number density (molecules/cm^3)
						// Using existing layer density
						//layer->
						rhoWat = (vals[cD][i] / Na) * uH2O * vals[*j][i] * 1.0e-6;
						// rhoWat is now in g/m^3
						layer->wvden(rhoWat);
					}
					else if ("O2" == gasname) { newgas = new abs_O2; hasO2 = true; }
					else if ("N2" == gasname) { newgas = new abs_N2; hasN2 = true; }
					else continue; // Skip to next gas if not found
					
					newgas->setLayer(*layer); 
					newgas->numConc(vals[cD][i]); // Set number concentration. Will be used in lbl stuff.
					
					// Insert the gas into the layer
					// encapsulate into a pointer. Object already created, so no new needed
					std::shared_ptr<absorber> ptr(newgas); 
					layer->absorbers.insert(ptr);
				}
				// Fill in any missing gases if this is a base atmos load.
				if (!isOverlay)
				{
					if (!hasH2O)
					{
						// Do nothing, as I can't find H2O concentration
					}

					if (!hasO2)
					{ // Note: O2 has trouble!!!
						absorber *newgas = new abs_O2;
						newgas->setLayer(*layer);
						newgas->numConc(vals[cD][i]);
						std::shared_ptr<absorber> ptr(newgas); 
						layer->absorbers.insert(ptr);
					}

					if (!hasN2)
					{
						absorber *newgas = new abs_N2;
						newgas->setLayer(*layer);
						newgas->numConc(vals[cD][i]);
						std::shared_ptr<absorber> ptr(newgas); 
						layer->absorbers.insert(ptr);
					}
				}

				// Need to update water vapor density in each layer
				{
					// If H2O read (base or overlay), rhowat is already calculated. 
					// If not, if overlay and no H2O, check existing absorbers
					if (!hasH2O && isOverlay)
					{
						for (auto it = layer->absorbers.begin(); it != layer->absorbers.end(); it++)
						{
							string name;
							(*it)->getName(name);
							if (name == "H2O")
							{
								rhoWat = (*it)->wvden();
								break;
							}
						}
					}
					// Now, do another pass, recording water vapor density
					if (rhoWat) // Easily precalculated!
						for (auto it = layer->absorbers.begin(); it != layer->absorbers.end(); it++)
						{
							(*it)->wvden(rhoWat);
						}
				}

				// Insert the phase function, if specified. Replace any existing one.
				if (hPf)
				{
					// cPf is the column id for the pf id string
					string id = pfVals[i];
					unique_ptr<ddLoader> layerConv = ddLoader::findLoader(id, convSubheader);
					// Last step...
					layer->setPfLoader( move(layerConv) ); // Move into layer and invalidate layerConv
				}

				lastP = vals[cP][i]; // Safety check
			} // End of a very long loop
			// Finally done. Such a long function.
		}

		void atmos::saveProfile(const std::string &filename) const
		{
			std::cerr << "saveProfile not yet implemented, so " << filename << 
				" will not be written.\n";
			throw rtmath::debug::xUnimplementedFunction();
		}


	}; // end namespace atmos

}; // end namespace rtmath

