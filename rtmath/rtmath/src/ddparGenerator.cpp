#include "../rtmath/Stdafx.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <memory>
#include <string>
#include <vector>
#include <map>
#include <set>
#include <unordered_map>
#include <complex>
#include <boost/filesystem.hpp>
#include <boost/tokenizer.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <boost/math/constants/constants.hpp>
#include <cmath>
#include "../rtmath/refract.h"
#include "../rtmath/ddscat/ddparGenerator.h"
#include "../rtmath/ddscat/runScripts.h"

namespace rtmath {
	namespace ddscat {

		ddParGeneratorBase::~ddParGeneratorBase()
		{
		}

		ddParGenerator::ddParGenerator()
		{
		}

		ddParGenerator::ddParGenerator(const ddPar &base)
		{
			_base = base;
			// Figure out the type of shape, and produce an appropriate base shape
		}

		ddParGenerator::~ddParGenerator()
		{
		}

		void ddParGenerator::generate(const std::string &basedir) const
		{
			using namespace boost::filesystem;
			using namespace std;

			// Check if base directory exists. If not, try to create it.
			path bpath(basedir);
			{
				if (!exists(bpath)) boost::filesystem::create_directory(bpath);
				if (exists(bpath))
					if (!is_directory(bpath))
						throw rtmath::debug::xPathExistsWrongType(bpath.string().c_str());
			}

			// Keep a list of the assigned subdirectories
			std::set<std::string> dirs;

			// Now, iterate over the conbinations of initial parameters.
			ddParIteration itSetup(this);
			ddParIteration::const_iterator it;
			for (it = itSetup.begin(); it != itSetup.end(); it++)
			{
				// Now, the iterated quantity contains the individual frequency, shape, size, ...
				// for this run in the set of all combinations of parameters for a run.
				// Of course, some unit conversion is anticipated, as we've really just done some 
				// preprocessing and alias expansion. The whole purpose of ddParIteration was to 
				// do this setup and construct the iterated quantities.

				// Take copy of shape and apply properties
				///shared_ptr<shapeModifiable> nS( (shapeModifiable*) it->getshape()->clone() );
				using namespace MANIPULATED_QUANTITY;

				double TK;		// Temp in K
				double fGHz;	// Freq in GHz
				double um;		// wavelength in um

				// Make copy of base
				ddPar parout = _base;
				// TODO: check that splicing does not occur. If it does, use a better cast.
				std::shared_ptr<shapeModifiable> shape ( (shapeModifiable*) it->getshape()->clone() );

				// Update shape parameters based on quantities
				shapeModifiable::vertexMap mappings;
				shape->getVertices(mappings);		 // For the current shape, get a list of all possible mappings
				rtmath::graphs::setWeakVertex known; // The list of known vertices
				for (auto ot = it->_params.begin(); ot != it->_params.end(); ot++)
				{
					string quant = ot->first, units = ot->second.second;
					double val = ot->second.first;

					if (quant == "temp")
					{
						it->getParamValue<double>("temp",val,units);
						units::conv_temp tconv(units, "K");
						TK = tconv.convert(val);
						shape->set(TEMP, TK);
						known.insert(mappings.left.at(TEMP));
					} else if (quant == "freq")
					{
						it->getParamValue<double>("freq",val,units);
						units::conv_spec fconv(units, "GHz");
						fGHz = fconv.convert(val);
						shape->set(FREQ, fGHz);
						units::conv_spec umconv(units, "um");
						um = umconv.convert(val); // Wavelength in microns
						known.insert(mappings.left.at(FREQ));
					} // Allow setting of other quantities
					else {
						// Ask the shape to attempt to match the variable
						it->getParamValue<double>(quant,val,units);
						// NOTE: No unit conversions added. TODO!
						bool hasMap;
						size_t vid;
						std::shared_ptr< rtmath::graphs::vertex > vert;
						hasMap = shape->mapVertex(quant, vid, vert);
						// If it fails, then report an error
						if (!hasMap) continue; // TODO: make an error.
						shape->set((MQ) vid, val); // TODO: fix casting, as not MQ only is for 
													// the base vertices
					}
				}
				shape->update(known);

				// Set rotations
				rotations rots;
				it->getrots(rots);
				rots.out(parout);

				// Set frequency and wavelength
				shared_ptr<ddParParsers::ddParLineMixed<double, std::string> > wvlens
					( new ddParParsers::ddParLineMixed<double, std::string>(3, ddParParsers::WAVELENGTHS));
				wvlens->set<double>(0,um);
				wvlens->set<double>(1,um);
				wvlens->set<double>(2,1.0);
				wvlens->set<std::string>(3,"LIN");
				parout.insertKey(ddParParsers::WAVELENGTHS,static_pointer_cast<ddParParsers::ddParLine>(wvlens));

				// Finish shape generation, then get calculated reff
				double reff = shape->get(REFF); // in um by default
				shared_ptr<ddParParsers::ddParLineMixed<double, std::string> > reffline
					( new ddParParsers::ddParLineMixed<double, std::string>(3, ddParParsers::WAVELENGTHS));
				reffline->set<double>(0,reff);
				reffline->set<double>(1,reff);
				reffline->set<double>(2,1.0);
				reffline->set<std::string>(3,"LIN");
				parout.insertKey(ddParParsers::AEFF,static_pointer_cast<ddParParsers::ddParLine>(reffline));



				// Now, write the files!!!!!
				// Generate semirandom subdirectory name
				using namespace boost::uuids;
				random_generator gen;
				uuid u = gen();
				string dirname = boost::uuids::to_string(u);
				// Create the subdirectory
				path pdir = bpath / dirname;
				create_directory(pdir);

				// Write diel.tab
				// Calculate and set index of refraction (just for diel.tab. done in shapes.cpp already)
				std::complex<double> m;
				// TODO: support for different materials and 
				//		 refractive index functions
				refract::mice(fGHz, TK, m);
				refract::writeDiel( (pdir/"diel.tab").string(), m );

				// Write shape.dat (if needed)
				if (shape->canWrite())
					shape->write( (pdir/"shape.dat").string() );

				// Write ddscat.par
				parout.saveFile( (pdir/"ddscat.par").string() );

				// Write run definitions
				// Give the output class access to the graph vertices.
				it->write( (pdir/"params.txt").string() );

				// Write run script
				runScriptIndiv iscript(dirname);
				iscript.write(dirname);

				// Lastly, add this path into the global run script listing...
				dirs.insert(dirname);
			}


			// Now to write the global run script
			// This script will iterate through all of the directories and execute the individual runs

			runScriptGlobal glb;
			glb.addSubdir(dirs);
			glb.write(bpath.string());

		}

		void ddParGenerator::write(const std::string &basedir) const
		{
		}

		void ddParGenerator::read(const std::string &basedir)
		{
		}
	}
}



