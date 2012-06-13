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
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/serialization/serialization.hpp>
#include <boost/serialization/string.hpp> 
#include <boost/archive/xml_oarchive.hpp>
#include <boost/archive/xml_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <cmath>
#include "../rtmath/refract.h"
#include "../rtmath/ddscat/ddparGenerator.h"
#include "../rtmath/ddscat/runScripts.h"

namespace rtmath {
	namespace ddscat {

		ddPar ddParGenerator::_s_defaultBase;

		ddParGeneratorBase::~ddParGeneratorBase()
		{
		}

		ddParGenerator::ddParGenerator()
		{
			_base = _s_defaultBase;
			import(_base);
		}

		ddParGenerator::ddParGenerator(const ddPar &base)
		{
			_base = base;
			import(_base);
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

		void ddParGenerator::write(const std::string &basename) const
		{
			using namespace boost::filesystem;
			path pBase(basename), pXML;
			if (is_directory(pBase))
			{
				pXML = pBase / "runSet.xml";
				if (exists(pXML))
				{
					if (is_directory(pXML))
						throw rtmath::debug::xPathExistsWrongType(pXML.string().c_str());
					boost::filesystem::remove(pXML);
				}
				//throw rtmath::debug::xPathExistsWrongType
			} else {
				pXML = pBase;
				if (exists(pXML))
				{
					boost::filesystem::remove(pXML);
				}
			}

			// Okay, now to serialize and output...
			std::ofstream out(pXML.string().c_str());
			//boost::archive::text_oarchive oa(out);
			// oa << *this;
			boost::archive::xml_oarchive oa(out);
			oa << BOOST_SERIALIZATION_NVP(*this);
		}

		void ddParGenerator::read(const std::string &basename)
		{
			// This routine can accept either a base directory or an actual filename
			// If a base directory is given, search for runSet.xml.
			using namespace boost::filesystem;
			path pBase(basename), pXML;
			if (!exists(pBase)) throw rtmath::debug::xMissingFile(basename.c_str());
			if (is_directory(pBase))
			{
				// Search for runSet.xml
				path pCand = pBase / "runSet.xml";
				if (exists(pCand))
				{
					if (!is_directory(pCand))
					{
						pXML = pBase;
					} else {
						throw rtmath::debug::xPathExistsWrongType(pCand.string().c_str());
					}
				} else {
					throw rtmath::debug::xMissingFile(pCand.string().c_str());
				}
			} else 
			{
				pXML = pBase;
			}

			// Okay, now to serialize and input...
			std::ifstream in(pXML.string().c_str());
			//boost::archive::text_oarchive oa(out);
			// oa << *this;
			boost::archive::xml_iarchive ia(in);
			ia >> BOOST_SERIALIZATION_NVP(*this);
		}

		void ddParGenerator::import(const std::string &ddparfilename)
		{
			throw rtmath::debug::xUnimplementedFunction();
		}

		void ddParGenerator::import(const ddPar &base)
		{
			throw rtmath::debug::xUnimplementedFunction();
		}

		void ddParIterator::write(const std::string &outfile) const
		{
			throw rtmath::debug::xUnimplementedFunction();
		}

		void ddParIterator::getrots(rotations &out) const
		{
			throw rtmath::debug::xUnimplementedFunction();
		}

		std::shared_ptr<shapeModifiable> ddParIterator::getshape() const
		{
			throw rtmath::debug::xUnimplementedFunction();
		}

		ddParIteration::ddParIteration(const ddParGenerator *src)
		{
			//throw rtmath::debug::xUnimplementedFunction();
		}

		/*
		void ddParGenerator::read(const std::string &basename)
		{
			// This routine can accept either a base directory or an actual filename
			// If a base directory is given, search for runSet.xml.
			using namespace boost::filesystem;
			path pBase(basename), pXML;
			if (!exists(pBase)) throw rtmath::debug::xMissingFile(basename.c_str());
			if (is_directory(pBase))
			{
				// Search for runSet.xml
				path pCand = pBase / "runSet.xml";
				if (exists(pCand))
				{
					if (!is_directory(pCand))
					{
						pXML = pBase;
					} else {
						throw rtmath::debug::xPathExistsWrongType(pCand.string().c_str());
					}
				} else {
					throw rtmath::debug::xMissingFile(pCand.string().c_str());
				}
			} else 
			{
				pXML = pBase;
			}

			using boost::property_tree::ptree;
			ptree pt;

			// Load the XML file into the property tree
			read_xml(pXML.string(), pt);

			// Now, extract the xml file contents

			// Allow ddscat.par import to set default vaules
			//ddPar myBasePar;
			_baseParFile = pt.get<std::string>("runset.baseDDPAR", "");
			if (_baseParFile.size())
				import(_baseParFile);

			// Basic stuff
			_compressResults = pt.get<bool>("runset.compressResults",_compressResults);
			_genIndivScripts = pt.get<bool>("runset.genIndevScripts",_genIndivScripts);
			_genMassScript = pt.get<bool>("runset.genMassScript",_genMassScript);
			_shapeStats = pt.get<bool>("runset.shapeStats",_shapeStats);
			_registerDatabase = pt.get<bool>("runset.registerDatabase",_registerDatabase);
			_doExport = pt.get<bool>("runset.export",_doExport);
			// TODO: allow setting of default values in rtmath.conf
			_exportLoc = pt.get<std::string>("runset.exportLoc","/data/rhoneyag/incoming/");
			ddscatVer = pt.get<size_t>("runset.ddscatVer", ddscatVer);
			name = pt.get<std::string>("runset.name");
			description = pt.get<std::string>("runset.description");
			outLocation = pt.get<std::string>("runset.outLocation");

			_doTorques = pt.get<bool>("runset.CMTORQ",_doTorques);
			_solnMeth = pt.get<std::string>("runset.CMDSOL", _solnMeth);
			_FFTsolver = pt.get<std::string>("runset.CMDFFT", _FFTsolver);
			_Calpha = pt.get<std::string>("runset.CALPHA", _Calpha);
			_binning = pt.get<std::string>("runset.CBINFLAG", _binning);

			_Imem1 = pt.get<int>("runset.dimension.1", _Imem1);
			_Imem2 = pt.get<int>("runset.dimension.2", _Imem2);
			_Imem3 = pt.get<int>("runset.dimension.3", _Imem3);

			_doNearField = pt.get<bool>("runset.NRFLD.do", _doNearField);
			_near1 = pt.get<double>("runset.NRFLD.1", _near1);
			_near2 = pt.get<double>("runset.NRFLD.2", _near2);
			_near3 = pt.get<double>("runset.NRFLD.3", _near3);
			_near4 = pt.get<double>("runset.NRFLD.4", _near4);
			_near5 = pt.get<double>("runset.NRFLD.5", _near5);
			_near6 = pt.get<double>("runset.NRFLD.6", _near6);

			_maxTol = pt.get<double>("runset.TOL", _maxTol);
			_maxIter = pt.get<int>("runset.MXITER", _maxIter);
			_gamma = pt.get<double>("runset.GAMMA", _gamma);
			_etasca = pt.get<double>("runset.ETASCA", _etasca);
			_nambient = pt.get<double>("runset.NAMBIENT", _nambient);

			// Process each element in the trees for frequency, size, rotations, 
			// scatt angles, shape types and temperatures
			ptree pFreqs = pt.get_child("frequencies");
			for (auto it = pFreqs.begin(); it != pFreqs.end(); it++)
			{
				// Look at each child and use to populate frequencies list
				for (auto ot = it->second.begin(); ot != it->second.end(); ot++)
				{
					// Get range and units for population
				}
			}


			ptree pSizes = pt.get_child("sizes");
			ptree pRots = pt.get_child("rotations");
			ptree pSca = pt.get_child("scatt_angles");
			ptree pShapes = pt.get_child("shapes");
			ptree pTemps = pt.get_child("temperatures");

			// At this point, we have a complete representation of the xml file contents
		}
		*/

	}
}



