#include "../rtmath/Stdafx.h"
#include <iostream>
#include <algorithm>
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

		ddParGeneratorBase::ddParGeneratorBase()
			:
		ddscatVer(72),
			compressResults(false),
			genIndivScripts(true),
			genMassScript(true),
			shapeStats(false),
			registerDatabase(false),
			doExport(true),
			// Fill in zeros here, as base ddpar 
			// does not always import over
			doTorques(false),
			Imem1(0),
			Imem2(0),
			Imem3(0),
			doNearField(false),
			near1(0),
			near2(0),
			near3(0),
			near4(0),
			near5(0),
			near6(0),
			maxTol(0),
			maxIter(0),
			gamma(0),
			etasca(0),
			nambient(0),
			doSca(true)
		{

		}

		ddParGenerator::ddParGenerator()
		{
			base = _s_defaultBase;
		}

		ddParGenerator::ddParGenerator(const ddPar &base)
		{
			this->base = base;
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
			ddParIteration itSetup(*this);
			ddParIteration::const_iterator it;
			for (it = itSetup.begin(); it != itSetup.end(); it++)
			{
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
				it->exportDiel( (pdir/"diel.tab").string() );
				// Write shape.dat (if needed)
				it->exportShape( (pdir/"shape.dat").string() );
				// Write ddscat.par
				it->exportDDPAR( (pdir/"ddscat.par").string() );
				// Write run definitions
				ddParIterator::write(*it, (pdir).string() );
				// Write individual run script
				runScriptIndiv iscript(dirname, *this);
				iscript.write(dirname);
				// Lastly, add this path into the global run script listing...
				dirs.insert(dirname);
			}


			// Now to write the global run script
			// This script will iterate through all of the directories and execute the individual runs

			runScriptGlobal glb(*this);
			glb.addSubdir(dirs);
			glb.write(bpath.string());

		}

		void ddParGenerator::write(const ddParGenerator &obj, const std::string &basename)
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
			oa << BOOST_SERIALIZATION_NVP(obj);
		}

		void ddParGenerator::read(ddParGenerator &obj, const std::string &basename)
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
						pXML = pCand;
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
			ia >> BOOST_SERIALIZATION_NVP(obj);
		}

		void ddParGenerator::import(const std::string &ddparfilename)
		{
			using namespace boost::filesystem;
			path p(ddparfilename);
			if (!exists(ddparfilename)) throw rtmath::debug::xMissingFile(ddparfilename.c_str());
			rtmath::ddscat::ddPar ddfile(ddparfilename);
			base = ddfile;
		}

		void ddParIterator::read(ddParIterator &obj, const std::string &file)
		{
			// It's yet another serialization case
			using namespace boost::filesystem;
			path pBase(file), pXML;
			if (is_directory(pBase))
			{
				pXML = pBase / "ddParIterator.xml";
				if (!exists(pXML))
					throw rtmath::debug::xMissingFile(pXML.string().c_str());
				else if (is_directory(pXML))
					throw rtmath::debug::xPathExistsWrongType(pXML.string().c_str());
			} else {
				pXML = pBase;
				if (!exists(pXML))
					throw rtmath::debug::xMissingFile(pXML.string().c_str());
			}

			// Okay, now to serialize and output...
			std::ifstream in(pXML.string().c_str());
			boost::archive::xml_iarchive oa(in);
			oa >> BOOST_SERIALIZATION_NVP(obj);
		}

		void ddParIterator::write(const ddParIterator &obj, const std::string &outfile)
		{
			// It's yet another serialization case
			using namespace boost::filesystem;
			path pBase(outfile), pXML;
			if (is_directory(pBase))
			{
				pXML = pBase / "ddParIterator.xml";
				if (exists(pXML))
				{
					if (is_directory(pXML))
						throw rtmath::debug::xPathExistsWrongType(pXML.string().c_str());
					boost::filesystem::remove(pXML);
				}
			} else {
				pXML = pBase;
				if (exists(pXML))
				{
					boost::filesystem::remove(pXML);
				}
			}

			// Okay, now to serialize and output...
			std::ofstream out(pXML.string().c_str());
			boost::archive::xml_oarchive oa(out);
			oa << BOOST_SERIALIZATION_NVP(obj);
		}

		void ddParIterator::exportShape(const std::string &filename) const
		{
			if (shape->canWrite())
				shape->write(filename);
		}

		void ddParIterator::exportDiel(const std::string &filename) const
		{
			using namespace std;
			// Figure out the frequency and temperature in GHz and K.
			complex<double> m;
			double val;
			string units;

			double fGHz;
			auto it = shape->shapeConstraints.find("freq");
			val = *(it->second->pset.begin());
			units = it->second->units;
			units::conv_spec fconv(units, "GHz");
			fGHz = fconv.convert(val);

			double TK;
			it = shape->shapeConstraints.find("temp");
			val = *(it->second->pset.begin());
			units = it->second->units;
			units::conv_temp tconv(units, "K");
			TK = tconv.convert(val);

			refract::mice(fGHz, TK, m);
			refract::writeDiel(filename,m);

			GETOBJKEY();
			// TODO: add support for different refractive index calculators and 
			// for different materials (like iron)
		}

		void ddParIterator::exportDDPAR(ddPar &out) const
		{
			if (shape->useDDPAR() == false) return;
			// Most of the ddPar file will be filled in from the base ddParGenerator class,
			// with a few overrides from the iterator. Some of the properties come from the 
			// shapeModifiable member, such as effective radius.

			// Starting with the base (provided by ddParGenerator)
			out = _gen.base;

			// Apply iterator-specific properties
			// Rotations
			rots->out(out);

			// Also apply the shape properties to the ddPar output
			// This means: set CSHAPE, SHPAR1, SHPAR2, SHPAR3, AEFF
			// Wavelength
			shape->setDDPAR(out);

			// And that's really it for the ddscat.par file...
		}

		void ddParIterator::exportDDPAR(const std::string &filename) const
		{
			if (shape->useDDPAR() == false) return;
			ddPar outpar;
			exportDDPAR(outpar);
			outpar.saveFile(filename);
		}

		ddParIterator::ddParIterator(const ddParGenerator &gen, std::unique_ptr<shapeModifiable> shp)
			: _gen(gen)
		{
			shape = std::move(shp);
		}

		ddParIteration::ddParIteration(const ddParGenerator &src)
			: _gen(src)
		{
			_populate();
		}

		void ddParIteration::_populate()
		{
			using namespace std;

			/*	Iterate over all possible variations
			Variations include, but are not limited to: 
			shape, indiv shape params / global params,
			freq, temp, size (vol, eff rad).
			Go through each combo and create a ddParIterator expressing each one

			Take each shape entry
			It must come first, as the global params need to be added to the local params...
			*/
			for (auto rt = _gen.rots.begin(); rt != _gen.rots.end(); rt++)
			{
				// rt refers to the rotations. Rotations are more complex objects, so they are 
				// handled outside of the standard shapeConstraints
				for (auto it = _gen.shapes.begin(); it != _gen.shapes.end(); it++)
				{
					shapeConstraintContainer shapeConstraintsEffective;
					shapeConstraintsEffective = _gen.shapeConstraintsGlobal;
					for (auto ot = (*it)->shapeConstraints.begin(); ot != (*it)->shapeConstraints.end(); ot++)
						shapeConstraintsEffective.insert(*ot);


					// first - parameter name. pair->first is value, pair->second is units
					multimap<string, pair<double, string> > constraintsVaried;
					set<string> variedNames;

					// We now have the effective shape constraints. Gather those that need no expansion as a base.
					// Also, for the varied ones, gather them up into constraintsVaried
					shapeConstraintContainer shapeConstraintsEffectiveBase;
					for (auto ot = shapeConstraintsEffective.begin(); ot != shapeConstraintsEffective.end(); ot++)
					{
						bool unique = false;
						if (shapeConstraintsEffective.count(ot->first) == 1)
							if ((ot->second->size() == 1))
							{
								shapeConstraintsEffectiveBase.insert(*ot);
								unique = true;
							}
							if (!unique)
							{
								for (auto ut = ot->second->begin(); ut != ot->second->end(); ut++)
								{
									auto p = std::make_pair<double,string>(*ut, ot->second->units);
									constraintsVaried.insert(
										pair<string,pair<double,string> >(ot->first,p));
									if (variedNames.count(ot->first) == 0)
										variedNames.insert(ot->first);
								}
							}
					}

					// constraintsVaried now contains all possible constraint variations. Iterate over these to 
					// produce the ddParIterator entries in _elements

					// get rid of duplicates (they are possible still)
					{
						multimap<string, pair<double, string> > constraintsVariedUnique;
						multimap<string, pair<double, string> >::iterator dt;
						dt = unique_copy(constraintsVaried.begin(), constraintsVaried.end(), constraintsVariedUnique.begin());
						constraintsVaried = constraintsVariedUnique;
					}

					// Collect the elements into sets of variedNames
					map<string, set<pair<double,string> > > vmap;
					for (auto ot = variedNames.begin(); ot != variedNames.end(); ot++)
					{
						//pair<multimap<string,pair<double,string> >::iterator, multimap<string,pair<double,string> >::iterator >
						auto ret = constraintsVaried.equal_range(*ot);
						for (auto ut = ret.first; ut != ret.second; ut++)
						{
							if (!vmap.count(*ot))
							{
								static const set<pair<double,string> > base;
								vmap.insert(pair<string,set<pair<double,string> > >(*ot,base));
							}
							vmap.at(*ot).insert(*ut);
						}
					}

					// And now to permute everything.....
					// Set all iterators to begin
					map<string, set<pair<double,string> >::iterator> mapit;
					for (auto ot = variedNames.begin(); ot != variedNames.end(); ot++)
						mapit[*ot] = vmap[*ot].begin();
					bool done = false;
					// Permute until all are at end
					while (!done)
					{
						shapeConstraintContainer permuted = shapeConstraintsEffectiveBase; // initialize
						for (auto ot = mapit.begin(); ot != mapit.end(); ot++)
						{
							// Explicit, since it's hard to remember by this point
							double val;
							string name, units;

							val = ot->second->first;
							name = ot->first;
							units = ot->second->second;

							// Construct permuted object
							permuted.insert(pair<string,shared_ptr<shapeConstraint> >(
								name, make_shared<shapeConstraint>(name,val,units) ) );

							// As a reminder, (it) is an iterator to the current shape.....
							unique_ptr<shapeModifiable> nshape( dynamic_cast<shapeModifiable*>((*it)->clone()) );
							nshape->shapeConstraints = permuted;
							nshape->generate();

							// Make the iterator
							ddParIterator nit(_gen, move(nshape) );
							// Add the rotations
							nit.rots = *rt;
							// Insert into _elenemts using rvalue move (avoids copying)
							_elements.insert(move(nit));

						}

						// Advance the iterators
						// Advance the "leftmost" iterator until it hits the end. Then, reset it and advance one further.
						auto mit = mapit.begin();
						bool good = false;
						auto cit = mit->second;
						while (!done)
						{
							cit++;
							if (cit == vmap[mit->first].end())
							{
								cit = vmap[mit->first].begin();
								mit++;
							} else break;
							// Can't return here - only the current shape is done...
							if (mit == mapit.end()) done = true; // Breaks two loops, and moves on to next shape...
						}

					}
				}
			}


		}

	}
}

/*
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
GETOBJKEY();
//				parout.insertKey(ddParParsers::WAVELENGTHS,static_pointer_cast<ddParParsers::ddParLine>(wvlens));

// Finish shape generation, then get calculated reff
double reff = shape->get(REFF); // in um by default
shared_ptr<ddParParsers::ddParLineMixed<double, std::string> > reffline
( new ddParParsers::ddParLineMixed<double, std::string>(3, ddParParsers::WAVELENGTHS));
reffline->set<double>(0,reff);
reffline->set<double>(1,reff);
reffline->set<double>(2,1.0);
reffline->set<std::string>(3,"LIN");
//	parout.insertKey(ddParParsers::AEFF,static_pointer_cast<ddParParsers::ddParLine>(reffline));


*/
