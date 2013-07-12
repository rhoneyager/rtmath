#include "../rtmath/Stdafx.h"
#pragma warning( disable : 4996 ) // -D_SCL_SECURE_NO_WARNINGS
#pragma warning( disable : 4503 ) // decorated name length exceeded. with boost bimap mpl
#include <Ryan_Serialization/serialization.h>
#include <boost/filesystem.hpp>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>
#include "../rtmath/refract.h"
#include "../rtmath/ddscat/ddparGenerator.h"
#include "../rtmath/ddscat/ddVersions.h"
#include "../rtmath/ddscat/runScripts.h"

#include "../rtmath/error/error.h"

namespace rtmath {
	namespace ddscat {

		ddParGeneratorBase::~ddParGeneratorBase() {}

		ddParGeneratorBase::ddParGeneratorBase() : 
			compressResults(false),
			genIndivScripts(true),
			genMassScript(true),
			shapeStats(false),
			registerDatabase(false),
			doExport(true)
		{
			ddscatVer = rtmath::ddscat::ddVersions::getDefaultVer();
		}

		ddParGenerator::ddParGenerator()
		{
			base = *(ddPar::defaultInstance());
			base.populateDefaults();
		}

		ddParGenerator::ddParGenerator(const ddPar &base)
		{
			this->base = base;
			// Figure out the type of shape, and produce an appropriate base shape
		}

		ddParGenerator::~ddParGenerator() {}

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
			//Ryan_Serialization::write<ddParIteration>(itSetup, "debugIteration.xml");
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

				// Have shape handle the writing information
				// Not all need diel.tab, ddscat.par or a standard shape.dat file
				(*it)->shape->write( pdir.string(), base );

				// Write run definitions
				::Ryan_Serialization::write<ddParIterator>(**it,(pdir/"ddparIterator.xml").string());

				// Write individual run script
				runScriptIndiv iscript = (*it)->shape->prepRunScript(dirname, *this);
				iscript.write((pdir).string());

				// Lastly, add this path into the global run script listing...
				dirs.insert(dirname);
			}


			// Now to write the global run script
			// This script will iterate through all of the directories and execute the individual runs

			runScriptGlobal glb(*this);
			glb.addSubdir(dirs);
			glb.write(bpath.string());

		}

		void ddParGenerator::import(const std::string &ddparfilename)
		{
			using namespace boost::filesystem;
			path p(ddparfilename);
			if (!exists(ddparfilename)) throw rtmath::debug::xMissingFile(ddparfilename.c_str());
			rtmath::ddscat::ddPar ddfile(ddparfilename);
			base = ddfile;
		}

		ddParIterator::ddParIterator()
			: _genp(nullptr)
		{
		}

		ddParIterator::ddParIterator(const ddParGenerator &gen, boost::shared_ptr<shapeModifiable> shp)
			: _genp(&gen)
		{
			shape = shp;
		}

		ddParIteration::ddParIteration(const ddParGenerator &src)
			: _genp(&src)
		{
			_populate();
		}

		ddParIteration::ddParIteration()
			: _genp(nullptr)
		{
		}

		void ddParIteration::_populate()
		{
			using namespace std;

			if (!_genp) return;
			const ddParGenerator &_gen = *_genp;
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
					shapeConstraintsEffective = _gen.shapeConstraints;
					for (auto ot = (*it)->shapeConstraints.begin(); ot != (*it)->shapeConstraints.end(); ot++)
						shapeConstraintsEffective.insert(*ot);


					// first - parameter name. pair->first is value, pair->second is units
					multimap<const string, rtmath::units::unit_pair> constraintsVaried;
					//multimap<string, pair<double, string> > constraintsVaried;
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
									std::pair<double, std::string> p = std::pair<double,string>(*ut, ot->second->units);
									const string &name = ot->first;
									const string &units = ot->second->units;
									const double val = *ut;
									constraintsVaried.insert(pair<const string,units::unit_pair>(name, units::unit_pair(val,units)));
									//constraintsVaried.insert(make_pair<string,pair<double,string> >(name,make_pair<double,string>(val, units)));
									if (variedNames.count(ot->first) == 0)
										variedNames.insert(ot->first);
								}
							}
					}

					// constraintsVaried now contains all possible constraint variations. Iterate over these to 
					// produce the ddParIterator entries in _elements
					/*
					// get rid of duplicates (they are possible still)
					{
						multimap<const string, rtmath::units::unit_pair > constraintsVariedUnique;
						multimap<const string, rtmath::units::unit_pair >::iterator dt;
						// unique_copy will not work due to predicate differences
						//dt = unique_copy(constraintsVaried.begin(), constraintsVaried.end(), constraintsVariedUnique.begin());
						for (auto ot = constraintsVaried.begin(); ot != constraintsVaried.end(); ++ot)
						{
							if (!constraintsVariedUnique.count(ot->first))
								constraintsVariedUnique.insert(*ot);
						}
						constraintsVaried = constraintsVariedUnique;
					}
					*/
					// Collect the elements into sets of variedNames
					map<const string, set<rtmath::units::unit_pair > > vmap;
					for (auto ot = variedNames.begin(); ot != variedNames.end(); ot++)
					{
						//pair<multimap<string,pair<double,string> >::iterator, multimap<string,pair<double,string> >::iterator >
						auto ret = constraintsVaried.equal_range(*ot);
						for (auto ut = ret.first; ut != ret.second; ut++)
						{
							if (!vmap.count(*ot))
							{
								static const set<rtmath::units::unit_pair > base;
								vmap.insert(pair<string,set<rtmath::units::unit_pair > >(*ot,base));
							}
							vmap.at(*ot).insert(ut->second);
						}
					}
					
					// And now to permute everything.....
					// Set all iterators to begin
					// map<const string, set<rtmath::units::unit_pair > > vmap;
					map<string, set<rtmath::units::unit_pair >::iterator> mapit;
					for (auto ot = variedNames.begin(); ot != variedNames.end(); ot++)
						mapit[*ot] = vmap[*ot].begin();

					auto mit = mapit.begin();
					auto cit = &(mit->second); // TODO: begin debugging here

					// Loops will break when the end condition is detected

					do {
						// First, reset the iterators for incrementation
						if (mit != mapit.begin())
						{
							// Set range of mapits beteewn begin and mit to vmap[first].begin()
							// This leaves the most significant previously modified iterator 
							// unchanged, as desired
							for (auto rit = mapit.begin(); rit != mit; rit++)
							{
								rit->second = vmap[rit->first].begin();
							}
							// And don't forget to reset mit and cit
							mit = mapit.begin();
							cit = &(mit->second);
						}

						// Do object construction. In separate block for readability
						{
							//static int i=0;
							//i++;
							//std::cout << "Constructing object " << i << std::endl;
							shapeConstraintContainer permuted = shapeConstraintsEffectiveBase; // initialize
							for (auto ot = mapit.begin(); ot != mapit.end(); ot++)
							{
								// Explicit, since it's hard to remember by this point
								double val;
								string name, units;

								val = ot->second->first;
								name = ot->first;
								units = ot->second->second;

								//std::cout << "\t" << name << "\t" << val << "\t" << units << std::endl;

								// Construct permuted object
								permuted.insert(pair<string,shapeConstraintPtr >(
									name, shapeConstraintPtr
									(new shapeConstraint(name,boost::lexical_cast<std::string>(val),units) ) ) );
							}
							// As a reminder, (it) is an iterator to the current shape.....
							boost::shared_ptr<shapeModifiable> nshape( (*it)->clone() );
							nshape->shapeConstraints = permuted;
							nshape->update();

							nshape->setRots(*rt);

							// Make the iterator
							boost::shared_ptr<ddParIterator> nit(new ddParIterator(_gen, nshape ));
							// Add the rotations
							nit->rots = *rt;
							// Insert into _elements using rvalue move (avoids copying)
							_elements.insert(nit);

							// DEBUG CODE:
							// Print out the mapit entries for each object in _elements
							/*
							{
								for (auto et = _elements.begin(); et != _elements.end(); et++)
								{
									std::cout << "Element" << std::endl;
									for (auto st = (*et)->shape->shapeConstraints.begin(); st != (*et)->shape->shapeConstraints.end(); st++)
									{
										std::cout << "\t" << st->first << "\t" << *(st->second->begin()) << "\t" << st->second->units << std::endl;
									}
								}
							}
							*/
						}

						// Now, cit points to the leftmost iterator. Do recursive incrementation.
						// As the recursion progresses, cit will point to rightward iterators
						for (;;) 
						{
							(*cit)++;
							if ((*cit) == vmap[mit->first].end())
							{
								mit++;
								if (mit == mapit.end()) break; // mit at end also ends outer do-while loop
								cit = &(mit->second);
							} else {
								break;
							}
						} //while (*cit != vmap[mit->first].begin()); // cit reset when mit incremented. loop broken at break

					} while (mit != mapit.end());

				} // end of for (auto it = _gen.shapes.begin(); it != _gen.shapes.end(); it++)
			} // end of for (auto rt = _gen.rots.begin(); rt != _gen.rots.end(); rt++)


		} // end of _populate()


	}
}


