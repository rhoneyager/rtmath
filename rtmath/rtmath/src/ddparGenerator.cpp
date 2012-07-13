#include "../rtmath/Stdafx.h"

#include "../rtmath/refract.h"
#include "../rtmath/ddscat/ddparGenerator.h"
#include "../rtmath/ddscat/runScripts.h"
#include "../rtmath/serialization.h"

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
			shapeConstraintsGlobal(___sbase)
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
				(*it)->exportDiel( (pdir/"diel.tab").string() );
				// Write shape.dat (if needed)
				(*it)->exportShape( (pdir/"shape.dat").string() );
				// Write ddscat.par
				(*it)->exportDDPAR( (pdir/"ddscat.par").string() );
				// Write run definitions
				rtmath::serialization::write<ddParIterator>(**it,(pdir).string());
				//ddParIterator::write(**it, (pdir).string() );
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

		void ddParGenerator::import(const std::string &ddparfilename)
		{
			using namespace boost::filesystem;
			path p(ddparfilename);
			if (!exists(ddparfilename)) throw rtmath::debug::xMissingFile(ddparfilename.c_str());
			rtmath::ddscat::ddPar ddfile(ddparfilename);
			base = ddfile;
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
			if (!_genp) return;
			const ddParGenerator &_gen = *_genp;

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
			outpar.writeFile(filename);
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
					shapeConstraintsEffective = _gen.shapeConstraintsGlobal;
					for (auto ot = (*it)->shapeConstraints.begin(); ot != (*it)->shapeConstraints.end(); ot++)
						shapeConstraintsEffective.insert(*ot);


					// first - parameter name. pair->first is value, pair->second is units
					multimap<const string, rtmath::units::hasUnits> constraintsVaried;
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
									constraintsVaried.insert(pair<const string,units::hasUnits>(name, units::hasUnits(val,units)));
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
						multimap<const string, rtmath::units::hasUnits > constraintsVariedUnique;
						multimap<const string, rtmath::units::hasUnits >::iterator dt;
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
					map<const string, set<rtmath::units::hasUnits > > vmap;
					for (auto ot = variedNames.begin(); ot != variedNames.end(); ot++)
					{
						//pair<multimap<string,pair<double,string> >::iterator, multimap<string,pair<double,string> >::iterator >
						auto ret = constraintsVaried.equal_range(*ot);
						for (auto ut = ret.first; ut != ret.second; ut++)
						{
							if (!vmap.count(*ot))
							{
								static const set<rtmath::units::hasUnits > base;
								vmap.insert(pair<string,set<rtmath::units::hasUnits > >(*ot,base));
							}
							vmap.at(*ot).insert(ut->second);
						}
					}
					
					// And now to permute everything.....
					// Set all iterators to begin
					// map<const string, set<rtmath::units::hasUnits > > vmap;
					map<string, set<rtmath::units::hasUnits >::iterator> mapit;
					for (auto ot = variedNames.begin(); ot != variedNames.end(); ot++)
						mapit[*ot] = vmap[*ot].begin();

					auto mit = mapit.begin();
					auto cit = &(mit->second);

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

								val = ot->second->quant();
								name = ot->first;
								units = ot->second->units();

								//std::cout << "\t" << name << "\t" << val << "\t" << units << std::endl;

								// Construct permuted object
								permuted.insert(pair<string,boost::shared_ptr<shapeConstraint> >(
									name, boost::make_shared<shapeConstraint>(name,boost::lexical_cast<std::string>(val),units) ) );
							}
							// As a reminder, (it) is an iterator to the current shape.....
							boost::shared_ptr<shapeModifiable> nshape( dynamic_cast<shapeModifiable*>((*it)->clone()) );
							nshape->shapeConstraints = permuted;
							nshape->update();

							// Make the iterator
							boost::shared_ptr<ddParIterator> nit(new ddParIterator(_gen, nshape ));
							// Add the rotations
							nit->rots = *rt;
							// Insert into _elenemts using rvalue move (avoids copying)
							_elements.insert(nit);
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



BOOST_CLASS_EXPORT_IMPLEMENT(rtmath::ddscat::ddParGeneratorBase)
//BOOST_CLASS_EXPORT_IMPLEMENT(rtmath::ddscat::ddParIterator)
//BOOST_CLASS_EXPORT_IMPLEMENT(rtmath::ddscat::ddParIteration)
BOOST_CLASS_EXPORT_IMPLEMENT(rtmath::ddscat::ddParGenerator)

