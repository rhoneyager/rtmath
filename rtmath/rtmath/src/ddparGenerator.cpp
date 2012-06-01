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

		void ddParGeneratorBase::getShapeBase(std::shared_ptr<shapeModifiable> &base)
		{
			base = _shapeBase;
		}

		void ddParGeneratorBase::setShapeBase(std::shared_ptr<shapeModifiable> base)
		{
			_shapeBase = base;
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
			// Check if basedir exists. If not, create it.
			using namespace boost::filesystem;
			using namespace std;
			path bpath(basedir);
			{
				if (!exists(bpath)) boost::filesystem::create_directory(bpath);
				if (exists(bpath))
					if (!is_directory(bpath))
						throw rtmath::debug::xPathExistsWrongType(bpath.string().c_str());
			}

			// Keep a list of the assigned subdirectories
			std::set<std::string> dirs;

			// If a base shapefile exists and is not loaded, load it
			if (!_shapeBase)
			{
				throw rtmath::debug::xUnimplementedFunction();
			}

			// Now, iterate through the combinations of initial parameters
			// This is rather ridiculous
			// TODO: construct a better iterator notation that unifies the disparate sets into hasUnits constructs!!!
			MARKFUNC();
			for (auto fs = freqs.begin(); fs != freqs.end(); fs++)
			{
				// I can take sets of frequencies, in range notation, all with different units
				std::string fUnits = fs->second;
				std::vector<double> fr;
				fs->first.getLong(fr);

				for (auto f = fr.begin(); f != fr.end(); f++)
				{
					for (auto Ts = temps.begin(); Ts != temps.end(); Ts++)
					{
						std::string TUnits = Ts->second;
						std::vector<double> Tr;
						Ts->first.getLong(Tr);
						for (auto T = Tr.begin(); T != Tr.end(); T++)
						{
							for (auto szs = sizes.begin(); szs != sizes.end(); szs++)
							{
								std::string szUnits = szs->get<2>();
								MANIPULATED_QUANTITY::MANIPULATED_QUANTITY stype = szs->get<1>();
								std::vector<double> szr;
								szs->get<0>().getLong(szr);

								for (auto sz = szr.begin(); sz != szr.end(); sz++)
								{
									// TODO: allow handling of multiple output shapes. Including from several 
									// base files and also including ddscat-internal shapes.
									// Also, handle additional graph vertices and relationship insertion.
									for (auto r = rots.begin(); r != rots.end(); r++)
									{
										// No subdir / duplicate run existence check, as the directories 
										// have a semirandom naming scheme.


										// Take copy of shape and apply properties
										shared_ptr<shapeModifiable> nS( (shapeModifiable*) _shapeBase->clone() );
										using namespace MANIPULATED_QUANTITY;

										// Make copy of base
										ddPar parout = _base;

										// Set rotations
										r->out(parout);

										// Set temperature
										units::conv_temp tconv(TUnits, "K");
										double TK = tconv.convert(*T);
										nS->set(TEMP, TK);
										_shapeBase->set(TEMP, TK);

										// Set frequency and wavelength
										units::conv_spec fconv(fUnits, "GHz");
										double fGHz = fconv.convert(*f);
										nS->set(FREQ, fGHz);
										// Convert to microns
										units::conv_spec umconv(fUnits, "um");
										double um = umconv.convert(*f); // Wavelength in microns
										shared_ptr<ddParParsers::ddParLineMixed<double, std::string> > wvlens
											( new ddParParsers::ddParLineMixed<double, std::string>(3, ddParParsers::WAVELENGTHS));
										wvlens->set<double>(0,um);
										wvlens->set<double>(1,um);
										wvlens->set<double>(2,1.0);
										wvlens->set<std::string>(3,"LIN");
										parout.insertKey(ddParParsers::WAVELENGTHS,static_pointer_cast<ddParParsers::ddParLine>(wvlens));
										// Set in shape file
										_shapeBase->set(FREQ, fGHz);

										// That was far too much work...
										// Calculate and set index of refraction (just for diel.tab. done in shapes.cpp already)
										// TODO: remove reduncancy
										std::complex<double> m;
										// TODO: support for different materials and 
										//		 refractive index functions
										refract::mice(fGHz, TK, m);
										//_shapeBase->set(IREFR_R, m.real());
										//_shapeBase->set(IREFR_IM, m.imag());

										// Set the effective radius
										//	in shape file
										//	in ddscat.par also
										// TODO: extend unit conversion to handle volumes and other stuff intelligently
										// TODO: implement other converters and in a better manner
										{
											MARKFUNC();
											double quant;
											if (stype == MASS)
											{
												units::conv_mass c(szUnits, "kg");
												quant = c.convert(*sz);
											} else if (stype == VOL)
											{
												units::conv_vol c(szUnits, "um^3");
												quant = c.convert(*sz);
											} else if (stype == REFF)
											{
												units::conv_alt c(szUnits, "um");
												quant = c.convert(*sz);
											}
											_shapeBase->set(stype, quant);
											
											// Update shape parameters based on quantities
											shapeModifiable::vertexMap mappings;
											_shapeBase->getVertices(mappings);
											// Select mappings
											rtmath::graphs::setWeakVertex known;
											known.insert(mappings.left.at(stype));
											known.insert(mappings.left.at(FREQ));
											known.insert(mappings.left.at(TEMP));

											// TODO: insert other known mappings here
											//throw rtmath::debug::xUnimplementedFunction();

											_shapeBase->update(known);
										}

										// Finish shape generation, then get calculated reff
										double reff = _shapeBase->get(REFF); // in um by default
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
										refract::writeDiel( (pdir/"diel.tab").string(), m );
										// Write shape.dat (if needed)

										// Write ddscat.par
										parout.saveFile( (pdir/"ddscat.par").string() );

										// Write run definitions
										// Give the output class access to the graph vertices.
										// TODO: write indiv run definition file class

										// Write run script
										runScriptIndiv iscript(dirname);
										iscript.write(dirname);

										// Lastly, add this path into the global run script listing...
										dirs.insert(dirname);
									}
								}
							}
						}
					}
				}
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



