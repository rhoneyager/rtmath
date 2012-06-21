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
                        nambient(0)
                {
                    
                }
                
		ddParGenerator::ddParGenerator()
		{
			base = _s_defaultBase;
			import(base);
		}

		ddParGenerator::ddParGenerator(const ddPar &base)
		{
			this->base = base;
			import(base);
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
				ddPar parout = base;
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
						MARK();
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
				MARK();
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
				it->write( *it, (pdir).string() );

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
			ia >> BOOST_SERIALIZATION_NVP(obj);
		}

		void ddParGenerator::import(const std::string &ddparfilename)
		{
			using namespace boost::filesystem;
			path p(ddparfilename);
			if (!exists(ddparfilename)) throw rtmath::debug::xMissingFile(ddparfilename.c_str());
			rtmath::ddscat::ddPar ddfile(ddparfilename);
			base = ddfile;
			import(base);
		}

		void ddParGenerator::import(const ddPar &base)
		{
			using namespace std;
			using namespace rtmath::ddscat::ddParParsers;
			if (!base.size()) return;
			string sval;
			std::shared_ptr<const ddParLine> line;

			base.getKey(CMTORQ, line);
			static_pointer_cast<const ddParParsers::ddParLineSimple<string> >(line)->get(sval);
			(sval == "DOTORQ")? doTorques = true : doTorques = false;

			base.getKey(CMDSOL, line);
			static_pointer_cast<const ddParParsers::ddParLineSimple<string> >(line)->get(solnMeth);

			base.getKey(CMDFFT, line);
			static_pointer_cast<const ddParParsers::ddParLineSimple<string> >(line)->get(FFTsolver);

			base.getKey(CALPHA, line);
			static_pointer_cast<const ddParParsers::ddParLineSimple<string> >(line)->get(Calpha);

			base.getKey(CBINFLAG, line);
			static_pointer_cast<const ddParParsers::ddParLineSimple<string> >(line)->get(binning);

			base.getKey(DIMENSION, line);
			static_pointer_cast<const ddParParsers::ddParLineSimplePlural<size_t> >(line)->get(0, Imem1);
			static_pointer_cast<const ddParParsers::ddParLineSimplePlural<size_t> >(line)->get(1, Imem2);
			static_pointer_cast<const ddParParsers::ddParLineSimplePlural<size_t> >(line)->get(2, Imem3);

			base.getKey(NRFLD, line);
			size_t tsz;
			static_pointer_cast<const ddParParsers::ddParLineSimple<size_t> >(line)->get(tsz);
			(tsz > 0) ? doNearField = true : doNearField = false;
			base.getKey(FRACT_EXTENS, line);
			static_pointer_cast<const ddParParsers::ddParLineSimplePlural<double> >(line)->get(0, near1);
			static_pointer_cast<const ddParParsers::ddParLineSimplePlural<double> >(line)->get(1, near2);
			static_pointer_cast<const ddParParsers::ddParLineSimplePlural<double> >(line)->get(2, near3);
			static_pointer_cast<const ddParParsers::ddParLineSimplePlural<double> >(line)->get(3, near4);
			static_pointer_cast<const ddParParsers::ddParLineSimplePlural<double> >(line)->get(4, near5);
			static_pointer_cast<const ddParParsers::ddParLineSimplePlural<double> >(line)->get(5, near6);

			base.getKey(TOL, line);
			static_pointer_cast<const ddParParsers::ddParLineSimple<double> >(line)->get(maxTol);

			base.getKey(MXITER, line);
			static_pointer_cast<const ddParParsers::ddParLineSimple<size_t> >(line)->get(maxIter);

			base.getKey(GAMMA, line);
			static_pointer_cast<const ddParParsers::ddParLineSimple<double> >(line)->get(gamma);

			base.getKey(ETASCA, line);
			static_pointer_cast<const ddParParsers::ddParLineSimple<double> >(line)->get(etasca);

			base.getKey(CBINFLAG, line);
			static_pointer_cast<const ddParParsers::ddParLineSimple<double> >(line)->get(nambient);

			MARK();
			/* Things to import from ddPar:
			 shapefilebase
			 shape params
			 overall shape stuff
			 */
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

		void ddParIterator::getrots(rotations &out) const
		{
			out = _rot;
		}

		std::shared_ptr<shapeModifiable> ddParIterator::getshape() const
		{
			return _shape;
		}

		ddParIterator::ddParIterator()
		{
		}

		ddParIteration::ddParIteration(const ddParGenerator *src)
			:
				freqs(src->freqs),
				temps(src->temps),
				rots(src->rots),
				sizes(src->sizes),
				shapesBase(src->shapeBase)
		{
			using namespace std;
			// Iterate over rotations, freqs, temps, sizes, target types
			//throw rtmath::debug::xUnimplementedFunction();
			// Go through each combo and create a ddParIterator expressing each one

			// Iterate freqs first
			for (auto ft = freqs.begin(); ft != freqs.end(); ft++)
			{
				string fUnits = ft->second;
				for (auto ftb = ft->first.begin(); ftb != ft->first.end(); ftb++)
				{
					double f = *ftb;

					// Then iterate over rotations
					for (auto rt = rots.begin(); rt != rots.end(); rt++)
					{
						// And also iterate over shapes
						for (auto st = shapesBase.begin(); st != shapesBase.end(); st++)
						{
							// And iterate over temps
							for (auto tt = temps.begin(); tt != temps.end(); tt++)
							{
								string tUnits = tt->second;
								for (auto ttb = tt->first.begin(); ttb != tt->first.end(); ttb++)
								{
									double T = *ttb;

									// And lastly iterate over sizes
									for (auto ot = sizes.begin(); ot != sizes.end(); ot++)
									{
										string sUnits = ot->get<2>();
										MANIPULATED_QUANTITY::MANIPULATED_QUANTITY q = ot->get<1>();

										for (auto ott = ot->get<0>().begin(); ott != ot->get<0>().end(); ott++)
										{
											// And put all of this variation into a ddParIterator, somehow, ...
											ddParIterator nit;
											nit._shape = *st;
											nit._rot = *rt;
											MARK();
//											nit._params["temp"] = std::make_pair<double, string>(T, tUnits);
//											nit._params["freq"] = std::make_pair<double, string>(f, fUnits);
											MARK();
//											nit._params[rtmath::ddscat::MANIPULATED_QUANTITY::qnames[q]]
//												= std::make_pair<double, string>(*ott,sUnits); // q
										}
									}
								}
							}
						}
					}
				}
			}
		}

	}
}



