#include "../rtmath/Stdafx.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <memory>
#include <algorithm>
#include <string>
#include <vector>
#include <map>
#include <set>
#include <unordered_map>
#include <complex>
#include <boost/bimap.hpp>
#include <boost/filesystem.hpp>
#include <boost/tokenizer.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/math/constants/constants.hpp>
#include <boost/serialization/map.hpp>
#include <boost/assign.hpp>
#include <cmath>
#include "../rtmath/refract.h"
#include "../rtmath/units.h"
#include "../rtmath/ddscat/shapes.h"
#include "../rtmath/ddscat/shapefile.h"
#include "../rtmath/error/error.h"
#include "../rtmath/ddscat/ddpar.h"
#include "../rtmath/ddscat/ddparGenerator.h"
#include "../rtmath/ddscat/runScripts.h"

		// In ellipsoid, 1/4 = (x/d*shp1)^2 + (y/d*shp2)^2 + (z/d*shp3)^2
		// In cylinder, shp1 = length/d, shp2 = diam/d,
		//		shp3 = 1 for a1 || x, 2 for a1|| y, 3 for a1 || z
		// In hex_prism, shp1 = length of prism / d == dist betw hex faces / d
		//				shp2 = dist betw opp vertices on one hex face / d = 2 * side length / d
		//				shp3 = 1 for a1 || x and a2 || y, ..... (see documentation)
		// shapeModifiable class specializations allow for shape param setting and provide other 
		// vars, like mean radius.		

namespace rtmath {
	namespace ddscat {

		boost::shared_ptr<shapeModifiable> shapeModifiable::promote() const
		{
			// Get CSHAPE variable. If none, then return just the 
			// pure shapeModifiable as a sign of failure.
			std::string shp;
			auto it = shapeConstraints.find("CSHAPE");
			if (it != shapeConstraints.end()) shp = it->second->units;
			
			if (shp == "FROM_FILE")
			{
				boost::shared_ptr<shapeModifiable> res(new rtmath::ddscat::shapes::from_file);
				res->shapeConstraints = shapeConstraints;
				return res;
			}
			GETOBJKEY();
			// Last resort
			{
				boost::shared_ptr<shapeModifiable> res(new shapeModifiable);
				res->shapeConstraints = shapeConstraints;
				return res;
			}
		}

		namespace shapes {

			from_ddscat::from_ddscat()
			{
			}

			from_ddscat::~from_ddscat()
			{
			}

			runScriptIndiv from_ddscat::prepRunScript(const std::string &name, const ddParGenerator &gen) const
			{
				runScriptIndiv iscript(name, gen);
				std::ostringstream cmds;
				cmds << "module load ddscat" << std::endl
					<< "ddscat" << std::endl;
				iscript.setRunCmds(cmds.str());

				iscript.addFile("ddscat.par");
				iscript.addFile("diel.tab");

				return iscript;
			}

			runScriptIndiv from_file::prepRunScript(const std::string &name, const ddParGenerator &gen) const
			{
				runScriptIndiv iscript = from_ddscat::prepRunScript(name, gen);
				iscript.addFile("shape.dat");

				return iscript;
			}

			void from_ddscat::exportDiel(const std::string &filename) const
			{
				using namespace std;

				// Figure out the frequency and temperature in GHz and K.
				complex<double> m;
				double val;
				string units;

				double fGHz;
				auto it = shapeConstraints.find("freq");
				if (it == shapeConstraints.end()) throw rtmath::debug::xBadInput("Need freq for diel.tab");
				val = *(it->second->pset.begin());
				units = it->second->units;
				units::conv_spec fconv(units, "GHz");
				fGHz = fconv.convert(val);

				double TK;
				it = shapeConstraints.find("temp");
				if (it == shapeConstraints.end()) throw rtmath::debug::xBadInput("Need temp for diel.tab");
				val = *(it->second->pset.begin());
				units = it->second->units;
				units::conv_temp tconv(units, "K");
				TK = tconv.convert(val);

				refract::mice(fGHz, TK, m);
				refract::writeDiel(filename,m);

				//GETOBJKEY();
				// TODO: add support for different refractive index calculators and 
				// for different materials (like iron)
			}

			void from_ddscat::exportDDPAR(ddPar &out) const
			{
				// Most of the ddPar file will be filled in from the base ddParGenerator class,
				// with a few overrides from the iterator. Some of the properties come from the 
				// shapeModifiable member, such as effective radius.

				// out is provided as a base (to make it work better with ddParGenerator)

				// Apply iterator-specific properties
				// Rotations
				TASSERT(_rots);
				_rots->out(out);

				// Also apply the shape properties to the ddPar output
				// This means: set CSHAPE, SHPAR1, SHPAR2, SHPAR3, AEFF
				// Wavelength
				// Here, set CSHAPE, SHPAR1, SHPAR2, SHPAR3, AEFF, Wavelength

				// CSHAPE - shape type. If not present, select FROM_FILE
				// This is a string, not a number, but it is hidden in shapeConstraints 
				// since it was easier to releverage the iteration
				auto it = shapeConstraints.find("CSHAPE");
				if (it != shapeConstraints.end()) 
				{
					std::string shp = it->second->units;
					out.setShape(shp);
				} else {
					throw rtmath::debug::xBadInput("Incomplete shape. No CSHAPE set!");
					//out.setShape("FROM_FILE");
				}

				// SHPAR1, SHPAR2, SHPAR3
				it = shapeConstraints.find("SHPAR1");
				if (it != shapeConstraints.end()) 
				{
					out.shpar(0,*(it->second->pset.begin()));
					it = shapeConstraints.find("SHPAR2");
					TASSERT(it != shapeConstraints.end());
					out.shpar(1,*(it->second->pset.begin()));
					it = shapeConstraints.find("SHPAR3");
					TASSERT(it != shapeConstraints.end());
					out.shpar(2,*(it->second->pset.begin()));
				} else {
					out.shpar(0,1.0);
					out.shpar(1,1.0);
					out.shpar(2,1.0);
				}

				// AEFF
				{
					it = shapeConstraints.find("aeff");
					if (it == shapeConstraints.end()) throw rtmath::debug::xBadInput("Need aeff for ddscat.par");
					std::string units;
					double val, aeff;
					units = it->second->units;
					val = *(it->second->pset.begin());
					// Convert
					rtmath::units::conv_alt cnv(units, "um");
					aeff = cnv.convert(val);

					out.setAeff(aeff,aeff,1,"LIN");
				}

				// Wavelength
				{
					it = shapeConstraints.find("aeff");
					if (it == shapeConstraints.end()) throw rtmath::debug::xBadInput("Need aeff for ddscat.par");
					std::string units;
					double val, wvlen;
					units = it->second->units;
					val = *(it->second->pset.begin());
					// Convert
					rtmath::units::conv_spec cnv(units, "um");
					wvlen = cnv.convert(val);

					out.setWavelengths(wvlen,wvlen,1,"LIN");
				}


				// And that's really it for the ddscat.par file...
			}

			void from_ddscat::exportDDPAR(const std::string &filename, const ddPar &ddbase) const
			{
				ddPar outpar = ddbase;
				exportDDPAR(outpar);
				outpar.writeFile(filename);
			}

			void from_ddscat::write(const std::string &base, const ddPar &ddbase) const
			{
				boost::filesystem::path pdir(base);

				exportDiel( (pdir/"diel.tab").string() );
				exportDDPAR( (pdir/"ddscat.par").string(), ddbase );
			}

			void from_file::write(const std::string &base, const ddPar &ddbase) const
			{
				from_ddscat::write(base, ddbase);
				// And also write shape.dat
				boost::filesystem::path p(base);
				exportShape( (p / "shape.dat").string() );
			}

			from_file::from_file()
			{
				_constructGraph();
			}

			from_file::~from_file()
			{
			}

			void from_file::_constructGraph(bool makegraph)
			{
				using namespace rtmath::graphs;
				//shapeModifiable::_constructGraph(false); // shapeModifiable constructor does this
				_createVertex("CSHAPE", true);
				_set("CSHAPE", 0, "FROM_FILE"); // Need to do it here since this has no dependencies
				GETOBJKEY(); // Add other properties to prevent throwing.
				// TODO: eventually allow for shape.dat manipulation, which would then use these properties.
				//_set("CSHAPE", 0, "FROM_FILE");
				if (makegraph)
					_graph = boost::shared_ptr<graph>(new graph(_vertices));
			}

			void from_file::run(const std::string &id)
			{
				/*
				if (id == "CSHAPE")
				{
					// Should be set in from_file::_constructGraph
					_set("CSHAPE", 0, "FROM_FILE");
				}
				*/
				shapeModifiable::run(id);
			}

			bool from_file::runSupported(const std::string &id)
			{
				if (id == "CSHAPE") return true;
				return shapeModifiable::runSupported(id);
			}

			void from_file::exportShape(const std::string &fname) const
			{
				// This is the shape.dat writing function. 
				// from_file should provide a shapefile class that can handle the writing

				//GETOBJKEY();
				// TODO: allow for crude shape manipulation!
				// Allow stretching, squeezing, changes of aspect ratio

				// fname is the output filename. source file is in shapeConstraints.
				// Get source filename
				using namespace std;
				string fSource;
				{
					auto it = shapeConstraints.find("source_filename");
					if (it == shapeConstraints.end())
						throw rtmath::debug::xBadInput("Missing source_filename for shape.dat write");
					fSource = it->second->units;
				}

				// Verify that file exists
				boost::filesystem::path p(fSource);
				if (!exists(p))
					throw rtmath::debug::xMissingFile(fSource.c_str());


				boost::filesystem::copy_file(p, boost::filesystem::path(fname));

				/*
				rtmath::ddscat::shapefile shp(fSource);

				// TODO: do any complex manipulation here

				shp.write(fname);
				*/
			}

			ellipsoid::ellipsoid()
			{
				_constructGraph();
			}

			ellipsoid::~ellipsoid()
			{
			}

			void ellipsoid::_constructGraph(bool makegraph)
			{
				using namespace std;
				using namespace rtmath::graphs;
				//shapeModifiable::_constructGraph(false); // shapeModifiable constructor does this
				_createVertex("CSHAPE", true);
				_set("CSHAPE", 0, "ELLIPSOID"); // Need to do it here since this has no dependencies

				// Need to add a few variables, like interdipole spacing and the three shape parameters
				
				const size_t varnames_size = 4;
				const std::string rawvarnames[varnames_size] = {
					"d",
					"shpar1",
					"shpar2",
					"shpar3"
				};
				std::set<std::string> varnames( rawvarnames, rawvarnames + varnames_size );

				// Create vertices for end variables
				for (auto it = varnames.begin(); it != varnames.end(); ++it)
					_createVertex(*it, true);

				// Create vertices representing function relationships
				// This is a table that lists the name of the new node, the variable being calculated, 
				// and the necessary dependencies. This is much cleaner than repeated copying / pasting, 
				// and is much easier to read.
				const size_t varmapnames_size = 27;
				const std::string rawvarmapnames[varmapnames_size] = {
					// name,					target,				dependencies
					"D_Ax__SHPAR1",				"shpar1",			"d,ax",
					"D_Ay__SHPAR2",				"shpar2",			"d,ay",
					"D_Az__SHPAR3",				"shpar3",			"d,az",
					"Ax_SHPAR1__D",				"d",				"ax,shpar1",
					"Ay_SHPAR2__D",				"d",				"ax,shpar2",
					"Az_SHPAR3__D",				"d",				"ax,shpar3",
					"D_SHPAR1__Ax",				"ax",				"d,shpar1",
					"D_SHPAR2__Ay",				"ay",				"d,shpar2",
					"D_SHPAR3__Az",				"az",				"d,shpar3"
				};

				for (size_t i=0; i< varmapnames_size; i = i + 3)
				{
					const string &name = rawvarmapnames[i];
					const string &starget = rawvarmapnames[i+1];
					const string &deps = rawvarmapnames[i+2];

					_createVertex(name, starget, deps);
				}
			



				if (makegraph)
					_graph = boost::shared_ptr<graph>(new graph(_vertices));
			}

			void ellipsoid::run(const std::string &id)
			{
				// In ellipsoid, 1/4 = (x/d*shp1)^2 + (y/d*shp2)^2 + (z/d*shp3)^2
				using namespace std;
				using namespace rtmath::units;
				// This is the trivial case of converter manipulation.
				// The appropriate vertex has its run() method called, and it 
				// eventually makes its way to this bit of code. This is the default
				// catch-all converter, designed to handle several possible basic tasks. 
				// It's all really just in the name of extensibility.
				const double pi = boost::math::constants::pi<double>();
				double raw;
				string units;

				// These are rather repetitive, and I'd rather avoid potential coding 
				// errors from the repitition. I may eventually implement a set of 
				// parallel pattern functions, but this will suffice for now.
				for (size_t i=1; i<=3; i++)
				{
					// Figure out var x, y, z
					const char vi = 'x' + i - 1;

					ostringstream svrad;
					svrad << "a" << vi; // ax, ay, az
					ostringstream sshpar;
					sshpar << "shpar" << i;
					string vrad = svrad.str();
					string shpar = sshpar.str();

					{
						ostringstream sidpattern;
						sidpattern << "D_A" << vi << "__SHPAR" << i;
						if (id == sidpattern.str())
						{
							_get(vrad,raw,units); // "ax", raw, units
							double a = conv_alt(units,"um").convert(raw);
							_get("d",raw,units);
							double d = conv_alt(units,"um").convert(raw);

							double sn = 4.*a/d;
							_set(shpar, sn, "um");
							return;
						}
					}
					{
						ostringstream sidpattern;
						sidpattern << "A" << vi << "_SHPAR" << i << "__D";
						if (id == sidpattern.str())
						{
							_get(vrad,raw,units); // "ax", raw, units
							double a = conv_alt(units,"um").convert(raw);
							_get(shpar,raw,units);
							double sn = conv_alt(units,"um").convert(raw);

							double d = 4.*a/sn;
							_set("d", d, "um");
							return;
						}
					}
					{
						ostringstream sidpattern;
						sidpattern << "D_SHPAR" << i << "__A" << vi;
						if (id == sidpattern.str())
						{
							_get("d",raw,units);
							double d = conv_alt(units,"um").convert(raw);
							_get(shpar,raw,units);
							double sn = conv_alt(units,"um").convert(raw);

							double a = d*sn/4.;
							_set(vrad, a, "um");
							return;
						}
					}
				}

				
				shapeModifiable::run(id);
			}

			bool ellipsoid::runSupported(const std::string &id)
			{
				if (id == "CSHAPE") return true;
				if (id == "D_Ax__SHPAR1") return true;
				if (id == "D_Ay__SHPAR2") return true;
				if (id == "D_Az__SHPAR3") return true;
				if (id == "Ax_SHPAR1__D") return true;
				if (id == "Ay_SHPAR2__D") return true;
				if (id == "Az_SHPAR3__D") return true;
				if (id == "D_SHPAR1__Ax") return true;
				if (id == "D_SHPAR2__Ay") return true;
				if (id == "D_SHPAR3__Az") return true;
				return shapeModifiable::runSupported(id);
			}
		}

	}
}


