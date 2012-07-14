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
				//shapeModifiable::_constructGraph(false);
				_createVertex("CSHAPE", true);
				_set("CSHAPE", 0, "FROM_FILE"); // Need to do it here since this has no dependencies
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

		}

	}
}


