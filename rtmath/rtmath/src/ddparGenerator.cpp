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
#include <boost/math/constants/constants.hpp>
#include <cmath>
#include "../rtmath/ddscat/ddparGenerator.h"

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

			// If a base shapefile exists and is not loaded, load it
			{
				throw rtmath::debug::xUnimplementedFunction();
			}

			// Now, iterate through the combinations of initial parameters
			for (auto f = freqs.begin(); f != freqs.end(); f++)
			{
				for (auto T = temps.begin(); T != temps.end(); T++)
				{
					// Add iteration over effective radius...
					throw rtmath::debug::xUnimplementedFunction();
					//for (auto sz = _; sz != _; sz++)
					{
						for (auto r = rots.begin(); r != rots.end(); r++)
						{
							// Take copy of shape and apply properties
							shared_ptr<shapeModifiable> nS( _shapeBase->clone() );
							using namespace MANIPULATED_QUANTITY;

							// Make copy of base
							ddPar parout = _base;

							units::conv_temp tconv(T->units(), "K");
							nS->set(TEMP, tconv.convert(T->quant()));

							units::conv_spec fconv(f->units(), "GHz");
							nS->set(FREQ, fconv.convert(f->quant()));
							// Convert to microns
							units::conv_spec umconv(f->units(), "um");
							double um = umconv.convert(f->quant()); // Wavelength in microns
							shared_ptr<ddParParsers::ddParLineMixed<double, std::string> > wvlens
								( new ddParParsers::ddParLineMixed<double, std::string>(3, ddParParsers::WAVELENGTHS));
							wvlens->set<double>(0,um);
							wvlens->set<double>(1,um);
							wvlens->set<double>(2,1.0);
							wvlens->set<std::string>(3,"LIN");
							parout.insertKey(ddParParsers::WAVELENGTHS,static_pointer_cast<ddParParsers::ddParLine>(wvlens));
							// That was far too much work...

							// Set the effective radii
							//	in shape file
							//	in ddscat.par also
						
							r->out(parout);

							throw rtmath::debug::xUnimplementedFunction();
						}
					}
				}
			}
		}

		void ddParGenerator::write(const std::string &basedir) const
		{
		}

		void ddParGenerator::read(const std::string &basedir)
		{
		}
	}
}



