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

namespace rtmath {
	namespace ddscat {

		namespace shapes {

			from_file::from_file()
			{
			}

			from_file::~from_file()
			{
			}

			void from_file::_constructGraph()
			{
				shapeModifiable::_constructGraph();
			}

			void from_file::run(const std::string &id)
			{
				shapeModifiable::run(id);
			}

			bool from_file::runSupported(const std::string &id)
			{
				return shapeModifiable::runSupported(id);
			}

			void from_file::write(const std::string &fname) const
			{
				// This is the shape.dat writing function. 
				// from_file should provide a shapefile class that can handle the writing

				GETOBJKEY();
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

				rtmath::ddscat::shapefile shp(fSource);
				shp.write(fname);
			}

		}

	}
}


