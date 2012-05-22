#pragma once
/* The generator acts to construct a set of ddscat.par files and the associated 
 * mtabs based on a template and a set of varied parameters.
 * The parameter variation generates separate files, primarily because ddscat
 * doesn't have the necessary sophistication to deal with our common usage scenarios.
 */

#include <memory>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <map>
#include <set>
#include <vector>
#include <complex>
#include <boost/tokenizer.hpp>
#include "ddpar.h"
#include "shapes.h"

namespace rtmath {
	namespace ddscat {

		class ddParGeneratorBase
		{
		public:
			ddParGeneratorBase();
			virtual ~ddParGeneratorBase();
		protected:
			ddPar _base;
			// The parameters that are variable
			//		Add tag for shape type // Shape
			std::string _shapefilebase;
			std::set<double> _freqs;
			std::set<double> _temps;
			//std::set<double> _idsps; // Interdipole spacings
			//		Add tag for particle sizes and other aspects of shape parameters.
			//		These sizes can be specified in terms of effective radius, 
			//		max dimension, volume, mass or some other quantity. This 
			//		requires knowledge of the target shape.

			// Can also vary scattering and rotation angle calculations
			// Be lazy and just specify as we would in ddscat for now...
			// TODO:



			// These don't go into a ddscat.par file
			std::string _name, _description, _outLocation;
			size_t _ddscatVer;
			bool _compressResults, _genIndivScripts, _genMassScript;
			bool _shapeStats, _registerDatabase;
			std::string _exportLoc;
		};

		class ddParGenerator : public ddParGeneratorBase
		{
		public:
			ddParGenerator();
			ddParGenerator(const ddPar &base);
			virtual ~ddParGenerator();
			void write() const;
			void read(const std::string &basedir);
		};

	} // end ddscat
} // end rtmath



