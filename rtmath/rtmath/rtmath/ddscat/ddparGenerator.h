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
#include "rotations.h"
#include "../coords.h"
#include "../units.h"

namespace rtmath {
	namespace ddscat {

		class ddParGeneratorBase
		{
		public:
			ddParGeneratorBase()
				:
					ddscatVer(72),
					_compressResults(false),
					_genIndivScripts(true),
					_genMassScript(true),
					_shapeStats(false),
					_registerDatabase(false)

					{ }
			virtual ~ddParGeneratorBase();
			std::set<units::hasUnits> freqs, temps;
			std::set<rotations> rots;
			std::string name, description, outLocation;
			size_t ddscatVer;
		protected:
			ddPar _base;
			// The parameters that are variable
			//		Add tag for shape type // Shape
			std::string _shapefilebase;

			// Can also vary scattering and rotation angle calculations
			// Be lazy and just specify as we would in ddscat for now...
			// TODO:



			// These don't go into a ddscat.par file
			
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
			void write(const std::string &basedir) const;
			void read(const std::string &basedir);
		};

	} // end ddscat
} // end rtmath



