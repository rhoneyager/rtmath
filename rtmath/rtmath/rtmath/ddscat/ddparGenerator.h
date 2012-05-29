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
#include <boost/tuple/tuple.hpp>
#include <boost/tuple/tuple_comparison.hpp>
#include "ddpar.h"
#include "shapes.h"
#include "rotations.h"
#include "../common_templates.h"
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
			//std::set<units::hasUnits> freqs, temps;
			std::set<rotations> rots;
			// Freqs and temps exist as pairs of values
			std::set<std::pair<std::string, std::string> > freqs, temps;
			//std::set<boost::tuple<
			std::string name, description, outLocation;
			size_t ddscatVer;
			std::string strPreCmds;
			std::string strPostCdms;

			void setShapeBase(std::shared_ptr<shapeModifiable> base);
			void getShapeBase(std::shared_ptr<shapeModifiable> &base);
		protected:
			ddPar _base;
			// The parameters that are variable
			//		Add tag for shape type // Shape
			std::string _shapefilebase;
			std::shared_ptr<shapeModifiable> _shapeBase;



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
			void write(const std::string &filename) const;
			void generate(const std::string &basedir) const;
			void read(const std::string &basedir);
		};

	} // end ddscat
} // end rtmath



