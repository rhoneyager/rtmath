#pragma once
#include <memory>
#include <string>
#include <vector>
#include <map>

// Needs extensive use of filesystem
// (for reading whole directories, manipulating paths, ...)
#include <boost/filesystem.hpp>

/* ddscat.h - Set of classes used for creating ddscat runs, loading ddscat output into model as a daPf,
 * converting ddscat output into the much older format used for comparison with Evas' rt4, and some
 * netcdf stuff.
 * Can write both ddscat.par files and provide a script that iterates through runs and executes them.
 */

namespace rtmath {

	namespace ddscat {

		namespace SOLUTIONMETHOD {


			enum SOLUTIONMETHOD {
				NONE
			};

		}; // end namespace solutionmethod

		namespace GEOMETRY {
			enum GEOMETRY {
				NONE,
				ELLIPSOID
			};
		};

		class ddRun {
		public:
			ddRun();
			~ddRun();
			std::string name;
			std::string description;
			//void writeFile(std::string filename);
			//void readFile(std::string filename);
			//void writeCDF();
			//void readCDF();
		private:
			double _memInit[3];
			GEOMETRY::GEOMETRY _geom;
			SOULTIONMETHOD::SOLUTIONMETHOD _method;
			double _shapeParams[3];
			size_t _ncomp;
			std::vector<std::string> _refrIndexFiles;
			double _toleranceErr;
			double _toleranceGamma;
			double _toleranceEta;
			double _wavelengths[4];
			double _Reff[4];
			double _incidPol[6];
			bool _iorth;
			bool _iwrksc;
			bool _iwrpol;
			double _rotBeta[3];
			double _rotTheta[3];
			double _rotPhi[3];
			double _iStart[3];
			std::vector<int> _Sijs;
			bool _frameTarget;
			std::vector< std::vector<double> > _scattPlanes;
			std::vector<std::string> _prelimOptions;
		};

		class ddRunSet {
			// Set of runs. Used for generating scripts for
			// propagation to many computers at once
		};

		struct ddOutputMueller {

		};

		class ddOutput {
			//void writeEvans(std::string filename);
			//void writeCDF();
			//void readCDF();
			//void readEvans();
			//void readdir();
			//void writedir();
			//void readfile();
			// void writefile();
			// void genPf(daPf &target);
		private:
			size_t _numDipoles;
			double _daeff;
			double _d;
			double _xmin, _xmax, _ymin, _ymax, _zmin, _zmax;
			double _aEff;
			double _wavelength;
			double _kAeff;
			std::vector<int> _Sijs;
			// ddOutputMueller
		};


	}; // end namespace ddscat

}; // end namespace rtmath

