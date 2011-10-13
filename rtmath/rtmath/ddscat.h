#pragma once
#include <memory>
#include <string>
#include <vector>
#include <map>
#include <bitset>
#include <cstdio>
#include <cstring>
#include <complex>

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
			ddOutputMueller()
			{
				pol = 0;
				memset(Sij,0,16);
			}
			double pol;
			double Sij[16];
		};

		struct ddOutputScatt {
			std::complex<double> f11, f12, f21, f22;
		};

		struct ddCoords {
			ddCoords(double theta, double phi)
			{
				this->theta = theta;
				this->phi = phi;
			}
			double theta, phi;
		};

		struct ddOutComp
		{
			bool operator() (const ddCoords &lhs, const ddCoords &rhs) const
			{
				if (lhs.theta < rhs.theta) return true;
				if (lhs.theta > rhs.theta) return false; // i want strict weak ordering with theta first
				if (lhs.phi < rhs.phi) return true;
				return false;
			}
		};

		class ddOutputSingle {
		public:
			ddOutputSingle(double beta, double theta, double phi); // rotation angles
			inline void beta(double newbeta) { _beta = newbeta; }
			inline double beta() const { return _beta; }
			inline void theta(double newtheta) { _theta = newtheta; }
			inline double theta() const { return _theta; }
			inline void phi(double newphi) { _phi = newphi; }
			inline double phi() const { return _phi; }
			inline void wavelength(double newwavelength) { _wavelegth = newwavelength; }
			inline double wavelength() const { return _wavelegth; }
			inline void sizep(double newsizep) { _sizep = newsizep; }
			inline double sizep() const { return _sizep; }
			inline void reff(double newreff) { _reff = newreff; }
			inline double reff() const { return _reff; }
			inline void d(double newD) { _d = newD; }
			inline double d() const { return _d; }
			inline void numDipoles(double newDipoles) { _numDipoles = newDipoles; }
			inline size_t numDipoles() const { return _numDipoles; }

			void setF(const ddCoords &coords, const ddOutputScatt &f);
			void setS(const ddCoords &coords, const ddOutputMueller &s);
			void getF(const ddCoords &coords, ddOutputScatt &f) const;
			void getS(const ddCoords &coords, ddOutputMueller &s) const;
			void calcS(); // calculate S from f
		private:
			ddOutputSingle();
			void _init(); // sets sensible values for everything (hidden common init code)
			double _beta, _theta, _phi; // rotation angles
			double _wavelength;
			double _sizep, _reff; // TODO: set starting from here
			double _d;
			size_t _numDipoles;
			std::vector<std::complex<double> > _n;
			// Todo: get rid of duplication of elements
			// std::bitset<16> _ijs;
			std::map<ddCoords, ddOutputScatt, ddOutComp> _fijs;
			std::map<ddCoords, ddOutputMueller, ddOutComp> _Sijs;
		};

		class ddOutput {
		public:
			ddOutput();
			void writeEvans(std::string filename);
			//void writeCDF();
			//void readCDF();
			void readEvans(std::string filename);
			void readdir(std::string dirpath);
			void writedir(std::string dirpath);
			//void readfile();
			//void writefile();
			void genPf(daPf &target);
		private:
			bool _valid;
			size_t _numDipoles;
			double _daeff;
			double _d;
			double _xmin, _xmax, _ymin, _ymax, _zmin, _zmax;
			double _aEff;
			double _wavelength;
			double _kAeff;
		};

	}; // end namespace ddscat

}; // end namespace rtmath

