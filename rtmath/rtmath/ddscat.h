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
			SOLUTIONMETHOD::SOLUTIONMETHOD _method;
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
			double Sij[4][4];
		};

		struct ddOutputScatt {
			std::complex<double> f[4];
		};

		struct ddRotCoords {
			ddRotCoords(double beta, double theta, double phi)
			{
				this->beta = beta;
				this->theta = teta;
				this->phi = phi;
			}
			double beta, theta, phi;
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

			bool operator() (const ddRotCoords &lhs, const ddRotCoords &rhs) const
			{
				if (lhs.beta < rhs.beta) return true;
				if (lhs.beta > rhs.beta) return false;
				if (lhs.theta < rhs.theta) return true;
				if (lhs.theta > rhs.theta) return false; // i want strict weak ordering with theta first
				if (lhs.phi < rhs.phi) return true;
				return false;
			}
		};

		class ddOutputSingle {
		public:
			ddOutputSingle(double beta, double theta, double phi); // rotation angles
			ddOutputSingle(const std::string &filename) { _init(); loadFile(filename); }
			void loadFile(const std::string &fileheader);
			inline void beta(double newbeta) { _beta = newbeta; }
			inline double beta() const { return _beta; }
			inline void theta(double newtheta) { _theta = newtheta; }
			inline double theta() const { return _theta; }
			inline void phi(double newphi) { _phi = newphi; }
			inline double phi() const { return _phi; }
			inline void wavelength(double newwavelength) { _wavelength = newwavelength; }
			inline double wavelength() const { return _wavelength; }
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
			//inline ddOutputScatt getF(const ddCoords &coords) const { ddOutputScatt res; getF(coords, res); return res; }
			void getS(const ddCoords &coords, ddOutputMueller &s) const;
			//inline ddOutputScatt getS(const ddCoords &coords) const { ddOutputMueller res; getS(coords, res); return res; }
			void calcS(); // calculate S from f
			std::vector<std::complex<double> > n;
		private:
			ddOutputSingle();
			void _init(); // sets sensible values for everything (hidden common init code)
			double _beta, _theta, _phi; // rotation angles
			double _wavelength;
			double _sizep, _reff; // TODO: set starting from here
			double _d;
			size_t _numDipoles;
			//std::vector<std::complex<double> > _n;
			// Todo: get rid of duplication of elements
			// std::bitset<16> _ijs;
			mutable std::map<ddCoords, ddOutputScatt, ddOutComp> _fijs;
			mutable std::map<ddCoords, ddOutputMueller, ddOutComp> _Sijs;
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
			//void genPf(daPf &target);
		private:
			bool _valid;
			size_t _numDipoles;
			double _daeff;
			double _d;
			double _xmin, _xmax, _ymin, _ymax, _zmin, _zmax;
			double _aEff;
			double _wavelength;
			double _kAeff;
			std::map<ddRotCoords, ddOutputSingle, ddOutComp> _data;
		public:
		};

	}; // end namespace ddscat

}; // end namespace rtmath

