#pragma once
#include <memory>
#include <string>
#include <vector>
#include <map>
#include <bitset>
#include <cstdio>
#include <cstring>
#include <complex>
#include "matrixop.h"
#include "damatrix.h"
#include "enums.h"
#include "phaseFunc.h"
#include "cdf-ddscat.h"
#include "quadrature.h"

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

		class ddScattMatrix{
			// A complex matrix that holds the scattering amplitudes (2x2)
			// It may be converted into other forms
		public:
			// Standard constructor. theta and phi in degrees.
			// wavelength in microns
			ddScattMatrix(double theta, double phi, double wavelength);
			ddScattMatrix();
			std::complex<double> vals[2][2]; // Just for convenience
			mutable std::complex<double> S[4]; // also for convenience
			inline double theta() const {return _theta;}
			inline double phi() const {return _phi;}
			ddScattMatrix& operator=(const ddScattMatrix &rhs);
			ddScattMatrix operator+(const ddScattMatrix &rhs);
			ddScattMatrix& operator+=(const ddScattMatrix &rhs);
			bool operator==(const ddScattMatrix &rhs) const;
			bool operator!=(const ddScattMatrix &rhs) const;
			void print() const;
			void update();
			bool lock;
			double Pnn[4][4];
			double Knn[4][4];
		public: // Conversions start here
			void genS();
			void mueller(double Pnn[4][4]) const;
			void mueller(matrixop &res) const;
			inline matrixop mueller() const { matrixop res(2,4,4); mueller(res); return res; }
			void extinction(double Knn[4][4]) const;
			void extinction(matrixop &res) const;
			inline matrixop extinction() const { matrixop res(2,4,4); extinction(res); return res; }
		private:
			double _theta, _phi, _wavelength;
		};

		struct ddCoords {
			ddCoords(double theta, double phi)
			{
				this->theta = theta;
				this->phi = phi;
				alpha = theta;
			}
			double theta, phi, alpha;
		};

		struct ddCoords3 {
			ddCoords3(double beta, double theta, double phi)
			{
				this->beta = beta;
				this->theta = theta;
				this->phi = phi;
			}
			double theta, phi, beta;
		};

		struct ddCoordsComp
		{
			bool operator() (const ddCoords &lhs, const ddCoords &rhs) const
			{
				if (lhs.phi < rhs.phi) return true;
				if (lhs.phi > rhs.phi) return false; // i want strict weak ordering with theta first
				if (lhs.theta < rhs.theta) return true;
				return false;
			}
			bool operator() (const ddCoords3 &lhs, const ddCoords3 &rhs) const
			{
				if (lhs.beta < rhs.beta) return true;
				if (lhs.theta < rhs.theta) return true;
				if (lhs.theta > rhs.theta) return false; // i want strict weak ordering with theta first
				if (lhs.phi < rhs.phi) return true;
				return false;
			}
		};

		class ddOutputSingle // : public rtmath::phaseFunc
			: public rtmath::damatrix
		{
			// Class contains the output of a single ddscat file
			// This usually contains the results for a given Beta, Theta, Phi
			// And has the scattering amplitude matrix elements
			// It is for a given wavenumber, size parameter
			// effective radius and shape (also determining the number of
			// dipoles and a few other quantities).
			// Results are presented with varied theta and phi (note lower case)
		public:
			ddOutputSingle(double beta, double theta, double phi, double wavelength); // rotation angles
			ddOutputSingle(const std::string &filename) { _init(); loadFile(filename); }
			ddOutputSingle() { _init(); }
			void loadFile(const std::string &filename);
			void getF(const ddCoords &coords, ddScattMatrix &f) const;
			void setF(const ddCoords &coords, const ddScattMatrix &f);
			ddOutputSingle& operator=(const ddOutputSingle &rhs);
			void print() const;
			void genEmissionVectors();
			std::shared_ptr<matrixop> evalSInt(const mapid &valmap);
			// evaluate phase function at a given scattering angle:
			virtual std::shared_ptr<matrixop> eval(double alpha) const;
			virtual std::shared_ptr<matrixop> eval(const mapid &valmap) const;
			void writeCDFheader(cdfParams &params) const;
			void writeCDF(const std::string &filename) const;
			void size(std::set<double> &thetas, std::set<double> &phis) const;
		public:
			void _init();
			double _Beta, _Theta, _Phi;
			double _wavelength, _numDipoles, _reff;
			double _shape[3]; // for ellipsoids
			mutable std::map<ddCoords, ddScattMatrix, ddCoordsComp> _fs;
			mutable std::map<double, matrixop> _sigmas; // The emission matrices
		};

		class ddOutputEnsemble : public ddOutputSingle
		{
			// Class is a pure virtual class designed to do ensemble weighting
			// of a set of ddOutputSingle
		public:
			ddOutputEnsemble() {}
			std::map<ddCoords3, ddOutputSingle, ddCoordsComp> _ensemble;
			void generate(); // virtual based on weighting method
			virtual double weight(const ddCoords &coords, const ddCoords &delta) = 0;
			virtual bool needwtwt() {return true;}
			ddOutputSingle res;
		};

		class ddOutputEnsembleIso : public ddOutputEnsemble
		{
		public:
			ddOutputEnsembleIso() {}
			virtual double weight(const ddCoords &coords, const ddCoords &delta);
			virtual bool needwtwt() {return false;}
		};

		class ddOutputEnsembleGaussian : public ddOutputEnsemble
		{
		public:
			ddOutputEnsembleGaussian(double mu, double sigma)
			{
				this->mu = mu;
				this->sigma = sigma;
			}
			virtual double weight(const ddCoords &coords, const ddCoords &delta) = 0;
			double mu, sigma;
		};

		class ddOutputEnsembleGaussianPhi : public ddOutputEnsembleGaussian
		{
		public:
			ddOutputEnsembleGaussianPhi(double mu, double sigma) : 
			  ddOutputEnsembleGaussian(mu,sigma) {};
			virtual double weight(const ddCoords &coords, const ddCoords &delta);
			double mu, sigma;
		};

		class ddOutputEnsembleGaussianTheta : public ddOutputEnsembleGaussian
		{
		public:
			ddOutputEnsembleGaussianTheta(double mu, double sigma) : 
			  ddOutputEnsembleGaussian(mu,sigma) {};
			virtual double weight(const ddCoords &coords, const ddCoords &delta);
			double mu, sigma;
		};

		class ddOutput {
			// Class represents the output of a ddscat run
			// Can be loaded by specifying the path of a ddscat.par file
			// Otherwise, holds an array of all possible rotations that exist
			// and can compute the phase functions for any weighted combination

			// Can write Evans-formatted data
			// TODO: netcdf read/write, ddscat-formatted write
		public:
			ddOutput();
			ddOutput(const std::string &ddscatparFile) { _init(); loadFile(ddscatparFile); }
			void loadFile(const std::string &ddscatparFile);
			// writeEvans interpolates to the quadrature points and writes out the
			// mueller matrix, extinction matrix and emission matrices
			// It assumes that particles are aligned horizontally (but I take care of this)
			//void writeEvans(const std::string scatFile) const;
			void get(const ddCoords3 &coords, ddOutputSingle &f) const;
			void set(const ddCoords3 &coords, const ddOutputSingle &f);
		public:
			void _init();
			mutable std::map<ddCoords3, ddOutputSingle, ddCoordsComp> _data;
		};
	}; // end namespace ddscat

}; // end namespace rtmath

