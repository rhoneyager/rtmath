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

		class scattMatrix{
			// A complex matrix that holds the scattering amplitudes (2x2)
			// It may be converted into other forms
		public:
			// Standard constructor. theta and phi in degrees.
			scattMatrix(double theta, double phi);
			scattMatrix();
			std::complex<double> vals[2][2]; // Just for convenience
			inline double theta() const {return _theta;}
			inline double phi() const {return _phi;}
			scattMatrix& operator=(const scattMatrix &rhs);
			bool operator==(const scattMatrix &rhs) const;
			bool operator!=(const scattMatrix &rhs) const;
			void print() const;
		private:
			double _theta, _phi;
		};

		struct ddCoords {
			ddCoords(double theta, double phi)
			{
				this->theta = theta;
				this->phi = phi;
			}
			double theta, phi;
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
				if (lhs.theta < rhs.theta) return true;
				if (lhs.theta > rhs.theta) return false; // i want strict weak ordering with theta first
				if (lhs.phi < rhs.phi) return true;
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

		class ddOutputSingle {
			// Class contains the output of a single ddscat file
			// This usually contains the results for a given Beta, Theta, Phi
			// And has the scattering amplitude matrix elements
			// It is for a given wavenumber, size parameter
			// effective radius and shape (also determining the number of
			// dipoles and a few other quantities).
			// Results are presented with varied theta and phi (note lower case)
		public:
			ddOutputSingle(double beta, double theta, double phi); // rotation angles
			ddOutputSingle(const std::string &filename) { _init(); loadFile(filename); }
			ddOutputSingle() { _init(); }
			void loadFile(const std::string &filename);
			void getF(const ddCoords &coords, scattMatrix &f) const;
			void setF(const ddCoords &coords, const scattMatrix &f);
			ddOutputSingle& operator=(const ddOutputSingle &rhs);
			void print() const;
		public:
			void _init();
			double _Beta, _Theta, _Phi;
			double _wavelength, _numDipoles, _reff;
			double _shape[3]; // for ellipsoids
			mutable std::map<ddCoords, scattMatrix, ddCoordsComp> _fs;
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

