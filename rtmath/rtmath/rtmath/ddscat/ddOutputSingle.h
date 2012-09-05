#pragma once
#include <string>
#include <iostream>
#include <vector>
#include <map>
#include <set>
#include <complex>
#include <boost/shared_ptr.hpp>
#include "../matrixop.h"
#include "../interpolatable.h"
#include "../phaseFunc.h"
#include "ddScattMatrix.h"
#include "shapefile.h"
#include "../da/daStatic.h"

namespace rtmath {
	
	namespace ddscat {

		class ddOutputSingle
		{
			// Class contains the output of a single ddscat file
			// Doesn't quite inherit from daStatic. The files loaded by the ddOutput class
			// contain information on the scattering and emission matrices, so they are logically
			// two separate entries.
			// Note: ensemble providers inherit from this!
		public:
			ddOutputSingle();
			virtual ~ddOutputSingle();

			// Direct reading and writing of ddscat-formatted files (avg, sca and fml)
			void readFile(const std::string &filename);
			void writeFile(const std::string &filename) const;

			void clear();
			/*
			void print(std::ostream &out = std::cerr) const;
			void write(const std::string &filename) const;
			void writeCSV(const std::string &filename) const;
			void writeCSV(std::ostream &out = std::cerr) const;
			void writeEvans(std::ostream &out, double freq) const;
			*/
			inline double beta() const {return _beta; }
			inline double theta() const {return _theta;}
			inline double phi() const {return _phi;}
			inline double freq() const {return _freq; }

			bool operator<(const ddOutputSingle &rhs) const;
		private:
			void _init();

		public: // Made public for now so ensembles work. May just make that a friend class.
			virtual void _insert(boost::shared_ptr<const ddscat::ddScattMatrix> &obj);
		protected:
			double _beta, _theta, _phi, _freq, _wavelength;
			double _reff;
			size_t _numDipoles;
			size_t _version;
			std::string _filename;
			boost::shared_ptr<shapefile> _shape;
			// Note the shared_ptr construction to save memory
			// Contains the raw scattering matrices
			//mutable std::set<std::shared_ptr<const ddscat::ddScattMatrix> > _scattMatricesRaw;
			// Contains the raw + interpolated scattering matrices
			// The interpolation method is set using other functions. Upon setting a method for
			// interpolation, this set is cleared and recalculated
			//mutable std::set<std::shared_ptr<const ddScattMatrix> > _scattMatricesAll;
			// A map is convenient to provide better support for interpolation.
			// This way, the interpolation functions / classes only have to deal with
			// ddCoords entries, without knowing the full type for ddScattMatrix or
			// any other types that interpolate.
			// This is just the mapping. Interpolation only knows the first part (coords::cyclic<double>)
			//mutable std::unordered_map<coords::cyclic<double>, std::shared_ptr<const ddScattMatrix>, 
			//	boost::hash<coords::cyclic<double> > > _interpMap;
		};

	}

}

