#pragma once
#include <memory>
#include <string>
#include <iostream>
#include <vector>
#include <map>
#include <set>
#include <unordered_map>
#include <complex>
#include "../matrixop.h"
#include "../interpolatable.h"
#include "../phaseFunc.h"
#include "ddScattMatrix.h"
#include "../da/daStatic.h"
#include "../coords.h"

namespace rtmath {
	
	namespace ddscat {

		class ddOutputSingle : public interpolation::ddOutputSingleInterp
		{
			// Class contains the output of a single ddscat file
			// Doesn't quite inherit from daStatic. The files loaded by the ddOutput class
			// contain information on the scattering and emission matrices, so they are logically
			// two separate entries.
			// Note: ensemble providers inherit from this!
		public:
			// Constructors need to completely initialize, as I have const ddOutputSingle
			// shared pointers.
			// Read from ddscat file
			ddOutputSingle(const std::string &filename);
			// Read from stream (Evans?/Other format?)

			
			virtual ~ddOutputSingle();

			void loadFile(const std::string &filename);
			// Stream input (Evans, CSV)
			void clear();

			void print(std::ostream &out = std::cerr) const;
			void writeCSV(const std::string &filename) const;
			void writeCSV(std::ostream &out = std::cerr) const;

			inline double beta() const {return _beta; }
			inline double theta() const {return _theta;}
			inline double phi() const {return _phi;}
			inline double freq() const {return _freq; }

			void genCoords(coords::cyclic<double> &res) const;
			inline coords::cyclic<double> genCoords() const { coords::cyclic<double> res; genCoords(res); return res; }

			// Generation functions
			// virtual std::shared_ptr<const daStatic> emissionVector(   ...   ) const;
			// Scattering and extinction providers for daStatic
			// virtual std::shared_ptr<const daStatic> P() const;
			// virtual std::shared_ptr<const daStatic> K() const;

		private:
			void _init();

		public: // Made public for now so ensembles work. May just make that a friend class.
			ddOutputSingle();
			virtual void _insert(std::shared_ptr<const ddscat::ddScattMatrix> &obj);
		protected:
			double _beta, _theta, _phi, _freq, _wavelength;
			double _reff;
			size_t _numDipoles;
			std::string _filename;
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

// ostream override
std::ostream & operator<<(std::ostream &stream, const rtmath::ddscat::ddOutputSingle &ob);
// istream override
//std::istream &operator>>(std::istream &stream, rtmath::ddscat::ddOutputSingle &ob);
