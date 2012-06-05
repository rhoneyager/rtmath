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
#include <unordered_set>
#include <unordered_map>
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
 #include "../Public_Domain/MurmurHash3.h"
 
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
			// Freqs and temps exist as pairs of values (parameter set, units)
			std::set< std::pair< paramSet<double> , std::string > > freqs, temps;
			// Sizes exist as a tuple with the parameter set, actual variable and variable units
			// By actual variable, I mean effective radius, mass or volume. It's quite convenient this way.
			std::set< boost::tuple< paramSet<double>, MANIPULATED_QUANTITY::MANIPULATED_QUANTITY, std::string > > sizes;
			std::string name, description, outLocation;
			size_t ddscatVer;
			std::string strPreCmds;
			std::string strPostCdms;

		protected:
			ddPar _base;
			// The parameters that are variable
			//		Add tag for shape type // Shape
			std::string _shapefilebase;
			std::set<std::shared_ptr<shapeModifiable> > _shapeBase;



			// These don't go into a ddscat.par file
			
			bool _compressResults, _genIndivScripts, _genMassScript;
			bool _shapeStats, _registerDatabase;
			std::string _exportLoc;
		};

		class ddParIterator
		{
		public:
			ddParIterator();
			//double f() const; // Frequency, in GHz
			double T() const; // Temp, in K
			void getrots(rotations &out) const; // Export list of rotations
			std::shared_ptr<shapeModifiable> getshape() const; // Provide the shape that is to be manipulated
			void write(const std::string &outfile) const;
			template<class T>
				bool getParamValue(const std::string &name, T &val, std::string &units) const
				{
					if (_params.count(name))
					{
						std::ostringstream out;
						out << _params.at(name).first;
						std::istringstream in(out.str());
						in >> val;
						units = _params.at(name).second;
						return true;
					} else {
						val = 0;
						units = "";
						return false;
					}
				}
			inline HASH_t hash() const
			{
				HASH_t res;
				HASH(this, sizeof(*this), HASHSEED, &res);
				return res;
			}
			friend struct std::less<rtmath::ddscat::ddParIterator>;
			friend class ddParIteration;
			friend class ddParGenerator;
		private:
			// The initial parameters are passed in this form:
			// 1 - the name of the parameter
			// 2 - the value of the parameter
			// 3 - the units of the parameter
			// TODO: use a better method instead of a string key?
			std::map< std::string, std::pair< double, std::string > > _params;
			rotations _rot;
			std::shared_ptr<shapeModifiable> _shape; // Pulls from _params to set graph
		};

		inline std::size_t hash_value(ddParIterator const& x)
		{
			return (size_t) x.hash();
		}

		// This whole class exists just to encapsulate all of the conversion and iteration into a separate step
		class ddParIteration
		{
		public:
			ddParIteration(const ddParGenerator *src);
			// Iterators go here
			typedef std::unordered_set<ddParIterator, boost::hash<rtmath::ddscat::ddParIterator> > data_t;
			typedef data_t::const_iterator const_iterator;
			inline const_iterator begin() const { return _elements.begin(); }
			inline const_iterator end() const { return _elements.end(); }
		private:
			std::set< std::pair< paramSet<double> , std::string > > freqs, temps;
			std::set<rotations> rots;
			std::set< boost::tuple< paramSet<double>, MANIPULATED_QUANTITY::MANIPULATED_QUANTITY, std::string > > sizes;
			std::set<std::shared_ptr<shapeModifiable> > _shapesBase;

			// Use boost unordered stuff here
			//std::unordered_set<ddParIterator> _elements 
			data_t _elements;
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


namespace std {
	template <> struct hash<rtmath::ddscat::ddParIterator>
	{
		size_t operator()(const rtmath::ddscat::ddParIterator & x) const
		{
			// Really need to cast for the unordered map to work
			return (size_t) x.hash();
		}
	};


	template <> struct less<rtmath::ddscat::ddParIterator >
	{
		bool operator() (const rtmath::ddscat::ddParIterator &lhs, const rtmath::ddscat::ddParIterator &rhs) const
		{
			// Check f, mu, mun, phi, phin
			throw rtmath::debug::xUnimplementedFunction();
			//if (lhs.f != rhs.f) return lhs.f < rhs.f;
			//if (lhs.mu != rhs.mu) return lhs.mu < rhs.mu;
			//if (lhs.mun != rhs.mun) return lhs.mun < rhs.mun;
			//if (lhs.phi != rhs.phi) return lhs.phi < rhs.phi;
			//if (lhs.phin != rhs.phin) return lhs.phin < rhs.phin;

			return false;
		}
	};
}; // end namespace std


