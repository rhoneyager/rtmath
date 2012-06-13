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
#include <boost/archive/xml_oarchive.hpp>
#include <boost/archive/xml_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/serialization/map.hpp>
#include <boost/serialization/set.hpp>
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
					_registerDatabase(false),
					_doExport(true)
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
			ddPar _base; // TODO: deprecated.
			std::string _baseParFile;
			// The parameters that are variable
			//		Add tag for shape type // Shape
			std::string _shapefilebase; // TODO: deprecated.
			std::set<std::shared_ptr<shapeModifiable> > _shapeBase;



			// These don't go into a ddscat.par file
			
			bool _compressResults, _genIndivScripts, _genMassScript;
			bool _shapeStats, _registerDatabase, _doExport;
			std::string _exportLoc;

			bool _doTorques;
			std::string _solnMeth, _FFTsolver, _Calpha, _binning;

			int _Imem1, _Imem2, _Imem3;
			bool _doNearField;
			double _near1, _near2, _near3, _near4, _near5, _near6;
			double _maxTol;
			int _maxIter;
			double _gamma;
			double _etasca;
			double _nambient;
			friend class boost::serialization::access;
			// When the class Archive corresponds to an output archive, the
			// & operator is defined similar to <<.  Likewise, when the class Archive
			// is a type of input archive the & operator is defined similar to >>.
			template<class Archive>
				void serialize(Archive & ar, const unsigned int version)
				{
					ar & BOOST_SERIALIZATION_NVP(name);
					ar & BOOST_SERIALIZATION_NVP(description);
					ar & BOOST_SERIALIZATION_NVP(outLocation);
					ar & BOOST_SERIALIZATION_NVP(ddscatVer);
					ar & BOOST_SERIALIZATION_NVP(strPreCmds);
					ar & BOOST_SERIALIZATION_NVP(strPostCdms);
					ar & BOOST_SERIALIZATION_NVP(_baseParFile);
					ar & BOOST_SERIALIZATION_NVP(_shapefilebase);

					ar & BOOST_SERIALIZATION_NVP(_compressResults);
					ar & BOOST_SERIALIZATION_NVP(_genIndivScripts);
					ar & BOOST_SERIALIZATION_NVP(_genMassScript);
					ar & BOOST_SERIALIZATION_NVP(_shapeStats);
					ar & BOOST_SERIALIZATION_NVP(_registerDatabase);
					ar & BOOST_SERIALIZATION_NVP(_doExport);
					ar & BOOST_SERIALIZATION_NVP(_exportLoc);

					ar & BOOST_SERIALIZATION_NVP(_doTorques);
					ar & BOOST_SERIALIZATION_NVP(_solnMeth);
					ar & BOOST_SERIALIZATION_NVP(_FFTsolver);
					ar & BOOST_SERIALIZATION_NVP(_Calpha);
					ar & BOOST_SERIALIZATION_NVP(_binning);
				
					ar & BOOST_SERIALIZATION_NVP(_Imem1);
					ar & BOOST_SERIALIZATION_NVP(_Imem2);
					ar & BOOST_SERIALIZATION_NVP(_Imem3);
					ar & BOOST_SERIALIZATION_NVP(_doNearField);
					ar & BOOST_SERIALIZATION_NVP(_near1);
					ar & BOOST_SERIALIZATION_NVP(_near2);
					ar & BOOST_SERIALIZATION_NVP(_near3);
					ar & BOOST_SERIALIZATION_NVP(_near4);
					ar & BOOST_SERIALIZATION_NVP(_near5);
					ar & BOOST_SERIALIZATION_NVP(_near6);

					ar & BOOST_SERIALIZATION_NVP(_maxTol);
					ar & BOOST_SERIALIZATION_NVP(_maxIter);
					ar & BOOST_SERIALIZATION_NVP(_gamma);
					ar & BOOST_SERIALIZATION_NVP(_etasca);
					ar & BOOST_SERIALIZATION_NVP(_nambient);

					ar & BOOST_SERIALIZATION_NVP(freqs);
					ar & BOOST_SERIALIZATION_NVP(temps);
					ar & BOOST_SERIALIZATION_NVP(sizes);
					ar & BOOST_SERIALIZATION_NVP(rots);
				}
		};

		//BOOST_CLASS_TRACKING(ddParGeneratorBase, track_always);

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
			void write(const std::string &basename) const;
			void import(const std::string &ddparfilename);
			void import(const ddPar &base);
			void generate(const std::string &basedir) const;
			void read(const std::string &basename);
		public: // Static stuff here
			void setDefaultBase(const std::string &ddbasefilename);
			void setDefaultBase(const ddPar &base);
			friend class boost::serialization::access;
			template<class Archive>
				void serialize(Archive & ar, const unsigned int version)
				{
					// serialize base class information
					//ar & BOOST_SERIALIZATION_NVP(
					//	boost::serialization::base_object<ddParGeneratorBase>(*this)
					//	);
					ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(ddParGeneratorBase);
				}
		private:
			static ddPar _s_defaultBase;
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


