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
#include <boost/archive/xml_oarchive.hpp>
#include <boost/archive/xml_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/serialization/access.hpp>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/map.hpp>
#include <boost/serialization/set.hpp>
#include "ddpar.h"
#include "shapes.h"
#include "rotations.h"
#include "../common_templates.h"
#include "../coords.h"
#include "../units.h"
#include "../error/debug.h"
#include "../Public_Domain/MurmurHash3.h"
 
namespace rtmath {
	namespace ddscat {

		class ddParIteration;

		class ddParGeneratorBase
		{
		public:
			// TODO: have constructor fill in rest of vars
			ddParGeneratorBase();
			virtual ~ddParGeneratorBase();
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

		public:
			// TODO: make protected and add access functions
			ddPar base; // TODO: deprecated.
			std::string baseParFile;
			// The parameters that are variable
			//		Add tag for shape type // Shape
			std::string shapefilebase; // TODO: deprecated.
			std::set<std::shared_ptr<shapeModifiable> > shapeBase;



			// These don't go into a ddscat.par file
			
			bool compressResults, genIndivScripts, genMassScript;
			bool shapeStats, registerDatabase, doExport;
			std::string exportLoc;

			bool doTorques;
			std::string solnMeth, FFTsolver, Calpha, binning;

			size_t Imem1, Imem2, Imem3;
			bool doNearField;
			double near1, near2, near3, near4, near5, near6;
			double maxTol;
			size_t maxIter;
			double gamma;
			double etasca;
			double nambient;
			friend class ddParIteration;
			friend class boost::serialization::access;
			// When the class Archive corresponds to an output archive, the
			// & operator is defined similar to <<.  Likewise, when the class Archive
			// is a type of input archive the & operator is defined similar to >>.
                private:
			template<class Archive>
				void serialize(Archive & ar, const unsigned int version)
				{
					ar & BOOST_SERIALIZATION_NVP(name);
					ar & BOOST_SERIALIZATION_NVP(description);
					ar & BOOST_SERIALIZATION_NVP(outLocation);
					ar & BOOST_SERIALIZATION_NVP(ddscatVer);
					ar & BOOST_SERIALIZATION_NVP(strPreCmds);
					ar & BOOST_SERIALIZATION_NVP(strPostCdms);
					ar & BOOST_SERIALIZATION_NVP(baseParFile);
					ar & BOOST_SERIALIZATION_NVP(shapefilebase);

					ar & BOOST_SERIALIZATION_NVP(compressResults);
					ar & BOOST_SERIALIZATION_NVP(genIndivScripts);
					ar & BOOST_SERIALIZATION_NVP(genMassScript);
					ar & BOOST_SERIALIZATION_NVP(shapeStats);
					ar & BOOST_SERIALIZATION_NVP(registerDatabase);
					ar & BOOST_SERIALIZATION_NVP(doExport);
					ar & BOOST_SERIALIZATION_NVP(exportLoc);

					ar & BOOST_SERIALIZATION_NVP(doTorques);
					ar & BOOST_SERIALIZATION_NVP(solnMeth);
					ar & BOOST_SERIALIZATION_NVP(FFTsolver);
					ar & BOOST_SERIALIZATION_NVP(Calpha);
					ar & BOOST_SERIALIZATION_NVP(binning);
				
					ar & BOOST_SERIALIZATION_NVP(Imem1);
					ar & BOOST_SERIALIZATION_NVP(Imem2);
					ar & BOOST_SERIALIZATION_NVP(Imem3);
					ar & BOOST_SERIALIZATION_NVP(doNearField);
					ar & BOOST_SERIALIZATION_NVP(near1);
					ar & BOOST_SERIALIZATION_NVP(near2);
					ar & BOOST_SERIALIZATION_NVP(near3);
					ar & BOOST_SERIALIZATION_NVP(near4);
					ar & BOOST_SERIALIZATION_NVP(near5);
					ar & BOOST_SERIALIZATION_NVP(near6);

					ar & BOOST_SERIALIZATION_NVP(maxTol);
					ar & BOOST_SERIALIZATION_NVP(maxIter);
					ar & BOOST_SERIALIZATION_NVP(gamma);
					ar & BOOST_SERIALIZATION_NVP(etasca);
					ar & BOOST_SERIALIZATION_NVP(nambient);

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
			static void write(const ddParIterator &obj, const std::string &outfile);
                        static void read(ddParIterator &obj, const std::string &file);
			// TODO: add partial conversion function to ddpar
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
			friend class boost::serialization::access;
			// When the class Archive corresponds to an output archive, the
			// & operator is defined similar to <<.  Likewise, when the class Archive
			// is a type of input archive the & operator is defined similar to >>.
			template<class Archive>
				void serialize(Archive & ar, const unsigned int version)
				{
					ar & BOOST_SERIALIZATION_NVP(_params);
					ar & BOOST_SERIALIZATION_NVP(_rot);
					MARKFUNC();
					//ar & BOOST_SERIALIZATION_NVP();
				}
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

		class ddParGenerator;
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
			const std::set< std::pair< paramSet<double> , std::string > > &freqs, &temps;
			const std::set<rotations> &rots;
			const std::set< boost::tuple< paramSet<double>, MANIPULATED_QUANTITY::MANIPULATED_QUANTITY, std::string > > &sizes;
			const std::set<std::shared_ptr<shapeModifiable> > &shapesBase;

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
			static void write(const ddParGenerator &obj, const std::string &basename);
			void import(const std::string &ddparfilename);
			void import(const ddPar &base);
			void generate(const std::string &basedir) const;
			static void read(ddParGenerator &obj, const std::string &basename);
		public: // Static stuff here
			void setDefaultBase(const std::string &ddbasefilename);
			void setDefaultBase(const ddPar &base);
			friend class ddParIteration;
			friend class boost::serialization::access;
                private:
			template<class Archive>
				void serialize(Archive & ar, const unsigned int version)
				{
					ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(ddParGeneratorBase);
				}
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
			MARK();
			//throw rtmath::debug::xUnimplementedFunction();
			return &lhs < &rhs; // This is a horrible idea.......
			//if (lhs.f != rhs.f) return lhs.f < rhs.f;

			//return false;
		}
	};
}; // end namespace std


