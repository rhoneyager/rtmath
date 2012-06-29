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

		// Using a map because it provides easy access to the constraint var name,
		// really making it an indexed set
		typedef std::multimap< std::string, std::shared_ptr<shapeConstraint> > shapeConstraintContainer;

		class ddParGeneratorBase
		{
		public:
			ddParGeneratorBase();
			virtual ~ddParGeneratorBase();
			// freqs and temps are not part of the shape constraints
			std::set<rotations> rots;
			
			shapeConstraintContainer shapeConstraintsGlobal;
			// The shapes to generate
			std::set<std::unique_ptr<shapeModifiable> > shapes;

			ddPar base;
			std::string baseParFile;

			// These don't go into a ddscat.par file
			std::string name, description, outLocation;
			size_t ddscatVer;
			std::string strPreCmds;
			std::string strPostCdms;
			bool compressResults, genIndivScripts, genMassScript;
			bool shapeStats, registerDatabase, doExport;
			std::string exportLoc;

			// These do go into a ddscat.par file in some form or another

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
			bool doSca;
			std::set< boost::tuple<double,double,double,double> > scaDirs;


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

				ar & BOOST_SERIALIZATION_NVP(doSca);
				ar & BOOST_SERIALIZATION_NVP(scaDirs);

				ar & BOOST_SERIALIZATION_NVP(shapeConstraintsGlobal);
				ar & BOOST_SERIALIZATION_NVP(rots);
				ar & BOOST_SERIALIZATION_NVP(shapes);
			}
		};

		class ddParGenerator;

		class ddParIterator
		{
		public:
			// Constructor takes the pointer so we don't have to copy every conceivable property
			ddParIterator(const ddParGenerator &gen, std::unique_ptr<shapeModifiable> shp);
			static void write(const ddParIterator &obj, const std::string &outfile);
			static void read(ddParIterator &obj, const std::string &file);

			void exportShape(const std::string &filename) const;
			void exportDiel(const std::string &filename) const;
			void exportDDPAR(ddPar &out) const;
			void exportDDPAR(const std::string &filename) const;

			// The shape is a special clone of one of the ddParGenerator shapes, with no iteration over
			// values - only one element in each shapeConstraint entry.
			// contains freq, temp, reff, and every other needed quantity
			std::unique_ptr<shapeModifiable> shape;

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
		private:
			const ddParGenerator &_gen;
			// When the class Archive corresponds to an output archive, the
			// & operator is defined similar to <<.  Likewise, when the class Archive
			// is a type of input archive the & operator is defined similar to >>.
			template<class Archive>
			void serialize(Archive & ar, const unsigned int version)
			{
				ar & boost::serialization::make_nvp("ddParGenerator", _gen);
				ar & BOOST_SERIALIZATION_NVP(shape);
			}
		};

		inline std::size_t hash_value(ddParIterator const& x)
		{
			return (size_t) x.hash();
		}

		// This whole class exists just to encapsulate all of the conversion and iteration into a separate step
		class ddParIteration
		{
		public:
			ddParIteration(const ddParGenerator &gen);
			// Iterators go here
			typedef std::set<ddParIterator > data_t;
			typedef data_t::const_iterator const_iterator;
			inline const_iterator begin() const { return _elements.begin(); }
			inline const_iterator end() const { return _elements.end(); }
		private:
			data_t _elements;
			const ddParGenerator &_gen;
			void _populate();
			friend class boost::serialization::access;
			template<class Archive>
			void serialize(Archive & ar, const unsigned int version)
			{
				ar & boost::serialization::make_nvp("elements", _elements);
				ar & boost::serialization::make_nvp("ddParGenerator", _gen);
			}
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
			friend class ddParIterator;
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


