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
#include <boost/shared_ptr.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <boost/weak_ptr.hpp>
#include <boost/scoped_ptr.hpp>
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
		class ddParIterator;

		

		class ddParGeneratorBase
		{
		public:
			ddParGeneratorBase();
			virtual ~ddParGeneratorBase();
			// freqs and temps are now part of the shape constraints
			std::set<boost::shared_ptr<rotations> > rots;
			// The other constraints
			shapeConstraintContainer shapeConstraintsGlobal;
			// The shapes to generate
			std::set<boost::shared_ptr<shapeModifiable> > shapes;

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

			friend class ddParIteration;
			friend class ddParIterator;
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

				// The par file is not serialized, as it is written separately
				//ar & boost::serialization::make_nvp("ddPar", base);

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
			ddParIterator(const ddParGenerator &gen, boost::shared_ptr<shapeModifiable> shp);
			static void write(const ddParIterator &obj, const std::string &outfile);
			static void read(ddParIterator &obj, const std::string &file);

			void exportShape(const std::string &filename) const;
			void exportDiel(const std::string &filename) const;
			void exportDDPAR(ddPar &out) const;
			void exportDDPAR(const std::string &filename) const;

			// The shape is a special clone of one of the ddParGenerator shapes, with no iteration over
			// values - only one element in each shapeConstraint entry.
			// contains freq, temp, reff, and every other needed quantity
			boost::shared_ptr<shapeModifiable> shape;
			boost::shared_ptr<rotations> rots;

			inline HASH_t hash() const
			{
				HASH_t res;
				HASH(this, sizeof(*this), HASHSEED, &res);
				return res;
			}

			friend class ddParIteration;
			friend class ddParGenerator;
			friend class boost::serialization::access;
		private:
			//const ddParGenerator &_gen;
			const ddParGenerator *_genp;
			ddParIterator() { _genp = nullptr; }
			// When the class Archive corresponds to an output archive, the
			// & operator is defined similar to <<.  Likewise, when the class Archive
			// is a type of input archive the & operator is defined similar to >>.
			template<class Archive>
			void serialize(Archive & ar, const unsigned int version)
			{
				ar & boost::serialization::make_nvp("ddParGenerator", _genp);
				ar & BOOST_SERIALIZATION_NVP(shape);
				ar & BOOST_SERIALIZATION_NVP(rots);
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
			static void write(const ddParIteration &obj, const std::string &outfile);
			static void read(ddParIteration &obj, const std::string &file);
			// Iterators go here
			typedef std::set< boost::shared_ptr<ddParIterator> > data_t;
			typedef data_t::const_iterator const_iterator;
			inline const_iterator begin() const { return _elements.begin(); }
			inline const_iterator end() const { return _elements.end(); }
		private:
			data_t _elements;
			//const ddParGenerator &_gen;
			const ddParGenerator *_genp;
			void _populate();
			friend class boost::serialization::access;
			template<class Archive>
			void serialize(Archive & ar, const unsigned int version)
			{
				ar & boost::serialization::make_nvp("elements", _elements);
				ar & boost::serialization::make_nvp("ddParGenerator", _genp);
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

}; // end namespace std


