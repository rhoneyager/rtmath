#pragma once
#include "../defs.h"
#include <string>
#include <vector>
#include <map>
#include <set>
#include <complex>
#include <boost/shared_ptr.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <boost/serialization/export.hpp>
#include <boost/serialization/assume_abstract.hpp>
#include <boost/serialization/version.hpp>

#include "../hash.h"
#include "../registry.h"

namespace boost { namespace program_options { 
	class options_description; class variables_map; } }
namespace boost { namespace filesystem { class path; } }


namespace rtmath {
	namespace ddscat {
		class ddOutput;
		class ddOutput_IO_output_registry {};
	}
	namespace registry {
		extern template struct IO_class_registry<
			::rtmath::ddscat::ddOutput>;

		extern template class usesDLLregistry<
			::rtmath::ddscat::ddOutput_IO_output_registry,
			IO_class_registry<::rtmath::ddscat::ddOutput> >;
		
	}
	namespace ddscat {

		class ddOutputSingle;
		class ddOutputGenerator;
		namespace shapefile
		{
			class shapefile;
		}
		class shapeFileStats;
		class ddPar;
		class ddOutputGeneratorConnector;

		/** \brief Expresses the result of a ddscat run.
		 *
		 * This class can contain the complete output of a ddscat run. This includes 
		 * the shapefile, stats, sca, fml and avg files.
		 *
		 * This provides output for a single frequency, refractive index set and 
		 * effective radius. The most general type of ddscat run may contain many 
		 * permutations of these.
		 **/
		class DLEXPORT_rtmath_ddscat ddOutput :
			virtual public ::rtmath::registry::usesDLLregistry<
				::rtmath::ddscat::ddOutput_IO_output_registry, 
				::rtmath::registry::IO_class_registry<::rtmath::ddscat::ddOutput> >
		{
			friend class ::boost::serialization::access;
			template<class Archive>
			void serialize(Archive & ar, const unsigned int version);
			static void initPaths();
		public:
			ddOutput();

			/// A brief description of the run
			std::string description;
			/// Frequency (GHz)
			double freq;
			/// Effective radius (um)
			double aeff;
			/// Refractive indices (in order in ddscat.par file)
			std::vector<std::complex<double> > ms;
			/// Paths of source files. Used in consolidation.
			std::multiset<std::string> sources;
			/// User-set brief description snippets. Used in isolating sets of runs.
			std::multiset<std::string> tags;
			/// DDSCAT run version tag
			std::string ddvertag;

			/// The ensemble average results
			boost::shared_ptr<ddOutputSingle> avg;
			/// Original avg file, just in case
			boost::shared_ptr<ddOutputSingle> avg_original;

			/// Constituent sca inputs
			std::set<boost::shared_ptr<ddOutputSingle> > scas;
			/// Initial sca inputs before fml recalculation
			std::set<boost::shared_ptr<ddOutputSingle> > scas_original;
			/// Raw fml inputs
			std::set<boost::shared_ptr<ddOutputSingle> > fmls;

			/// Weights for the sca and fml files in the average.
			/// Sum of all of these should equal unity.
			std::map<boost::shared_ptr<ddOutputSingle>, float > weights;
			
			/// Hash of shape file contents (an identifier)
			HASH_t shapeHash;
			/// The shape file (may load fully later)
			mutable boost::shared_ptr<::rtmath::ddscat::shapefile::shapefile> shape;
			/// Shape file statistics (may load fully later)
			mutable boost::shared_ptr<shapeFileStats> stats;
			/// Load the full shape file and stats
			void loadShape() const;

			/// The ddscat parameter file
			boost::shared_ptr<ddPar> parfile;

			/// Pointer to any ensemble generator used to generate the avg results
			///
			/// Empty if ddOutput was directly loaded from a ddscat run.
			//boost::shared_ptr<ddOutputGenerator> generator;

			/// Write output to file
			void writeFile(const std::string &filename, const std::string &type = "") const;

			/// Read xml file (using serialization)
			void readFile(const std::string &filename);

			/**
			* \brief Generate a standardized file name (for saving) based on the 
			* ddOutput contents
			*
			* The name is based off of the shape hash, the frequency, the effective 
			* radius, and the ddscat version tag.
			**/
			std::string genName() const;

			/// Generate a ddOutputSingle .avg object that reflects the 
			/// sca file weights.
			/// \todo Finish implementing
			void updateAVG();

			/// Expand output to a given directory
			void expand(const std::string &outdir, bool writeShape=false) const;

			/*
			/// Generate ddOutput from a set of ddOutputSingle
			static boost::shared_ptr<ddOutput> generate(
				boost::shared_ptr<ddOutputSingle> avg,
				boost::shared_ptr<shapefile> shape,
				std::set<boost::shared_ptr<ddOutputSingle> > sources);
			*/

			/// Generate ddOutput from a .avg file, a .par file and a shape
			/// \todo Add a settings provider
			static boost::shared_ptr<ddOutput> generate(
				boost::shared_ptr<ddOutputSingle> avg,
				boost::shared_ptr<ddPar> par,
				boost::shared_ptr<::rtmath::ddscat::shapefile::shapefile> shape);
			

			/// Generate ddOutput from a ddscat output directory
			static boost::shared_ptr<ddOutput> generate(
				const std::string &dir, bool noLoadRots = false);

			/// Read xml file (using serialization)
			static boost::shared_ptr<ddOutput> load(const std::string &filename);

			/// Write run to the hash directory (convenience function)
			void writeToHash() const;

			/**
			 * \brief Adds ddOutput options to a program
			 *
			 * \item cmdline provides options only allowed on the command line
			 * \item config provides options available on the command line and in a config file
			 * \item hidden provides options allowed anywhere, but are not displayed to the user
			 **/
			static void add_options(
				boost::program_options::options_description &cmdline,
				boost::program_options::options_description &config,
				boost::program_options::options_description &hidden);
			/// Processes static options defined in add_options
			/// \todo Add processor for non-static options
			static void process_static_options(
				boost::program_options::variables_map &vm);
			/**
			 * \brief Retrieve the base hash paths
			 *
			 * \item pHashRuns is the base run hash directory
			 **/
			static void getHashPaths(
				boost::filesystem::path &pHashRuns);

			/// Load stats based on hash
			/// \throws rtmath::debug::xMissingFile if the hashed stats not found
			//static boost::shared_ptr<shapeFileStats> loadHash(
			//	const HASH_t &hash);
			/// Load stats based on hash
			/// \throws rtmath::debug::xMissingFile if the hashed stats not found
			//static boost::shared_ptr<shapeFileStats> loadHash(
			//	const std::string &hash);
		};

		/*
		class ddOutput {
			// Class represents the output of a ddscat run
			// Can be loaded by specifying the path of a ddscat.par file
			// Otherwise, holds an array of all possible rotations that exist
			// and can compute the phase functions for any weighted combination
		public:
			ddOutput() {}
			ddOutput(const std::string &ddparfile) {}
			void loadFile(const std::string &ddparfile);

			void insert(const std::shared_ptr<const ddscat::ddOutputSingle> &obj);
			void get(const coords::cyclic<double> &crds, 
				std::shared_ptr<const ddscat::ddOutputSingle> &obj,
				bool interpolate = false) const;
			void freqs(std::set<double> &freq) const;
			void clear();
			// Ensemble generation function
			// Uses ddOutputSingles as raw data, and takes the name of a provider
			// class for the type of ensemble weighting.
			// This is an alias for the appropriate ddOutputEnsemble function
			void ensemble(const ddOutputEnsemble &provider, ddOutputSingle &res) const;

		protected:
			// The ddOutputSingle raw data
			mutable std::set<std::shared_ptr<const ddscat::ddOutputSingle> > _outputSingleRaw;
			mutable std::unordered_map<
				coords::cyclic<double>, 
				std::shared_ptr<const ddscat::ddOutputSingle>, 
				boost::hash<coords::cyclic<double> > 
				> _mapOutputSingleRaw;
			std::string _filename;
			std::shared_ptr<shapefile> _shape;
		private:
			void _init();
		};
		*/

		/**
		 * \brief Provides an object that represents ddOutput import 
		 * settings.
		 *
		 * This class stores settings for importing, because 
		 * too many function overloads would result otherwise.
		 **/
		class DLEXPORT_rtmath_ddscat ddOutputGeneratorConnector
		{
			ddOutputGeneratorConnector();
			bool doStatCalc;
			bool shapeHash;
			bool statHash;
		public:
			static boost::shared_ptr<ddOutputGeneratorConnector> instance();
		};

	}
}


BOOST_CLASS_EXPORT_KEY(rtmath::ddscat::ddOutput);
