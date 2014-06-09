#pragma once
#include "../defs.h"
#include <string>
#include <vector>
#include <map>
#include <set>
#include <complex>
#include <boost/shared_ptr.hpp>
#include <boost/enable_shared_from_this.hpp>
//#include <boost/serialization/export.hpp>
//#include <boost/serialization/assume_abstract.hpp>
//#include <boost/serialization/version.hpp>

#include "../hash.h"
#include "../common_templates.h"
#include "../registry.h"
#include "../io.h"


namespace boost { namespace program_options { 
	class options_description; class variables_map; } }
namespace boost { namespace filesystem { class path; } }


namespace rtmath {
	namespace ddscat {
		class ddOutput;
		class ddOutput_IO_output_registry {};
		class ddOutput_IO_input_registry {};
		//class ddOutput_serialization {};
	}
	namespace registry {
		extern template struct IO_class_registry_writer<
			::rtmath::ddscat::ddOutput>;

		extern template class usesDLLregistry<
			::rtmath::ddscat::ddOutput_IO_output_registry,
			IO_class_registry_writer<::rtmath::ddscat::ddOutput> >;

		extern template struct IO_class_registry_reader<
			::rtmath::ddscat::ddOutput>;

		extern template class usesDLLregistry<
			::rtmath::ddscat::ddOutput_IO_input_registry,
			IO_class_registry_reader<::rtmath::ddscat::ddOutput> >;
		
	}
	namespace ddscat {

		class ddOriData;
		//class ddOutputGenerator;
		namespace shapefile
		{
			class shapefile;
		}
		namespace stats
		{
			class shapeFileStats;
		}
		class ddPar;
		//class ddOutputGeneratorConnector;

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
			::rtmath::registry::IO_class_registry_writer<::rtmath::ddscat::ddOutput> >,
			virtual public ::rtmath::io::implementsStandardWriter<ddOutput, ddOutput_IO_output_registry>,
			virtual public ::rtmath::registry::usesDLLregistry<
			::rtmath::ddscat::ddOutput_IO_input_registry,
			::rtmath::registry::IO_class_registry_reader<::rtmath::ddscat::ddOutput> >,
			virtual public ::rtmath::io::implementsStandardReader<ddOutput, ddOutput_IO_input_registry>
		{
			static void initPaths();
			void resize(size_t numOris, size_t numTotAngles);
			void resizeFML(size_t numTotAngles);
			void finalize();
		public:
			ddOutput();

			/// Regenerates ddOutputSingle entries from tables (used in hdf5 read)
			//void doImport();

			/// A brief description of the run
			std::string description;
			/// Frequency (GHz)
			double freq;
			/// Effective radius (um)
			double aeff;
			/// Temperature (K)
			double temp;

			/// Refractive indices (in order in ddscat.par file)
			std::vector<std::complex<double> > ms;
			/// Paths of source files. Used in consolidation.
			std::multiset<std::string> sources;
			/// User-set brief description snippets. Used in isolating sets of runs.
			std::multiset<std::string> tags;
			/// DDSCAT run version tag
			std::string ddvertag;

			/// Encapsulating enum in namespace, as an enum class is too restrictive
			class stat_entries {
			public:
				enum stat_entries_doubles
				{
					// D/AEFF
					D, XMIN, XMAX, YMIN, YMAX, ZMIN, ZMAX, AEFF, WAVE, FREQ,
					// K*AEFF
					NAMBIENT,
					TOL,
					TA1TFX, TA1TFY, TA1TFZ,
					TA2TFX, TA2TFY, TA2TFZ,
					TFKX, TFKY, TFKZ,
					IPV1TFXR, IPV1TFXI, IPV1TFYR, IPV1TFYI, IPV1TFZR, IPV1TFZI,
					IPV2TFXR, IPV2TFXI, IPV2TFYR, IPV2TFYI, IPV2TFZR, IPV2TFZI,
					TA1LFX, TA1LFY, TA1LFZ,
					TA2LFX, TA2LFY, TA2LFZ,
					LFKX, LFKY, LFKZ,
					IPV1LFXR, IPV1LFXI, IPV1LFYR, IPV1LFYI, IPV1LFZR, IPV1LFZI,
					IPV2LFXR, IPV2LFXI, IPV2LFYR, IPV2LFYI, IPV2LFZR, IPV2LFZI,
					BETA, THETA, PHI, ETASCA,
					QEXT1, QABS1, QSCA1, G11, G21, QBK1, QPHA1,
					QEXT2, QABS2, QSCA2, G12, G22, QBK2, QPHA2,
					QEXTM, QABSM, QSCAM, G1M, G2M, QBKM, QPHAM,
					QPOL, DQPHA,
					QSCAG11, QSCAG21, QSCAG31, ITER1, MXITER1, NSCA1,
					QSCAG12, QSCAG22, QSCAG32, ITER2, MXITER2, NSCA2,
					QSCAG1M, QSCAG2M, QSCAG3M,
					NUM_STAT_ENTRIES_DOUBLES
				};

				enum stat_entries_size_ts
				{
					// DOWEIGHT indicates whether this is an avg entry or a raw oriented result.
					VERSION, NUM_DIPOLES, NAVG, NUMP, NUMF, DOWEIGHT,
					NUM_STAT_ENTRIES_INTS
				};

				enum stat_entries_strings {
					TARGET,
					DDAMETH,
					CCGMETH,
					SHAPE,
					NUM_STAT_ENTRIES_STRINGS
				};
			};

			/// Table containing orientation data (cross-sections, etc.)
			/// Set when listing folder.
			Eigen::Matrix<double, Eigen::Dynamic, stat_entries::NUM_STAT_ENTRIES_DOUBLES> oridata_d;
			Eigen::Matrix<size_t, Eigen::Dynamic, stat_entries::NUM_STAT_ENTRIES_INTS> oridata_i;
			std::vector<std::array<std::string, stat_entries::NUM_STAT_ENTRIES_STRINGS> > oridata_s;

			//Eigen::Matrix<double, 1, stat_entries::NUM_STAT_ENTRIES_DOUBLES> > avgoridata_d;
			//Eigen::Matrix<size_t, 1, stat_entries::NUM_STAT_ENTRIES_INTS> > avgoridata_i;
			//std::array<std::string, stat_entries::stat_entries > avgoridata_s;


			/// Encapsulating enum in namespace, as an enum class is too restrictive
			class fmlColDefs
			{
				/// Table containing fml data
			public: enum fmlDefs
				{
					/// Match to a ORI index (TODO: use an integer)
					ORIINDEX,
					/// Scattering-angle specific
					THETAB, PHIB,
					F00R, F00I, F01R, F01I, F10R, F10I, F11R, F11I,
					NUM_FMLCOLDEFS
				};
			};
			/// Table containing fml data. Delayed allocation because the size resides within a file being read.
			boost::shared_ptr<Eigen::Matrix<float, Eigen::Dynamic, fmlColDefs::NUM_FMLCOLDEFS> > fmldata;
			

			/// Weights for the sca and fml files in the average.
			/// Sum of all of these should equal unity.
			//typedef std::map<boost::shared_ptr<ddOutputSingle>, float > weights;
			//static boost::shared_ptr<ddOutputSingle> genAvg(const weights&);
			
			/// Hash of shape file contents (an identifier)
			HASH_t shapeHash;
			/// The shape file (may load fully later)
			mutable boost::shared_ptr<const ::rtmath::ddscat::shapefile::shapefile> shape;
			/// Shape file statistics (may load fully later)
			mutable boost::shared_ptr<stats::shapeFileStats> stats;
			/// Load the full shape file and stats
			void loadShape() const;

			/// The ddscat parameter file
			boost::shared_ptr<ddPar> parfile;

			/// Pointer to any ensemble generator used to generate the avg results
			///
			/// Empty if ddOutput was directly loaded from a ddscat run.
			//boost::shared_ptr<ddOutputGenerator> generator;

			/**
			* \brief Generate a standardized file name (for saving) based on the 
			* ddOutput contents
			*
			* The name is based off of the shape hash, the frequency, temperature, 
			* the effective radius, and the ddscat version tag.
			**/
			std::string genName() const;

			/**
			* \brief Generate a standardized file name (for saving) based on the 
			* ddOutput contents
			*
			* The name is based off of the the frequency, temperature, the effective 
			* radius, and the ddscat version tag.
			**/
			std::string genNameSmall() const;

			/// Generate a ddOutputSingle .avg object that reflects the 
			/// sca file weights.
			/// \todo Finish implementing
			//void updateAVG();

			/// Expand output to a given directory
			void expand(const std::string &outdir, bool writeShape=false) const;


			/// Generate ddOutput from a ddscat output directory
			static boost::shared_ptr<ddOutput> generate(
				const std::string &dir, bool noLoadRots = false);

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

	}
}
