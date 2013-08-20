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
//#include "../matrixop.h"
//#include "../phaseFunc.h"
//#include "ddOutputEnsemble.h"
//#include "ddOutputSingle.h"
//#include "ddScattMatrix.h"
//#include "shapefile.h"

namespace rtmath {
	namespace ddscat {

		class ddOutputSingle;
		class ddOutputGenerator;
		class shapefile;
		class shapeFileStats;
		class ddPar;

		/** \brief Expresses the result of a ddscat run.
		 *
		 * This class can contain the complete output of a ddscat run. This includes 
		 * the shapefile, stats, sca, fml and avg files.
		 *
		 * This provides output for a single frequency, refractive index set and 
		 * effective radius. The most general type of ddscat run may contain many 
		 * permutations of these.
		 **/
		class DLEXPORT_rtmath_ddscat ddOutput
		{
			friend class ::boost::serialization::access;
			template<class Archive>
			void serialize(Archive & ar, const unsigned int version);
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

			/// The ensemble average results
			boost::shared_ptr<ddOutputSingle> avg;
			/// Raw sca inputs
			std::set<boost::shared_ptr<ddOutputSingle> > scas;
			/// Raw fml inputs
			std::set<boost::shared_ptr<ddOutputSingle> > fmls;

			/// Weights for the sca and fml files in the average.
			/// Sum of all of these should equal unity.
			std::map<boost::shared_ptr<ddOutputSingle>, float > weights;
			
			/// Hash of shape file contents (an identifier)
			HASH_t shapeHash;
			/// The shape file (may load fully later)
			mutable boost::shared_ptr<shapefile> shape;
			/// Shape file statistics (may load fully later)
			mutable boost::shared_ptr<shapeFileStats> stats;
			/// Load the full shape file and stats
			void loadShape();

			/// The ddscat parameter file
			boost::shared_ptr<ddPar> parfile;

			/// Pointer to any ensemble generator used to generate the avg results
			///
			/// Empty if ddOutput was directly loaded from a ddscat run.
			boost::shared_ptr<ddOutputGenerator> generator;

			/// Write output to file (using serialization)
			void writeFile(const std::string &filename) const;

			/// Read xml file (using serialization)
			void readFile(const std::string &filename);

			/// Generate a standardized file name (for saving) based on the 
			/// ddOutput contents. (TODO)
			std::string genName() const;

			/// Generate a ddOutputSingle .avg object that reflects the 
			/// sca file weights.
			/// \todo Finish implementing
			void updateAVG();

			/*
			/// Generate ddOutput from a set of ddOutputSingle
			static boost::shared_ptr<ddOutput> generate(
				boost::shared_ptr<ddOutputSingle> avg,
				boost::shared_ptr<shapefile> shape,
				std::set<boost::shared_ptr<ddOutputSingle> > sources);

			/// Generate ddOutput from a .avg file and a shape only
			static boost::shared_ptr<ddOutput> generate(
				boost::shared_ptr<ddOutputSingle> avg,
				boost::shared_ptr<shapefile> shape);
			*/

			/// Generate ddOutput from a ddscat output directory
			static boost::shared_ptr<ddOutput> generate(
				const std::string &dir);

			
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

	} // end ddscat
} // end rtmath


BOOST_CLASS_EXPORT_KEY(rtmath::ddscat::ddOutput);
