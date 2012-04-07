#pragma once
#include <memory>
#include <string>
#include <vector>
#include <map>
#include <bitset>
#include <cstdio>
#include <cstring>
#include <complex>
#include "../matrixop.h"
#include "../phaseFunc.h"
#include "ddOutputEnsemble.h"
#include "ddOutputSingle.h"
#include "ddScattMatrix.h"
#include "shapefile.h"

namespace rtmath {
	namespace ddscat {

		class ddOutput {
			// Class represents the output of a ddscat run
			// Can be loaded by specifying the path of a ddscat.par file
			// Otherwise, holds an array of all possible rotations that exist
			// and can compute the phase functions for any weighted combination
		public:
			ddOutput();
			ddOutput(const std::string &ddparfile);
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

	} // end ddscat
} // end rtmath

