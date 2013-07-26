#include "../rtmath/Stdafx.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <memory>
#include <string>
#include <vector>
#include <map>
#include <set>
#include <multiset>
#include <unordered_map>
#include <complex>
#include <boost/math/special_functions/erf.hpp>
#include <boost/math/distributions/normal.hpp>
#include "../rtmath/ddscat/ddOutputEnsemble.h"
#include "../rtmath/ddscat/ddOutputSingle.h"
#include "../rtmath/ddscat/ddOutput.h"
#include "../rtmath/ddscat/ddweights.h"
#include "../rtmath/error/error.h"

namespace rtmath {
	namespace ddscat {

		ddOutputEnsemble::ddOutputEnsemble(boost::shared_ptr<ddOutput> source) : src(source)
		{
			res = boost::shared_ptr<ddOutput>(new ddOutput());
			/// \todo Give ensemble class naming function
			res->description = "Ensemble output";
			res->tags = src->tags;
			res->tags.insert("Ensemble output");

			res->freq = src->freq;
			res->aeff = src->aeff;
			res->ms = src->ms;
			res->sources = src->sources;
			res->scas = src->scas;
			res->fmls = src->fmls;

			res->avg = boost::shared_ptr<ddOutputSingle>(new ddOutputSingle());
			
			res->stats = src->stats;
			res->shape = src->shape;
			res->shapeHash = src->shapeHash;
			res->parfile = src->parfile;
			res->generator = shared_from_this();
		}

		ddOutputEnsembleSimple::ddOutputEnsembleSimple(boost::shared_ptr<ddOutput> source) 
			: ddOutputEnsemble(source)
		{
			// Construct the avg file by simply averaging all sca outputs
			// This assumes that the sca files have the same scale for scattering output

			float numScas = (float) res->scas.size();
			for (auto &sca : res->scas)
			{
				res->weights.insert(std::pair<boost::shared_ptr<ddOutputSingle>, float>
					(sca, 1.0f / numScas));
			}

			/// \todo Construct the ddOutputSingle here
			throw debug::xUnimplementedFunction();
		}

		/*
		ddOutputEnsembleGaussian::ddOutputEnsembleGaussian(double sigma, size_t coord_varying)
		{
			_sigma = sigma;
			_coord_varying = coord_varying;
		}

		ddOutputEnsembleGaussian::~ddOutputEnsembleGaussian()
		{
		}

		void ddOutputEnsembleGaussian::genEnsemble(const std::unordered_map<coords::cyclic<double>, 
			std::shared_ptr<const ddscat::ddOutputSingle>, 
			boost::hash<coords::cyclic<double> > 
			> &_mapOutputSingleRaw, 
			ddOutputSingle &res) const
		{
			// Long definition for what is a simple unordered_map
			// Ensemble varies based on varying coordinate (beta, theta or phi)
			// ensemble should also reflect frequency boundaries.

			// So, split everything up based on frequency (coord. 0).
			// Handle each frequency separately, then recombine at the end.
			res.clear();

			using namespace std;
			map<double, multiset<double> > fmap; // histiograms of orientations along selected axis by frequency
			for (auto it = _mapOutputSingleRaw.begin(); it != _mapOutputSingleRaw.end(); ++it)
			{
				double f, crd;
				f = it->first.get(0);
				crd = it->first.get(_coord_varying);
				// If fmap does not already have this frequency, add it.
				if (fmap.count(f) == 0)
				{
					multiset<double> test;
					fmap[f] = test;
				}

				// Insert the coordinate as an element of fmap
				fmap[f].insert(crd);
			}



			// Now, iterate through the frequency containers and generate the weighting maps
			for (map<double, multiset<double> >::const_iterator it = fmap.begin(); it != fmap.end(); ++it)
			{
				// Generate the weights
				gaussianPosWeights gw(_sigma,it->second);
				std::map<double,double> weights;
				gw.getWeights(weights);

				// Collection of shared pointers that will be added to ddOutputSingle
				// After addition with _insert, they are treated as const!
				map<coords::cyclic<double>, shared_ptr<matrixop> > insmap;
				//unordered_map<coords::cyclic<double>, matrixop > insmap;

				// Iterate through _mapOutputSingleRaw and match up with the weights
				for (auto ot = _mapOutputSingleRaw.begin(); ot != _mapOutputSingleRaw.end(); ++ot)
				{
					// If the map's coordinates match (on frequency), then use this in the result
					if (ot->first.get(0) != it->first) continue; // Will catch at another (it) iteration
					double weight;
					weight = weights.at( ot->first.get(_coord_varying) );
					//if (weight > 0.01)
					//{
					//	cerr << "bingo\n";
					//}

					// Iterate through the ddScattMatrices
					for (auto pt = ot->second->_scattMatricesRaw.begin();
						pt != ot->second->_scattMatricesRaw.end(); ++pt)
					{
						matrixop m(2,2,4);
						(*pt)->getF(m); // Load this way
						m = m * weight; // Rescale for weight
						coords::cyclic<double> crds = (*pt)->genCoords();
						// If not in insmap, create a new entry. Otherwise, add to current entry.
						matrixop src(2,2,4);
						// Take entry and add m (already rescaled by weight) to it.
						// More complicated because the shared_ptr gets in the way
						if (insmap.count(crds))
						{
							src = *insmap[crds]; // Assignment operator
							insmap.erase(crds);
						}
						src = src + m;
						// Store (again) in insmap
						insmap[crds] = shared_ptr<matrixop>(new matrixop(src));
					}
					// All ddScattMatrices have been iterated through
				}
				// All ddOutputSingle have been iterated through. The appropriate weights have been added!
				// So, insmap is a complete, weighted map of coords and the 
				// corresponding scattering matrices in expanded matrixop form.
				
				// Create the ddScattMatrix entries of this frequency and insert into res
				for (auto ot = insmap.begin(); ot != insmap.end(); ++ot)
				{
					ddScattMatrix *nd = new ddScattMatrix(it->first,ot->first.get(1),ot->first.get(2));
					nd->setF(*(ot->second));
					// TODO: check assignment operator and copy constructor for ddscattmatrix
					shared_ptr<const ddScattMatrix> np(nd );
					res._insert(np);
				}
			}
			// All frequencies are done.
			// So the gaussian ensemble averaging is now complete!
		}

		ddOutputEnsembleIsotropic::ddOutputEnsembleIsotropic(size_t coord_varying)
		{
			_coord_varying = coord_varying;
		}

		ddOutputEnsembleIsotropic::~ddOutputEnsembleIsotropic()
		{
		}

		void ddOutputEnsembleIsotropic::genEnsemble(const std::unordered_map<coords::cyclic<double>, 
			std::shared_ptr<const ddscat::ddOutputSingle>, 
			boost::hash<coords::cyclic<double> > 
			> &_mapOutputSingleRaw, 
			ddOutputSingle &res) const
		{
			// Long definition for what is a simple unordered_map
			// Ensemble varies based on varying coordinate (beta, theta or phi)
			// ensemble should also reflect frequency boundaries.

			// So, split everything up based on frequency (coord. 0).
			// Handle each frequency separately, then recombine at the end.
			res.clear();

			using namespace std;
			map<double, multiset<double> > fmap; // histiograms of orientations along selected axis by frequency
			for (auto it = _mapOutputSingleRaw.begin(); it != _mapOutputSingleRaw.end(); ++it)
			{
				double f, crd;
				f = it->first.get(0);
				crd = it->first.get(_coord_varying);
				// If fmap does not already have this frequency, add it.
				if (fmap.count(f) == 0)
				{
					multiset<double> test;
					fmap[f] = test;
				}

				// Insert the coordinate as an element of fmap
				fmap[f].insert(crd);
			}



			// Now, iterate through the frequency containers and generate the weighting maps
			for (map<double, multiset<double> >::const_iterator it = fmap.begin(); it != fmap.end(); ++it)
			{
				// Generate the weights
				isoPosWeights iw(it->second);
				std::map<double,double> weights;
				iw.getWeights(weights);

				// Collection of shared pointers that will be added to ddOutputSingle
				// After addition with _insert, they are treated as const!
				map<coords::cyclic<double>, shared_ptr<matrixop> > insmap;

				// Iterate through _mapOutputSingleRaw and match up with the weights
				for (auto ot = _mapOutputSingleRaw.begin(); ot != _mapOutputSingleRaw.end(); ++ot)
				{
					// If the map's coordinates match (on frequency), then use this in the result
					if (ot->first.get(0) != it->first) continue; // Will catch at another (it) iteration
					double weight;
					weight = weights.at( ot->first.get(_coord_varying) );

					// Iterate through the ddScattMatrices
					for (auto pt = ot->second->_scattMatricesRaw.begin();
						pt != ot->second->_scattMatricesRaw.end(); ++pt)
					{
						matrixop m(2,2,4);
						(*pt)->getF(m); // Load this way
						m = m * weight; // Rescale for weight
						coords::cyclic<double> crds = (*pt)->genCoords();
						// If not in insmap, create a new entry. Otherwise, add to current entry.
						matrixop src(2,2,4);
						// Take entry and add m (already rescaled by weight) to it.
						// More complicated because the shared_ptr gets in the way
						if (insmap.count(crds))
						{
							src = *insmap[crds]; // Assignment operator
							insmap.erase(crds);
						}
						src = src + m;
						// Store (again) in insmap
						insmap[crds] = shared_ptr<matrixop>(new matrixop(src));
					}
					// All ddScattMatrices have been iterated through
				}
				// All ddOutputSingle have been iterated through. The appropriate weights have been added!
				// So, insmap is a complete, weighted map of coords and the 
				// corresponding scattering matrices in expanded matrixop form.
				
				// Create the ddScattMatrix entries of this frequency and insert into res
				for (auto ot = insmap.begin(); ot != insmap.end(); ++ot)
				{
					ddScattMatrix nd(it->first,ot->first.get(1),ot->first.get(2));
					nd.setF(*(ot->second));
					// TODO: check assignment operator and copy constructor for ddscattmatrix
					shared_ptr<const ddScattMatrix> np(new ddScattMatrix(nd) );
					res._insert(np);
				}
			}
			// All frequencies are done.
			// So the gaussian ensemble averaging is now complete!
		}

		ddOutputEnsembleAligned::ddOutputEnsembleAligned()
		{
		}

		ddOutputEnsembleAligned::~ddOutputEnsembleAligned()
		{
		}

		void ddOutputEnsembleAligned::genEnsemble(const std::unordered_map<coords::cyclic<double>, 
			std::shared_ptr<const ddscat::ddOutputSingle>, 
			boost::hash<coords::cyclic<double> > 
			> &_mapOutputSingleRaw, 
			ddOutputSingle &res) const
		{
			// Long definition for what is a simple unordered_map
			// Ensemble varies based on varying coordinate (beta, theta or phi)
			// ensemble should also reflect frequency boundaries.

			// So, split everything up based on frequency (coord. 0).
			// Handle each frequency separately, then recombine at the end.
			res.clear();

			using namespace std;
			if (_mapOutputSingleRaw.size() == 0) throw rtmath::debug::xBadInput("No members to generate ensemble with.");
			auto it = _mapOutputSingleRaw.begin();
			res = *it->second;

		}
		*/

	}
}
