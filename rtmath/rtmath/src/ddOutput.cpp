#include "Stdafx-ddscat.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <memory>
#include <string>
#include <vector>
#include <map>
#include <set>
#include <complex>
#include <boost/filesystem.hpp>
#include <cmath>
#include <Ryan_Serialization/serialization.h>

#include "../rtmath/ddscat/ddpar.h"
#include "../rtmath/ddscat/ddOutput.h"
#include "../rtmath/ddscat/ddOutputSingle.h"
#include "../rtmath/ddscat/ddOutputGenerator.h"
#include "../rtmath/hash.h"
#include "../rtmath/ddscat/shapefile.h"
#include "../rtmath/ddscat/shapestats.h"
#include "../rtmath/units.h"
#include "../rtmath/error/error.h"

namespace rtmath {
	namespace ddscat {
		ddOutput::ddOutput() : 
			freq(0), aeff(0)
		{
		}

		void ddOutput::loadShape()
		{
			// Load stats and shape based on the hash
			stats = shapeFileStats::loadHash(this->shapeHash);
			stats->load();
			shape = stats->_shp;
		}

		boost::shared_ptr<ddOutput> ddOutput::generate(const std::string &dir)
		{
			// Handling typical case with only one .avg output (one frequency, one aeff)
			boost::shared_ptr<ddOutput> res(new ddOutput());
			using namespace boost::filesystem;
			using std::vector;
			path pBase(dir);
			if (!exists(pBase)) throw debug::xMissingFile(dir.c_str());
			
			// Single level iteration through the path tree
			vector<path> cands;
			copy(directory_iterator(pBase), directory_iterator(), back_inserter(cands));
			for (const path& p : cands)
			{
				// Handle compressed files (in case my or Liu's scripts compressed the input)
				std::string uncompressed, meth;
				Ryan_Serialization::uncompressed_name(p.string(), uncompressed, meth);
				path praw(uncompressed);
				// Extract entension of files in ._ form
				// Note: some files (like mtable) have no extension. I don't use these.
				if (!praw.has_extension()) continue;
				path pext = praw.extension();
				// .avg, .sca and .fml are ddOutputSingle objects. Place them in appropriate places.
				// .out (target.out) and shape.dat refer to the shapefile. Only one needs to be loaded.
				// .par refers to the ddscat.par file. Useful in describing run inputs.
				// .log indicates a saved ddscat log file (implemented in my scripts). Useful for 
				// detecting the overall run time, but not necessary for now.
				if (pext.string() == ".avg" || pext.string() == ".sca" || pext.string() == ".fml")
				{
					boost::shared_ptr<ddOutputSingle> dds(new ddOutputSingle(p.string()));
					if (pext.string() == ".avg")
					{
						if (res->avg) throw debug::xBadInput("Simple ddOutput generator accepts only one avg file");
						res->avg = dds;
					}
					if (pext.string() == ".sca")
					{
						res->scas.insert(dds);
					}
					if (pext.string() == ".fml")
					{
						res->fmls.insert(dds);
					}
				} else if (pext.string() == ".par")
				{
					res->parfile = boost::shared_ptr<ddPar>(new ddPar(p.string()));
				} else if (pext.string() == ".dat" || pext.string() == ".out")
				{
					if (res->shape) continue; // Only needs to be loaded once
					// Note: the hashed object is the fundamental thing here that needs to be loaded
					// The other stuff is only loaded for processing, and is not serialized directly.
					res->shape = boost::shared_ptr<shapefile>(new shapefile(p.string()));
					// Get the hash and load the stats
					res->shapeHash = res->shape->hash();
					res->stats = shapeFileStats::genStats(res->shape);
				}
			}

			// Set a basic source descriptor
			res->sources.insert(dir);
			// Set a basic tag based on the avg file's TARGET line
			{
				std::string starget;
				res->avg->getTARGET(starget);
				res->tags.insert(starget);
			}

			// Set the frequency and effective radius
			res->freq = units::conv_spec("um","GHz").convert(res->avg->wave());
			res->aeff = res->avg->aeff();

			// Set generator to null generator
			// Nothing need be done

			// Set weights to ddscat standards.
			// Isotropic if there are an even number of thetas
			/// \todo Using simpson's rule weights if odd.
#pragma message("TODO: simpson's rule implementation")
			float numScas = (float) res->scas.size();
			float numThetas;
			{
				std::set<double> thetas;
				/// \todo Implement ddOutputSingle::numThetas function for avg file?
				for (auto &sca : res->scas)
				{
					if (!thetas.count(sca->theta())) thetas.insert(sca->theta());
				}
				numThetas = (float) thetas.size();
			}
			/// \todo Put this elsewhere, for the ddOutPutEnsembleDDSCAT function to also use
			for (auto &sca : res->scas)
			{
				float weight = 0;
				auto fApprox = [](double a, double b) -> bool
				{
					if (abs((a-b)/a) < 0.001) return true;
					return false;
				};
				if (fApprox(sca->theta(),0) || fApprox(sca->theta(),180))
				{
					// Simpson's rule case
					// int_a^b f(x)dx = (b-a)/6 * [f(a)+4f((a+b)/2)+f(b)]
					// Thetas are the _midpoints_ of the intervals, so the integration
					/** \note From Orient.f90
					!** Specify weight factors WGTA, WGTB
					!   (Note: weight function WGTA = 4*pi*P/(NTHETA*NPHI), where
					!    P=(probability/solid angle) of orientation in direction THETA,PHI .
					!    The orientational averaging program automatically samples uniformly
					!    in cos(theta) to allow for d(solid angle)=sin(theta)d(theta)d(phi).
					!   Present version assumes random orientations.
					!   When NTHETA is >1 and even, we use trapezoidal integration
					!   When NTHETA is >1 and odd, we use Simpson's rule integration.
					**/
					/// \todo Change this
					weight = 1.0f / numScas; // Isotropic case

				} else {
					weight = 1.0f / numScas; // Isotropic case
				}
				res->weights.insert(std::pair<boost::shared_ptr<ddOutputSingle>, float>
					(sca, weight));
			}

			// Populate ms
			/// \todo Allow for multiple refractive indices
			res->ms.push_back(res->avg->getM());

			// Have the stats add in all relevant rotations
			for (auto &sca : res->scas)
			{
				res->stats->calcStatsRot(sca->beta(), sca->theta(), sca->phi());
			}

			// Save the shape in the hash location, if necessary
			res->shape->writeToHash();
			// Resave the stats in the hash location
			res->stats->writeToHash();

			return res;
		}



		/*
		void ddOutput::clear()
		{
			_init();
			_outputSingleRaw.clear();
			_mapOutputSingleRaw.clear();
		}

		void ddOutput::insert(const std::shared_ptr<const ddscat::ddOutputSingle> &obj)
		{
			_outputSingleRaw.insert(obj);
			if (_mapOutputSingleRaw.count(obj->genCoords()) == 0)
				_mapOutputSingleRaw[obj->genCoords()] = obj;
		}

		void ddOutput::get(const coords::cyclic<double> &crds, 
			std::shared_ptr<const ddscat::ddOutputSingle> &obj,
			bool interpolate) const
		{
			if (_mapOutputSingleRaw.count(crds))
			{
				obj = _mapOutputSingleRaw.at(crds);
				return;
			}

			if (interpolate)
			{
				obj = nullptr;
				throw rtmath::debug::xUnimplementedFunction();
				return;
			}

			// Failure to get appropriate output
			obj = nullptr;
			// TODO: throw stuff?
			//throw rtmath::debug::xUnimplementedFunction();
			return;
		}

		void ddOutput::ensemble(const ddOutputEnsemble &provider, ddOutputSingle &res) const
		{
			// Will pass _mapOutputSingleRaw to the provider,
			// as it already has the coordinate mappings.
			// The ddOutputSingle entry will not refer to the originating ScattMatrices,
			// as this many references would be too complex. Instead, it will be standalone.
			provider.genEnsemble(_mapOutputSingleRaw,res);
		}
		*/
	}
}
