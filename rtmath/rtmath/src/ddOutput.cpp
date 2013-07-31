#include "../rtmath/Stdafx.h"
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
			// Set a basic tag?

			// Set the frequency and effective radius
			res->freq = units::conv_spec("um","GHz").convert(res->avg->wave());
			res->aeff = res->avg->aeff();

			// Set generator to null generator
			// Nothing need be done

			// Set isotropic weights
			float numScas = (float) res->scas.size();
			for (auto &sca : res->scas)
			{
				res->weights.insert(std::pair<boost::shared_ptr<ddOutputSingle>, float>
					(sca, 1.0f / numScas));
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
		ddOutput::ddOutput()
		{
			_init();
		}

		ddOutput::ddOutput(const std::string &ddparfile)
		{
			_init();
			loadFile(ddparfile);
		}

		void ddOutput::_init()
		{
			_filename = "";
			_shape = 0;
		}

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

		void ddOutput::freqs(std::set<double> &freq) const
		{
			freq.clear();
			// Iterate through ddOutputSingle to get set of frequencies
			for (auto it = _outputSingleRaw.begin(); it != _outputSingleRaw.end(); ++it)
			{
				double f = (*it)->freq();
				if (freq.count(f) == 0)
					freq.insert(f);
			}
		}

		void ddOutput::loadFile(const std::string &ddparfile)
		{
			using namespace std;
			using namespace boost::filesystem;

			boost::filesystem::path p(ddparfile.c_str()), dir, ddfile;
			if (!exists(p)) throw rtmath::debug::xMissingFile(ddparfile.c_str());
			if (is_regular_file(p))
			{
				// Need to extract the directory path
				ddfile = p;
				dir = p.parent_path();
			}
			if (is_directory(p))
			{
				// Need to extract the path for ddscat.par
				dir = p;
				ddfile = p / "ddscat.par";
			}
			if (!exists(ddfile))
			{
				throw rtmath::debug::xMissingFile(ddfile.string().c_str());
			}
			//cerr << "Directory: " << dir << endl;
			//cerr << "ddscat par: " << ddfile << endl;

			// Find and load the shapefile
			boost::filesystem::path pshapepath;
			string shapepath;
			{
				// Figure out where the shape file is located.
				path ptarget = p / "target.out";
				path pshapedat = p / "shape.dat";
				if (exists(ptarget))
				{ pshapepath = ptarget;
				} else if (exists(pshapedat))
				{ pshapepath = pshapedat;
				} else {
					throw rtmath::debug::xMissingFile("shape.dat or target.out");
				}
				shapepath = pshapepath.string();
				if (exists(pshapepath))
					_shape = shared_ptr<shapefile>(new shapefile(shapepath));
			}

			// Use boost to select and open all files in path
			// Iterate through each .fml file and load
			vector<path> files;
			copy(directory_iterator(dir), directory_iterator(), back_inserter(files));

			//cerr << "There are " << files.size() << " files in the directory." << endl;
			// Iterate through file list for .fml files
			vector<path>::const_iterator it;
			size_t counter = 0;
			for (it = files.begin(); it != files.end(); it++)
			{
				//cout << *it << "\t" << it->extension() << endl;
//				it->extension().str();
//				if (string(".fml").compare(it->extension().c_str()) == 0)
				if (it->extension().string() == string(".fml") )
				{
					// Load the file
					//cout << "Loading " << it->string() << endl;
					//ddOutputSingle news(it->string());
					std::shared_ptr<const ddscat::ddOutputSingle> news 
						(new ddscat::ddOutputSingle(it->string()));
					//coords::cyclic<double> crds = news->genCoords(); // not used
					this->insert(news);
					counter++;
				}
			}
			//cerr << "Of these, " << counter << " fml files were loaded." << endl;

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
