#include "../rtmath/Stdafx.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <memory>
#include <string>
#include <vector>
#include <map>
#include <set>
#include <unordered_map>
#include <complex>
#include <boost/filesystem.hpp>
#include <cmath>
#include "../rtmath/matrixop.h"
#include "../rtmath/error/error.h"
#include "../rtmath/ddscat/ddOutput.h"
#include "../rtmath/ddscat/ddOutputSingle.h"
#include "../rtmath/ddscat/ddOutputEnsemble.h"

namespace rtmath {
	namespace ddscat {
		
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

	} // end ddscat
} // end rtmath
