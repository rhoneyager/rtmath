#include "../rtmath/Stdafx.h"
#include <memory>
#include <vector>
#include <map>
#include <iostream>
#include <fstream>
#include <sstream>
#include <locale>
#include <boost/filesystem.hpp>
#include <boost/tokenizer.hpp>
#include "../rtmath/atmos.h"
#include "../rtmath/absorb.h"
#include "../rtmath/error/error.h"
#include "../rtmath/config.h"
#include "../rtmath/command.h"
#include "../rtmath/ddscat/ddscat.h"

#include "../rtmath/ddscat/ddLoader.h"

namespace rtmath {

	ddLoader::ddLoader()
	{
		_init();
	}

	ddLoader::~ddLoader()
	{
	}

	void ddLoader::_init()
	{
		_P = nullptr;
		_K = nullptr;
		_EmV = nullptr;
	}

	std::unique_ptr<ddLoader> ddLoader::findLoader(const std::string &id, const std::string &prepend)
	{
		// First, parse id string and get first element
		typedef boost::tokenizer<boost::char_separator<char> >
			tokenizer;
		boost::char_separator<char> sep(":");
		std::vector<std::string> candIds;
		{
			std::vector<std::string> parsed;
			tokenizer tokens(id, sep);
			for (tokenizer::iterator it = tokens.begin();
				it != tokens.end(); ++it)
				parsed.push_back(*it);
			if (parsed.size() == 0) 
				throw rtmath::debug::xBadInput("Empty ddLoader::findLoader id.");
			// Make parsed[0] lower case and place in s.
			std::string s;
			if (parsed.size())
			{
				std::transform(parsed[0].begin(), parsed[0].end(), s.begin(), ::tolower);
				candIds.push_back(s);
			}
		}
		{
			std::vector<std::string> parsed;
			tokenizer tokens(prepend, sep);
			for (tokenizer::iterator it = tokens.begin();
				it != tokens.end(); ++it)
				parsed.push_back(*it);
			// Make parsed[0] lower case and place in s.
			std::string s;
			if (parsed.size())
			{
				std::transform(parsed[0].begin(), parsed[0].end(), s.begin(), ::tolower);
				candIds.push_back(s);
			}
		}


		std::unique_ptr<ddLoader> res = nullptr;
		// Go through all cases.
		// Only enabled entries here will be used in atmos load!
		for (auto it = candIds.begin(); it != candIds.end(); it++)
		{
			if (*it == "file")
			{
				//res = std::unique_ptr<ddLoader> (new ddLoaderFile(id, prepend));
				//return res;
			}
			if (*it == "mie")
			{
				res = std::unique_ptr<ddLoader> (new ddLoaderMie(id, prepend));
				return res;
			}
			if (*it == "rayleigh")
			{
			}
		}


		// No implemented match.
		std::string err = "Unimplemented loader id ";
		err.append(id);
		throw rtmath::debug::xBadInput(err.c_str());
		return nullptr;
	}

	ddLoaderFile::ddLoaderFile(const std::string &id, const std::string &subheader)
	{
		using namespace std;
		// Parse id field to get properties for file load
		typedef boost::tokenizer<boost::char_separator<char> >
			tokenizer;
		boost::char_separator<char> sep(":");
		string name;
		vector<string> props;
		// filename should be first non-'file' parameter
		{
			tokenizer tokens(id, sep);
			bool hFile = false;
			for (tokenizer::iterator it = tokens.begin();
				it != tokens.end(); ++it)
			{
				if (*it == "file") continue;
				if (*it == "") continue;
				if (!hFile)
				{
					name = *it;
					hFile = true;
				} else {
					props.push_back(*it);
				}
			}
			if (name == "")
				throw rtmath::debug::xBadInput(id.c_str());
		}
		// Actually load the file and produce the resultant damatrices

		// First, check for file / directory existence.
		// Locate file using the same method used to locate atmospheres, but
		// with the variation that the search directories are different.

		std::shared_ptr<config::configsegment> cRoot = rtmath::config::loadRtconfRoot();
		// Get common pf directory
		// May be overridden by app overwriting the key "pf/pfdir"
		// pfdirroot is the base search path. pfdir is the actual directory.
		string pfdir, ddpar;
		{
			string pfdirroot;
			cRoot->getVal("pf/pfdir", pfdirroot);
			rtmath::config::findFile psearch(pfdirroot, "");
			bool found = psearch.search(name,pfdir);
			if (!found) throw debug::xMissingFile(name.c_str());
			// Check for ddscat.par to make this a valid entry
			using namespace boost::filesystem;
			path pddpar = path(pfdir) / "ddscat.par";
			if (!exists(pddpar)) throw debug::xMissingFile(pddpar.string().c_str());
			ddpar = pddpar.string();
		}
		// Directory found to be pfdir. Now to load it.
		_ddset = shared_ptr<ddscat::ddOutput> (new ddscat::ddOutput(ddpar) );
		throw rtmath::debug::xUnimplementedFunction();
		// Call the routines to process the data
		// and include phase function orientation weighting.
		// TODO: pf orientation weighting may be specified with loader options.
		//       Look at subheader id data also.
		// Note: the ensembles can only be generated once a frequency is selected.
		// A special ensemble-generating container needs to be implemented.
		
		// Load pfs into interpolable damatrix entries (dafixed or dastatic?) for
		// scattering function, extinction and emission. 
		// TODO: Also see if f behaves linearly so that I only have to use it.
		
		// Issues to consider: interpolation along frequency axis also? Probably a bad idea.
		// Interpolation of copol/cxpol axes. Fully independent? Bilinear, bicubic, or other interp. method.
	}

	ddLoaderMie::ddLoaderMie(const std::string &id, const std::string &subheader)
	{
		throw rtmath::debug::xUnimplementedFunction();
		// Parse id field to get properties for mie sphere generation
		// This covers simple, one-size mie spheres (fixed radii,
		// size parameter varying with frequency).
		
		// No files need to be loaded for this method.
		// No orientational averaging needed either. Spherical particles!
		// Basic case only, so no size variation among particles!
		
		// Options include refractive index (do calculation otherwise), 
		// particle size (um)
		// The layer will privide other essentials, such as temperature.
		
		// Generate the damatrix handlers for the extinction and scattering matrices
		// Provide emission vector handlers.
	}

	ddLoaderFile::~ddLoaderFile()
	{
	}

	ddLoaderMie::~ddLoaderMie()
	{
	}

}; // end namespace rtmath

