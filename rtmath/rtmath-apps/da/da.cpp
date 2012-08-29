/* rtmath-da - A program that loads an atmosphere and conducts the desired scattering 
 * calculations at the desired frequencies. Phase functions are provided in either 
 * the atmosphere or in a loadable overlay file or in a Liu-formatted hydrology file.
 * The main atmosphere is loaded using the atmos class.
 */

#include <iostream>
#include <cmath>
#include <fstream>
#include <set>
#include <map>
#include <string>
#include <sstream>
#include <vector>
#include <memory>
#include <algorithm>
#include <boost/filesystem.hpp>
#include <boost/tokenizer.hpp>
#include <boost/lexical_cast.hpp>
#include <stdlib.h>
#include "../../rtmath/rtmath/absorb.h"
#include "../../rtmath/rtmath/atmos.h"
#include "../../rtmath/rtmath/command.h"
#include "../../rtmath/rtmath/config.h"
#include "../../rtmath/rtmath/splitSet.h"

void doHelp();

int main(int argc, char** argv)
{
	using namespace std;
	using namespace rtmath;
	try {
		cerr << "rtmath-da" << endl;
		rtmath::debug::appEntry(argc, argv);
		if (argc == 1) doHelp();
		config::parseParams p(argc,argv);
		string fAtmos;
		string fOutput; // Output FOLDER
		string fOverlay;
		set<double> freqs;
		bool verbose = true;
		bool dryrun = true;
		bool clobber = false;
		bool askClobber = true;
		vector<string> profpaths;
		shared_ptr<atmos::atmos> atmosBase;
		set<shared_ptr<atmos::atmos> > atmosOverlays;
		// Parse parameters and verify profile existence
		{
			bool flag = false;
			
			// Get rtmath config file path
			string rtconfpath;
			flag = p.readParam<string>("--config",rtconfpath);
			cerr << "Loading config file." << endl;
			std::shared_ptr<config::configsegment> cRoot = config::loadRtconfRoot(rtconfpath);

			// Get frequency range
			vector<string> sFreqs;
			flag = p.readParam<string>("-f",sFreqs);
			if (!flag) doHelp();
			for (auto it = sFreqs.begin(); it != sFreqs.end(); it++)
				rtmath::config::splitSet<double>(*it,freqs);
			if (freqs.size() == 0) doHelp();

			// Get atmosphere
			string profdir, profext;
			// Get profile directory
			cRoot->getVal("atmos/atmosDir", profdir);
			if (profdir == "") profdir = "./";
			// If profile search directory is user-specified, override
			p.readParam<string>("--profdir",profdir);
			cerr << "Profile search directories: " << profdir << endl;
			// Get profile extensions
			cRoot->getVal("atmos/profileExtensions",profext);
			// Get profile name for calculations
			vector<string> profnames;
			flag = p.readParam<string>("--profile", profnames);
			if (!flag) doHelp();
			// Multiple profiles may be considered. If this is the case, then
			// one profile is the base profile and the others are different 
			// combinations of overlays. The first if the base!
			
			for (auto it = profnames.begin(); it != profnames.end(); ++it)
			{
				cerr << "Finding " << *it << endl;
				config::findFile psearch(profdir,profext);
				string profpath;
				bool found = psearch.search(*it, profpath);
				if (found)
					cerr << "Found in " << profpath << endl;
				else
					throw debug::xMissingFile(it->c_str());
				profpaths.push_back(profpath);
			}
		}

		// Load the atmospheres
		// Do base atmosphere first
		for (auto it = profpaths.begin(); it != profpaths.end(); ++it)
		{
			//vector<string> profpaths;
			//shared_ptr<atmos::atmos> atmosBase;
			//set<unique_ptr<atmos::atmos> > atmosOverlays;
			if (it == profpaths.begin())
			{
				// Base atmosphere
				shared_ptr<atmos::atmos> a(new atmos::atmos(*it));
				atmosBase = a;
			} else {
				// Overlay atmosphere
				// Atmos class is an overlay. Copy base, then override certain aspects
				shared_ptr<atmos::atmos> o(new atmos::atmos(*it, *atmosBase ));
				atmosOverlays.insert( o );
			}
		}

		// Write out options if verbose
		if (verbose)
		{
			cerr << "Will perform atmospheric doubling-adding calculations." << endl;
			{
				cerr << "Frequencies: ";
				for (auto it = freqs.begin(); it !=freqs.end(); ++it)
				{
					if (it != freqs.begin())
						cerr << ", ";
					cerr << *it;
				}
				cerr << " GHz" << endl;
			}
			cerr << "Using base profile: " << atmosBase->name << endl;
			if (atmosOverlays.size())
			{
				cerr << "With overlays:\n";
				for (auto it=atmosOverlays.begin(); it!=atmosOverlays.end(); ++it)
					cerr << "\t" << (*it)->name << endl;
			}
		}

		if (dryrun)
		{
			cerr << "This is a dry run. Stopping here.\n";
			exit(0);
		}



		// Execute program
		// I've loaded the atmosphere and the overlays. The overlays will have referenced the phase 
		// functions and the necessary files will have been loaded.
		// I just need to loop through the atmospheres and all selected frequencies.

		// Check for output folder existence
		// If it exists, ask to clobber. If yes, unlink the existing folder.

		// Create output folder

		// Write out basic information in description file

		// Create a new set containing the composite of the base atmosphere and any overlays.
		set<shared_ptr<atmos::atmos> > setAtmos = atmosOverlays; // Soooo convenient
		setAtmos.insert(atmosBase);

		// Then, establish loops over atmospheres
		for (auto it = setAtmos.begin(); it != setAtmos.end(); ++it)
		{
			// Loop over possible combinations of input phase functions
			{
				// Loop over frequencies
				for (auto f = freqs.begin(); f != freqs.end(); ++f)
				{
					// Do the da calculations
					//(*it)->

					// Write calculation results to file

					// Write out run statistics to filr
					// These statistics include the time taken to run,
					// the start date/time and end date/time, the optical
					// depths of the layers, the frequency, phase function
					// files referenced, etc.
				}
			}
		}
	}
	catch (rtmath::debug::xError &err)
	{
		err.Display();
		exit(1);
	}

	return 0;
}

void doHelp()
{
	exit(1);
}
