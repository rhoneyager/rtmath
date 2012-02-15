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
#include "../../rtmath/rtmath/rtmath.h"

void doHelp();

int main(int argc, char** argv)
{
	using namespace std;
	using namespace rtmath;
	try {
		cerr << "rtmath-da" << endl;
		atexit(rtmath::debug::appExit);
		if (argc == 1) doHelp();
		config::parseParams p(argc,argv);
		string fAtmos;
		string fOutput;
		string fOverlay;
		set<double> freqs;
		bool verbose = true;
		bool dryrun = true;
		bool clobber = false;
		bool askClobber = true;
		vector<string> profpaths;
		shared_ptr<atmos::atmos> atmosBase;
		set<unique_ptr<atmos::atmos> > atmosOverlays;
		// Parse parameters and verify profile existence
		{
			bool flag = false;
			
			// Get rtmath config file path
			string rtconfpath;
			flag = p.readParam<string>("--config",rtconfpath);
			if (rtconfpath == "")
			{
				config::getConfigDefaultFile(rtconfpath);
			}
			cerr << "Loading config file: " << rtconfpath << endl;
			unique_ptr<config::configsegment> cRoot ( 
				config::configsegment::loadFile(rtconfpath.c_str(), NULL) );

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
				unique_ptr<atmos::atmos> o(new atmos::atmos(*it, *atmosBase ));
				atmosOverlays.insert( o.release() );
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

		// Execute program
		if (!dryrun)
		{
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
