// transmittance is a program that calculates transmittance through the atmosphere
// for a given atmospheric profile. It can either use Liu's formulas in the
// calculation, valid for the entire atmosphere only, or it can do a line-by-line
// approach, using the entire HITRAN spectral line database (HITRAN 2008)
// This program outputs data in ascii csv format for ease of loading into other 
// programs. netCDF output is also planned. The purpose of this program is to 
// verify that my lbl code works correctly, and to verify that my configuration-
// handling routines work correctly (they are what gives the path to HITRAN)

#include <cmath>
#include <memory>
#include <iostream>
#include <fstream>
#include <string>
#include <set>
#include <boost/filesystem.hpp>
#include <boost/tokenizer.hpp>
#include "../../rtmath/rtmath/rtmath.h"

void doHelp();

int main(int argc, char** argv)
{
	using namespace std;
	using namespace rtmath;
	try {
		// Process some of the flags
		atexit(rtmath::debug::appExit);
		if (argc == 1) doHelp();
		config::parseParams p(argc, argv);
		cerr << "rtmath-transmittance\n\n";
		bool flag;

		// Get master configuration file path
		// Used to find out where HITRAN is and where the profile paths are
		string rtconfpath;
		flag = p.readParam<string>("--config", rtconfpath);
		if (rtconfpath == "")
		{
			config::getConfigDefaultFile(rtconfpath);
		}
		cerr << "Loading config file: " << rtconfpath << endl;
		// Read in configuration file
		config::configsegment* cRoot = config::configsegment::loadFile(
				rtconfpath.c_str(), NULL);

		// Get list of desired frequencies
		// List has several possible formats. A single number (in GHz)
		// means that freq only. Otherwise, the forms start,step,stop,
		// with separators of ,: are parsed
		double f[3] = {0, 0, 0}; // Start, step, stop
		// Uses boost tokenizer
		{
			string strF;
			flag = p.readParam<string>("-f", strF);
			if (!flag)
			{
				cerr << "Frequency range must be specified.\n";
				doHelp();
			}
			// Tokenize the string
			// The way that I wrote it, connand::processCommands
			// cannot yet do this level of parsing. Too bad.
			typedef boost::tokenizer<boost::char_separator<char> >
				tokenizer;
			boost::char_separator<char> sep(",:");
			tokenizer tokens(strF, sep);
			std::vector<double> fs;
			for (tokenizer::iterator tok_iter = tokens.begin();
					tok_iter != tokens.end(); ++tok_iter)
				fs.push_back( atof(tok_iter->c_str()) );
			if (fs.size() < 3)
			{
				f[0] = fs[0];
				f[2] = fs[0];
				f[1] = 1;
				cerr << "Frequency: " << f[0] << " GHz" << endl;
			} else {
				f[0] = fs[0];
				f[1] = fs[1];
				f[2] = fs[2];
				cerr << "Frequency range (GHz): from " << f[0] << 
					" to " << f[2] << " step " << f[1] << endl;
			}
			if (f[0] <= 0 || f[2] < f[0] || f[1] <= 0 || f[2] <= 0)
			{
				cerr << "Bad frequency input.\n";
				doHelp();
			}
		}

		string profdir, profext;
		// Get profile directory
		cRoot->getVal("atmos/atmosDir", profdir);
		if (profdir == "") profdir == "./";
		// If profile search directory is user-specified, override
		p.readParam<string>("--profdir", profdir); 
		cerr << "Profile search directories: " << profdir << endl;
		// Get profile extensions
		cRoot->getVal("atmos/profileExtensions",profext);

		// Get profile name for calculations
		string profname;
		flag = p.readParam<string>("--profile", profname);
		if (!flag) doHelp();
		cerr << "Profile desired: " << profname << ". Searching..." << endl;
		// Find the actual path to the atmospheric profile
		string profpath;
		{
			rtmath::config::findFile psearch(profdir, profext);
			bool found = psearch.search(profname,profpath);
			if (!found) throw debug::xMissingFile(profname.c_str());
		}

		cerr << "Profile found in " << profpath << endl;
		
		/*
		// Get HITRAN paths
		{
			string fMolparam, fHITRAN, fParsum;
			cRoot->getVal("lbl/parsumFile", fParsum);
			cRoot->getVal("lbl/hitranFile", fHITRAN);
			cRoot->getVal("lbl/molparamsFile", fMolparam);
			cerr << "Loading HITRAN first (it provides profile line information)" << endl;
			using namespace rtmath::debug;
			using namespace rtmath::lbl;
			timestamp(false);
			specline::loadlines(fHITRAN.c_str(),
					fMolparam.c_str(), fParsum.c_str());
			specline::speclineShape = rtmath::lbl::lineshape::LORENTZIAN;
			timestamp(false);
			cerr << "HITRAN loaded in TODO seconds." << endl;
		}
		*/

		// Read atmospheric profile
		atmos::atmos atm;
		atm.loadProfile(profpath.c_str());
		cerr << "Running profile " << atm.name << endl;
		cerr << "frequency (GHz), tau (nepers)" << endl;

		{
			// Construct set of frequencies to analyze. Be nice and make sure
			// that the end frequency is in the set
			set<double> freqs;
			for (double fr = f[0]; fr <= f[2]; fr+=f[1])
				freqs.insert(fr);
			if (freqs.count(f[2]) == 0) freqs.insert(f[2]);

			// Loop through freqs
			set<double>::const_iterator it;
			for (it = freqs.begin(); it != freqs.end(); it++)
			{
				double tau = atm.tau(*it);
				cout << *it << "," << tau << endl;
			}
		}
		
	}
	catch (rtmath::debug::xError &err)
	{
		err.Display();
		cerr << endl;
		return 1;
	}
	return 0;
}


void doHelp()
{
	using namespace std;
	cout << "rtmath-transmittance\n";
	cout << "A program for calculating transmittance in an atmosphere." << endl;
	cout << "Options:\n";
	cout << "--config\n";
	cout << "\tOverride the path to the master config file. This\n";
	cout << "\tfile is used when finding atmospheric profiles\n";
	cout << "\tand the HITRAN database." << endl;
	cout << "-f (frequency range)\n";
	cout << "\tSpecify the range of frequencies (in GHz) for\n";
	cout << "\ttransmittance calculation. Either specify a\n";
	cout << "\tsingle frequency, or specify a set of frequencies\n";
	cout << "\tusing the form (start,increment,stop)." << endl;
	cout << "--profdir\n";
	cout << "\tOverride the search directory for atmospheric\n";
	cout << "\tprofiles. By default, we look in config and ./\n";
	cout << "--profile\n";
	cout << "\tSpecify the desired atmospheric profile. Consult\n";
	cout << "\tthe manual for the file format.\n";
	cout << endl;
	cout << endl;
	exit(1);
}

