/* This program will provide either a command-line or a console (ncurses) interface to ddscat parameter file generation.
 * This will work better than my current method of rewriting scripts in python and csh repeatedly. It will handle
 * interconversion of frequency/wavelength, automatic size selection, shape file generation, dielectric constant
 * generation (through getm/genmtab) and lots of other stuff. */

#include "StdAfx.h"
#include <boost/filesystem.hpp>
#include <iostream>
#include <fstream>
#include <sstream>
#include "../../rtmath/rtmath/rtmath.h"

void doHelp();

int main(int argc, char** argv)
{
	using namespace std;
	using namespace rtmath;
	try {
		cerr << argv[0] << endl;
		rtmath::debug::appEntry(argc, argv);

		string srcPath = "./"; // The directory or file with the config information
		const string defaultFile = "daparams.txt";
		string outputDir = "./"; // the directory root for the outputs

		// Read in the global config
		shared_ptr<config::configsegment> cRoot = config::loadRtconfRoot();
		cRoot->getVal("ddscat-genparams/outputPath", outputDir); // Set default outputDirectory if set here

		// Parse command-line parameters
		rtmath::config::parseParams p(argc,argv);
		if (p.readParam("-h")) doHelp();
		p.readParam<string>("-i",srcPath); // Override the source path
		p.readParam<string>("-o", outputDir); // Override the output path
		

		// This is all that I need. Now to figure out the input filename
		{
			using namespace boost::filesystem;
			path pIn(srcPath);
			if (boost::filesystem::is_directory(pIn))
				pIn = pIn / defaultFile;
			if (!exists(pIn)) throw rtmath::debug::xMissingFile(pIn.string().c_str());
			srcPath = pIn.string();
		}
		// Load source file as config file
		shared_ptr<config::configsegment> cFile = config::configsegment::loadFile(srcPath.c_str(),nullptr);

		// From here, query the config file to provide information about the targets
		size_t numRuns = 0;
		cFile->getVal<size_t>("RunSet/NumRuns", numRuns);

		// Iterate through each run
		for (size_t i=0;i<numRuns;i++)
		{
			// Create the branch name
			ostringstream bname;
			bname << "Run_" << i;
			shared_ptr<config::configsegment> cRun = cFile->getChild(bname.str());
			if (cRun == nullptr) throw debug::xBadInput(srcPath.c_str());

			string sName, sDate_created, sFreqs, sShape, sShapeParams, sVolume;
			cRun->getVal("Name",sName);
			cRun->getVal("Date_Created",sDate_created);
			cRun->getVal("Frequencies",sFreqs);
			cRun->getVal("Shape",sShape);
			cRun->getVal("Shape_Params",sShapeParams);
			cRun->getVal("Volume",sVolume);

			// Convert the strings into sets and other usable values.
		}
	}
	catch (rtmath::debug::xError &err)
	{
		err.Display();
		exit(1);
	}
	return 0;
}
