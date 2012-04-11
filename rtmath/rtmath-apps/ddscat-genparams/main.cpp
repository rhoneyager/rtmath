/* This program will provide either a command-line or a console (ncurses) interface to ddscat parameter file generation.
 * This will work better than my current method of rewriting scripts in python and csh repeatedly. It will handle
 * interconversion of frequency/wavelength, automatic size selection, shape file generation, dielectric constant
 * generation (through getm/genmtab) and lots of other stuff. */

#include "StdAfx.h"
#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>
#include <iostream>
#include <fstream>
#include <istream>
#include <iterator>
#include <sstream>
#include "../../rtmath/rtmath/rtmath.h"

void doHelp();
// boost has an issue when copying files. So, I'll use my own function for now.
void fcopy(const boost::filesystem::path &src, const boost::filesystem::path &dst);

int main(int argc, char** argv)
{
	using namespace std;
	using namespace rtmath;
	using namespace boost::filesystem;
	using namespace rtmath::config;
	using namespace rtmath::ddscat;
	using namespace rtmath::debug;
	try {
		cerr << argv[0] << endl;
		rtmath::debug::appEntry(argc, argv);
		config::parseParams p(argc,argv);
		if (p.readParam("-h")) doHelp();

		// Absolute default setting of input source directory path
		string srcPath = "./"; // The directory with the config information (ddscat.par and shape.dat)
		// Allow rtmath.conf override
		shared_ptr<config::configsegment> cRoot = config::loadRtconfRoot();
		cRoot->getVal("ddscat-genParams/inputPath", srcPath); // Set default outputDirectory if set here
		// And the command-line overrides this
		p.readParam<string>("-i",srcPath);

		// Output directory path
		string dstPath = "./out"; // The directory with the config information (ddscat.par and shape.dat)
		// Allow rtmath.conf override
		cRoot->getVal("ddscat-genParams/outputPath", dstPath); // Set default outputDirectory if set here
		// And the command-line overrides this
		p.readParam<string>("-o",dstPath);

		// Verify the input and output paths
		path pBase(srcPath);
		if (!exists(pBase)) throw rtmath::debug::xMissingFile(pBase.string().c_str());
		// Is this a directory or a file> If a file, go to the parent
		if (is_directory(pBase))
		{
		} else {
			pBase = pBase.parent_path();
		}

		// Make sure that ddscat.par exists
		path pParFile(pBase / "ddscat.par");
		if (!exists(pBase)) throw rtmath::debug::xMissingFile(pParFile.string().c_str());

		// Find the shape file
		path pShape;
		{
			// Figure out where the shape file is located.
			path ptarget = pBase / "target.out";
			path pshapedat = pBase / "shape.dat";
			if (exists(pshapedat))
			{ pShape = pshapedat;
			} else if (exists(ptarget))
			{ pShape = ptarget;
			} else {
				throw rtmath::debug::xMissingFile("shape.dat or target.out");
			}
		}


		// See if output directory exists. If not, then create it.
		// Also add safeguards to prevent overwriting input
		path pDest(dstPath);
		if (!exists(pDest))
		{
			// Create the directory
			boost::filesystem::create_directory(pDest);
		} else {
			// If it does exist, is it a directory?
			if (is_directory(pDest))
			{
				// And, if so, is it empty?
				// TODO!!!
				GETOBJKEY();
				//throw rtmath::debug::xBadInput("Destination exists");
			} else {
				throw rtmath::debug::xBadInput("Destination exists, and is a file, not a directory.");
			}
		}
		path pDpar(pDest / pParFile.filename());
		path pDshp(pDest / pShape.filename());

		// Okay, the files all exist. Now, to load them.
		rtmath::ddscat::ddPar ddfile(pParFile.string());
		
		// Files loaded. Now, what to do with them?

		// Use flags for this...
		// Set output version (7.2 is default)
		size_t ddVer = 72;
		p.readParam<size_t>("-v", ddVer);
		ddfile.version(ddVer);

		// Add in values passed from the command line (redo of beta, theta, phi, etc.)
		if(p.readParam("-e"))
		{
			// TODO: different consoles pass quoted / unquoted strings in different ways!
			//		 Modify command.h to specialize string loading, with concatenation and 
			//		 splitting, as necessary...
			throw rtmath::debug::xUnimplementedFunction();
			vector<string> vals;
			p.readParam<string>("-e",vals);
		}
		if(p.readParam("-f"))
		{
			// Get file that provides changes, and implement one line at a time...
			string overlay;
			p.readParam<string>("-f",overlay);
			path poverlay(overlay);
			if (!exists(poverlay))
				throw rtmath::debug::xMissingFile(overlay.c_str());
			if (is_directory(poverlay))
				throw rtmath::debug::xBadInput(overlay.c_str());
			ddfile.loadFile(overlay,true);
		}
		if(p.readParam("-d"))
		{
			throw rtmath::debug::xUnimplementedFunction();
		}
		// TODO: Also allow for a more expressive version of frequency, aeff, ...
		

		// Rewrite ddscat.par to the destination, with any version changes
		ddfile.saveFile(pDpar.string());

		// Copy shape.dat / target.out to destination (no need to write the loaded version)
		fcopy(pShape, pDshp);
//		boost::filesystem::copy_file(pShape, pDshp);
		// Copy any auxiliary files (like diel.tab) mentioned in ddscat.par
		shared_ptr<ddParParsers::ddParLineSimple<std::string> > dielloc = 
			std::dynamic_pointer_cast<ddParParsers::ddParLineSimple<std::string> >(ddfile.getKey(ddParParsers::IREFR));
		if (dielloc != nullptr)
		{
			string sdl;
			dielloc->get(sdl);
			// Trim annoying spaces from end of file
			using namespace boost::algorithm;
			using namespace boost::filesystem;
			trim(sdl);
			// Make path. If path is relative, make relative to source dir
			path pDTs(sdl);
			if (pDTs.is_relative()) pDTs = pBase / path(sdl);
			path pDTd(pDest / pDTs.filename());
			fcopy(pDTs, pDTd);
//			boost::filesystem::copy_file(pDTs, pDTd);
		}

		// Calculate shape file statistics output and write to destination
		if (p.readParam("-s"))
		{
			// TODO
			rtmath::ddscat::shapefile shp(pShape.string());
			throw rtmath::debug::xUnimplementedFunction();
			// ddscat is limited in that it assumes the refractive index is 
			// frequency-independent. Thus, all of the ddscat.par files that 
			// will be encountered will only provide information for one frequency.
			// However, multiple effective radii may be considered. 
			// Also, multiple orientations are calculated.
			// For each orientation / size combination, generate statistics
			// output. I need to figure out how the filenames map in ddscat.
			// Each output orientation will provide the KE and PE relative 
			// to the minimum possible potential energy.
			// Effective radii need a better parser than normal, as spacings may
			// be lin, log, exp or inv.
			// Orientations are linear except for theta, which is linear in cos(theta).

		}

		// And we're done!
	}
	catch (rtmath::debug::xError &err)
	{
		err.Display();
		exit(1);
	}
/*	catch (...)
	{
		rtmath::debug::dumpErrorLocation();
		exit(1);
	}
*/
	return 0;
}


/*
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
			throw rtmath::debug::xUnimplementedFunction();
		}
	}
	catch (rtmath::debug::xError &err)
	{
		err.Display();
		exit(1);
	}
	return 0;
}
*/

void fcopy(const boost::filesystem::path &src, const boost::filesystem::path &dst)
{
	using namespace std;
	fstream f(src.string().c_str(), fstream::in|fstream::binary);
	f << noskipws;
	istream_iterator<unsigned char> begin(f);
	istream_iterator<unsigned char> end;
	fstream f2(dst.string().c_str(), fstream::out|fstream::trunc|fstream::binary);
	ostream_iterator<char> begin2(f2);

	copy(begin,end,begin2);
}





