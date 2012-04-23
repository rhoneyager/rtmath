/* This program generates ddscat runs. It does more than converting ddscat.par files. It 
 * can generate several directories' worth of ddscat.par files, varying the frequency of a 
 * shape and recalculating the refractive index, along with being able to resize a shape while 
 * keeping its dimensions within a specified size distribution. 
 *
 * These two properties are quite useful when producing hexagonal plates and then generating 
 * dendrites that have the same aspect ratio and effective radius. This does entail being 
 * able to rewrite shape.par files, of course, to change the scaling of a given shape.
 * I have to do it this way because reproducing a shape file is outside of the scope of 
 * rtmath for now.
 */

#include <iostream>
#include <fstream>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/parameter/keyword.hpp>
#include <istream>
#include <iterator>
#include <sstream>
#include "../../rtmath/rtmath/rtmath.h"

// App-specific header files
#include "help.h"

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
		rtmath::debug::appEntry(argc, argv); // Standard config argument parsing

		// For this application, I will be using the boost command-line parsing system
		//config::parseParams p(argc,argv);
		//if (p.readParam("-h")) doHelp();


		// Load in the basic configuration file (rtmath defaults)

		// Depending on user input, either create a new ddscat structure based on defaults or 
		// load in the defaults from a user-defined file. It really is the same thing.


		// If this involves a shapefile, load it

		// Load in the temperature range

		// Load in the listed frequencies
		// These may be specified in either the standard form (start:[interval:]stop) or can
		// be expressed in expanded notation (start:[interval:]stop:spacing), where spacing 
		// is lin, log, inv or exp, or can include a list of frequencies that maps to the 
		// rtmath definitions.


		// Select the range of particle sizes to consider. This range may be specified in many 
		// possible formats, like volume, effective radius, major/minor axes
		// Note: this is also dependent upon the generated shapes. If no options are 
		// specified, just generate for the defaults listed in the default ddscat.par file.
		// Note: parsing leverages the frequency parsing stuff.
		// TODO: should also be able to specify data based on Gunn-Marshal and other size 
		// distributions, with points being able to select for this. For now, though, just 
		// specify this according to exponential spacings.

		// Specify interdipole spacing. May conflict with prior options if a shape.dat file is involved

		// Shall we output supplementary data?

		// Generate top-level output directory

		// Write overall parameter information file

		// Loop over all possible sets of conditions and execute generator function


		// And we're done
	}
	catch (rtmath::debug::xError &err)
	{
		err.Display();
		exit(1);
	}
	return 0;
}
