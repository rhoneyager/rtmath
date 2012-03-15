/* This program will provide either a command-line or a console (ncurses) interface to ddscat parameter file generation.
 * This will work better than my current method of rewriting scripts in python and csh repeatedly. It will handle
 * interconversion of frequency/wavelength, automatic size selection, shape file generation, dielectric constant
 * generation (through getm/genmtab) and lots of other stuff. */

#include "Stdafx.h"
#include "../../rtmath/rtmath/rtmath.h"

int main(int argc, char** argv)
{
	using namespace std;
	using namespace rtmath;
	try {
		cerr << argv[0] << endl;
		rtmath::debug::appEntry(argc, argv);
	}
	catch (rtmath::debug::xError &err)
	{
		err.Display();
		exit(1);
	}
	return 0;
}
