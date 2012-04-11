#include "StdAfx.h"
#include <iostream>

void doHelp()
{
	using namespace std;
	cerr << "---------------------\n\n";
	cerr << "Options:\n";
	cerr << "-i (input directory)\n";
	cerr << "\tSpecify the input directory for processing. At the\n";
	cerr << "\tleast, this directory must have ddscat.par and diel.tab.\n";
	cerr << "\tShape files will be loaded if detected.\n";
	cerr << "-o (output directory)\n";
	cerr << "\tSpecify a target for output. Must either not exist or be\n";
	cerr << "\tan empty directory for output.\n";
	cerr << "-v (ddscat output version)\n";
	cerr << "\tUse this option to convert between ddscat versions.\n";
	cerr << "\tSupported versions are 70 and 72 (7.0 and 7.2)\n";
	cerr << "-s\n";
	cerr << "\tIf shapefile exists, calculate statistics.\n";
	cerr << "-f (ddscat-like file)\n";
	cerr << "\tThis option is used for patching the main ddscat file with\n";
	cerr << "\talternate options. Used in changing betas, solver methods, ...\n";
	cerr << "-e (UNIMPLEMENTED, quoted ddscat line)\n";
	cerr << "\tLike -f, but passes line through the command line.\n";
	cerr << "-d (UNIMPLEMENTED, line id)\n";
	cerr << "\tDelete the specified line (generally not a good idea.\n";
	exit(1);
}
