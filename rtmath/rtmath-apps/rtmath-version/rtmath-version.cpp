#include <iostream>
#include <fstream>
#include <Ryan_Debug/debug.h>
#include "../../rtmath/rtmath/error/debug.h"
#include "../../rtmath/rtmath/config.h"

int main(int argc, char* argv[])
{
	using namespace std;
	using namespace rtmath;

	rtmath::debug::debug_preamble();

	string sConfig;
	rtmath::config::getConfigDefaultFile(sConfig);
	if (sConfig.size())
		cerr << "Using rtmath configuration file: " << sConfig << endl << endl;
	else cerr << "rtmath configuration file not found." << endl << endl;
	
	Ryan_Debug::printDebugInfo();

	return 0;
}
