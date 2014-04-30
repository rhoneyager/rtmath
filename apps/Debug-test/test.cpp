#include <iostream>
#include "../../Ryan_Debug/debug.h"
#include "../../Ryan_Debug/info.h"
#include "Ryan.Debug.DebugAssembly.manifest.h"

int main(int, char**)
{
	using namespace std;
	cerr << "Ryan Debug Testing Application" << endl << endl;
	cerr << "Library Build Settings:\n";
	Ryan_Debug::printDebugInfo();

	cerr << "Testing app build settings:\n";
	Ryan_Debug::debug_preamble(cerr);
	cerr << endl;
	return 0;
}

