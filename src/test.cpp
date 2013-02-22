#include <iostream>
#include "../ryan-debug/debug.h"
#include "../ryan-debug/info.h"

int main(int, char**)
{
	using namespace std;
	cerr << "Ryan Debug Testing Application" << endl << endl;
	cerr << "Library Build Settings:\n";
	ryan_debug::printDebugInfo();

	cerr << "Testing app build settings:\n";
	ryan_debug::debug_preamble(cerr);
	cerr << endl;
	return 0;
}

