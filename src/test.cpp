#include <iostream>
#include "../Ryan-Debug/debug.h"
#include "../Ryan-Debug/info.h"

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

