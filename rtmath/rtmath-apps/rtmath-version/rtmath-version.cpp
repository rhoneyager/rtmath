#include <iostream>
#include <fstream>
#include <Ryan-Debug/debug.h>
#include "../../rtmath/rtmath/error/debug.h"

int main(int argc, char* argv[])
{
	using namespace std;
	using namespace rtmath;

	rtmath::debug::debug_preamble();
	ryan_debug::printDebugInfo();

	return 0;
}
