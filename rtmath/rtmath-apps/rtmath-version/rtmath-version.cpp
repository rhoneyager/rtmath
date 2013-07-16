#include <iostream>
#include <fstream>
#include <Ryan_Debug/debug.h>
#include "../../rtmath/rtmath/error/debug.h"

int main(int argc, char* argv[])
{
	using namespace std;
	using namespace rtmath;

	rtmath::debug::debug_preamble();
	Ryan_Debug::printDebugInfo();

	return 0;
}
