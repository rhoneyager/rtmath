#include <iostream>
#include <fstream>

#include "../../rtmath/rtmath/rtmath.h"

int main(int argc, char* argv[])
{
	using namespace std;
	using namespace rtmath;

	atexit(rtmath::debug::appExit);
	rtmath::debug::debug_preamble();

	return 0;
}
