#include <iostream>
#include <fstream>

#include "../../rtmath/rtmath/rtmath.h"

int main(int argc, char* argv[])
{
	using namespace std;
	using namespace rtmath;

	rtmath::debug::debug_preamble();
#ifdef _WIN32
	std::getchar();
#endif
	return 0;
}
