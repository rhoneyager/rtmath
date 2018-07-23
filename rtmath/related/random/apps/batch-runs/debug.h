#pragma once

/* The debug functions are mostly taken out of rtmath. They exist here because I don't want for 
 * the tmatrix code to depend on rtmath. Neither do I want this program to be in rtmath at all.
 */
#include <iostream>


namespace tmatrix {
	namespace debug
	{
		void appEntry(int argc, char** argv);
		bool waitOnExit();
		void appExit();
		bool pidExists(int pid);
		void debug_preamble(std::ostream &out = std::cerr);
		int getPID();
		int getPPID(int pid = 0);
	}
}


