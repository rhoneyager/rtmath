#pragma once
// This contains the class used for processing commands.
// These commands are strings sent through MPI, through standard sockets, 
// through pipes, or are located in config segments.
// This is useful in coordinating several disparate processes.
#include <string>
#include <memory>

namespace rtmath {
	namespace config {
		void processCommands(const char* commands);
	}; // end namespace config
}; // end namespace rtmath

