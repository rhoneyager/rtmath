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
		class parseParams
		{
			public:
				parseParams(int argc, char** argv);
				bool readParam(const char* oName, double &val);
				bool readParam(const char* oName, size_t num, double *vals);
				bool readParam(const char* oName, std::string &val);
				bool readParam(const char* oName, bool &flag);
			private:
				int _ac;
				char** _av;
		};
	}; // end namespace config
}; // end namespace rtmath

