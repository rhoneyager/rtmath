#pragma once
// This contains the class used for processing commands.
// These commands are strings sent through MPI, through standard sockets, 
// through pipes, or are located in config segments.
// This is useful in coordinating several disparate processes.
#include <string>
#include <memory>
#include <boost/filesystem.hpp>
#include <set>
#include <vector>

namespace rtmath {
	namespace config {
		void processCommands(const char* commands);

		// parseParams is my implementation of a runtime command-line 
		// processor. It can find various arguments and their associated 
		// parameters. It can also convert between different storage units.
		class parseParams
		{
			public:
				parseParams(int argc, char** argv);
				bool readParam(const char* oName, double &val);
				bool readParam(const char* oName, size_t num, double *vals);
				bool readParam(const char* oName, std::string &val);
				bool readParam(const char* oName, bool &flag);
				bool readParam(const char* oName);
			private:
				int _ac;
				char** _av;
		};

		// findFile is another useful class, geared towards locating a file.
		// It is used in atmospheric profile searches, where the profile 
		// may be located in several directories and may contain various 
		// prefixes or suffixes.
		// Note: paths have ordering. Suffixes don't.
		class findFile
		{
		public:
			// Default dumb constructor
			findFile();
			// Set a default search path and a set of default optional extensions
			// searchPath and suffixes may be comma-delimited for convenience
			findFile(const std::string &searchPath,
				const std::string &suffixes);
			findFile(const std::vector<std::string> &searchPath,
				const std::set<std::string> &suffixes);
			void setSearch(const std::vector<std::string> &searchPath);
			void clearSearch();
			void addSearch(const std::string &searchPath);
			void addSearch(const boost::filesystem::path &searchPath);
			void clearSuffix();
			void setSuffix(const std::set<std::string> &suffixes);
			void addSuffix(const std::string &suffix);

			bool search(const std::string &token, std::string &res) const;
			bool searchSubDir() const;
		private:
			std::set<std::string> _suffixes;
			std::vector<std::string> _paths;
		};
	}; // end namespace config
}; // end namespace rtmath

