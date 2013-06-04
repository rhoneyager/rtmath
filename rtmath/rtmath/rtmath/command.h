#pragma once
// This contains the class used for processing commands.
// These commands are strings sent through MPI, through standard sockets, 
// through pipes, or are located in config segments.
// This is useful in coordinating several disparate processes.
#include <string>
#include <sstream>
#include <set>
#include <vector>

namespace boost { namespace filesystem { class path; } }

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

			// Check only for parameter existence
			bool readParam(const char* oName);

			// Read first occurance of the same flag only
			template <class T> bool readParam(const char* oName, T &val) const
			{
				// Find the option
				std::string op(oName);
				for (size_t i=0;i<(size_t) _ac-1;i++)
				{
					std::string p(_av[i]);
					if (p==op)
					{
						std::string v(_av[i+1]);
						// Use stringstream for conversion
						std::istringstream is(v);
						is >> val;
						return true;
					}
				}
				return false;
			}

			// Read multiple occurances of the same flag, preserving value order
			// If no intervening flags, will continue reading options, thus
			// allowing for shell-provided globbing
			template <class T> bool readParam(const char* oName, std::vector<T> &vals) const
			{
				// Find the option
				std::string op(oName);
				vals.clear();
				bool glob = false;
				for (size_t i=1;i<(size_t) _ac;i++)
				{
					std::string p(_av[i]);
					if (p[0] == _flag) glob = false;
					if (p==op) 
					{
						glob = true;
						continue;
					}
					if (glob)
					{
						std::string v(_av[i]);
						// Use stringstream for conversion
						std::istringstream is(v);
						T val;
						is >> val;
						vals.push_back(val);
					}
				}
				if (vals.size()) return true;
				return false;
			}

			// Read first occurance of the same flag only
			template <class T> bool readParam(const char* oName, size_t num, T *vals) const
			{
				// Find the option
				using namespace std;
				string op(oName);
				for (size_t i=0;i<(size_t) (_ac-num);i++)
				{
					string p(_av[i]);
					if (p==op)
					{
						for (size_t j=0;j<num;j++)
						{
							string s(_av[i+1+j]);
							std::istringstream is(s);
							is >> vals[j];
						}
						return true;
					}
				}
				return false;
			}

			// Read multiple occurances of the same flag, preserving value order
			template <class T> bool readParam(const char* oName, size_t num, std::vector< std::vector<T> > &vals) const
			{
				vals.clear();
				// Find the option
				using namespace std;
				string op(oName);
				for (size_t i=0;i<(size_t) (_ac-num);i++)
				{
					string p(_av[i]);
					if (p==op)
					{
						vector<T> cval;
						for (size_t j=0;j<num;j++)
						{
							T cur;
							string s(_av[i+1+j]);
							std::istringstream is(s);
							is >> cur;
							cval.push_back(cur);
						}
						vals.push_back(cval);
						i+=num;
					}
				}
				if (vals.size()) return true;
				return false;
			}

		private:
			int _ac;
			char** _av;
			char _flag;
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


	} // end namespace config
} // end namespace rtmath

