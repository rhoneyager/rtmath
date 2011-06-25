/* config.h - describes the structure that should be contained in options files that
 * involve the rtmath library. This is to avoid having so create system-specific answer
 * files or having to reenter options on each execution. The actual configuration files
 * will be based on the structure of Apache's httpd.conf.
 *
 * There are general options and specific containers. The file can reference other files
 * as well. Global options are visible in the subcontainers, though these may be
 * overridden for granularity and to allow for reusing files between multiple machines.
 * The rtmath main program will parse the file and look for it in a set of pre-programmed
 * locations upon execution.
 *
 * Once a main config file is read, the config options specify the next step. Daemons
 * will prepare themselves for runs and will then wait for a connection. Further
 * instructions will be transmitted in a structure similar to that of the standard
 * config file. That necessary execute blurb, if present in a console application,
 * will determine what the app does. If missing, an interactive app will just ask
 * the user what to do.
 */
#pragma once

#include <string>
#include <map>
#include <set>

namespace rtmath {
	namespace config {
		class configsegment;

		class configsegment {
		public:
			configsegment(const std::string &name);
			configsegment(const std::string &name, configsegment *parent);
			void getVal(const std::string &key, std::string &value);
			void setVal(const std::string &key, const std::string &value);
			configsegment* findSegment(const std::string &key);
			configsegment* getChild(const std::string &name);
			configsegment* getParent() const;
		private:
			std::string _segname;
			configsegment *_parent;
			std::map<std::string, std::string> _mapStr;
			std::set<configsegment*> _children;
		public: // And let's have a static loading function here!
			static configsegment* loadFile(const char* filename, configsegment* root);
		};

		// Easy-to-use function that looks in config for a property. If not found, ask the user!
		inline std::string queryConfig(configsegment* root, const std::string &key, const std::string &question)
		{
			std::string res;
			root->getVal(key,res);
			if (res.size() == 0)
			{
				std::cout << question;
				std::getline(std::cin,res);
			}
			return res;
		}

	}; // end namespace config
}; // end namespace rtmath
