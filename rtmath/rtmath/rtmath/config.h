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
#include <memory>

// TODO: fix findSegment so that it works
// TODO: provide output of tree structure with print() and ostream
// TODO: more throwable errors
// TODO: findSegment check for not found condition (currently returns garbage)

namespace rtmath {
	namespace config {

		class configsegment {
		public:
			static std::shared_ptr<configsegment> create(const std::string &name);
			static std::shared_ptr<configsegment> create(const std::string &name, 
				std::shared_ptr<configsegment> &parent);
			~configsegment();
			bool getVal(const std::string &key, std::string &value) const;
			bool getVal(const std::string &key, std::string &value, std::string defaultVal) const;
			void setVal(const std::string &key, const std::string &value);
			std::shared_ptr<configsegment> findSegment(const std::string &key) const;
			std::shared_ptr<configsegment> getChild(const std::string &name) const;
			std::shared_ptr<configsegment> addChild(std::shared_ptr<configsegment> child);
			std::shared_ptr<configsegment> getParent() const;
		protected:
			configsegment(const std::string &name);
			std::string _segname;
			std::weak_ptr<configsegment> _parent;
			std::weak_ptr<configsegment> _self;
			std::map<std::string, std::string> _mapStr;
			std::set<std::shared_ptr<configsegment> > _children;
		public: // And let's have a static loading function here!
			static std::shared_ptr<configsegment> loadFile(const char* filename, std::shared_ptr<configsegment> root);
		};

		// Easy-to-use function that looks in config for a property. If not found, ask the user!
		inline std::string queryConfig(std::shared_ptr<configsegment> &root, const std::string &key, const std::string &question)
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

		void getConfigDefaultFile(std::string &filename);
		std::shared_ptr<configsegment> getRtconfRoot();
		std::shared_ptr<configsegment> loadRtconfRoot(const std::string &filename);
		std::shared_ptr<configsegment> loadRtconfRoot();
		void setRtconfRoot(std::shared_ptr<configsegment> &root);

		extern std::shared_ptr<configsegment> _rtconfroot;
	}; // end namespace config
}; // end namespace rtmath
