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
#include <iostream>
#include <ostream>
#include <iomanip>
#include <map>
#include <set>
#include <memory>
#include <boost/lexical_cast.hpp>


// TODO: fix findSegment so that it works
// TODO: more throwable errors
// TODO: findSegment check for not found condition (currently returns garbage)
// TODO: restructure to explicitly enable symlinks
namespace rtmath {
	namespace config {

		class configsegment : public std::enable_shared_from_this<configsegment> {
		public:
			static std::shared_ptr<configsegment> create(const std::string &name);
			static std::shared_ptr<configsegment> create(const std::string &name, 
				std::shared_ptr<configsegment> &parent);
			configsegment(const std::string &name);
			configsegment(const std::string &name, std::shared_ptr<configsegment> &parent);
			~configsegment();
			std::shared_ptr<configsegment> getPtr() const;
			bool hasVal(const std::string &key) const;
			bool getVal(const std::string &key, std::string &value) const;
			template <class T> bool getVal(const std::string &key, T &value) const
			{
				std::string valS;
				bool success = getVal(key,valS);
				if (success)
					value = boost::lexical_cast<T>(valS);
				return success;
			}
			void setVal(const std::string &key, const std::string &value);
			void name(std::string &res) const;
			inline std::string name() const { std::string res; name(res); return res; }
			void move(std::shared_ptr<configsegment> &newparent);
			std::shared_ptr<configsegment> findSegment(const std::string &key) const;
			std::shared_ptr<configsegment> getChild(const std::string &name) const;
			std::shared_ptr<configsegment> addChild(std::shared_ptr<configsegment> child);
			std::shared_ptr<configsegment> getParent() const;
			void listKeys(std::set<std::string> &res) const;
			inline std::set<std::string> listKeys() const { std::set<std::string> res; listKeys(res); return res; }
			void listChildren(std::set<std::string> &res) const;
			inline std::set<std::string> listChildren() const { std::set<std::string> res; listChildren(res); return res; }
		protected:
			std::string _segname;
			std::weak_ptr<configsegment> _parent;
			//std::weak_ptr<configsegment> _self;
			std::map<std::string, std::string> _mapStr;
			std::set<std::shared_ptr<configsegment> > _children;
			std::set<std::weak_ptr<configsegment> > _symlinks;
		public: // And let's have a static loading function here!
			static std::shared_ptr<configsegment> loadFile(const char* filename, std::shared_ptr<configsegment> root);
			static std::shared_ptr<configsegment> loadFile(std::istream &indata, std::shared_ptr<configsegment> root, const std::string &cwd = "./");
			friend std::ostream& operator<<(std::ostream& stream, const rtmath::config::configsegment &ob);
			friend std::istream& operator>> (std::istream &stream, std::shared_ptr<rtmath::config::configsegment> &ob);
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

		std::ostream& operator<< (std::ostream &stream, const rtmath::config::configsegment &ob);
		std::istream& operator>> (std::istream &stream, std::shared_ptr<rtmath::config::configsegment> &ob);

	}; // end namespace config
}; // end namespace rtmath

// ostream override
//std::ostream& operator<< (std::ostream &stream, const rtmath::config::configsegment &ob);
// istream override
//std::istream& std::istream::operator>> (std::istream &stream, rtmath::config::configsegment &ob);


