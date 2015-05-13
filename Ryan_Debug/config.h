#pragma once
#include "defs.h"

#include <string>
#include <iostream>
#include <map>
#include <set>
#include <boost/shared_ptr.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <boost/lexical_cast.hpp> // used in getVal<>
#include "registry.h"
#include "io.h"

namespace Ryan_Debug {
	namespace config {
		class configsegment;
		class configsegment_IO_input_registry{};
		class configsegment_IO_output_registry{};
		class configsegment_OldStandard {};
		class configsegment_Boost {};
		class configsegment_Env {}; //todo
		class configsegment_Registry {}; //todo
		}
	namespace registry {
		extern template struct IO_class_registry_writer<
			::Ryan_Debug::config::configsegment>;

		extern template struct IO_class_registry_reader<
			::Ryan_Debug::config::configsegment>;

		extern template class usesDLLregistry<
			::Ryan_Debug::config::configsegment_IO_input_registry,
			IO_class_registry_reader<::Ryan_Debug::config::configsegment> >;

		extern template class usesDLLregistry<
			::Ryan_Debug::config::configsegment_IO_output_registry,
			IO_class_registry_writer<::Ryan_Debug::config::configsegment> >;
	}
	namespace io {
		template <>
		RYAN_DEBUG_DLEXPORT boost::shared_ptr
			<::Ryan_Debug::config::configsegment> customGenerator();
	}

	/** \brief describes the structure that should be contained in options files that
	 * involve the Ryan_Debug library.
	 * 
	 * This is to avoid having so create system-specific answer files or 
	 * having to reenter options on each execution. The actual configuration files
	 * will be based on the structure of Apache's httpd.conf.
	 *
	 * There are general options and specific containers. The file can reference other files
	 * as well. Global options are visible in the subcontainers, though these may be
	 * overridden for granularity and to allow for reusing files between multiple machines.
	 * The Ryan_Debug main program will parse the file and look for it in a set of pre-programmed
	 * locations upon execution.
	 *
	 * Once a main config file is read, the config options specify the next step. Daemons
	 * will prepare themselves for runs and will then wait for a connection. Further
	 * instructions will be transmitted in a structure similar to that of the standard
	 * config file. That necessary execute blurb, if present in a console application,
	 * will determine what the app does. If missing, an interactive app will just ask
	 * the user what to do.
	 **/
	namespace config {
		/// Provides local readers and writers for old configuration format (it's a binder)
		class RYAN_DEBUG_DLEXPORT implementsConfigOld :
			private Ryan_Debug::io::implementsIObasic<configsegment, configsegment_IO_output_registry,
			configsegment_IO_input_registry, configsegment_OldStandard>
		{
		public:
			virtual ~implementsConfigOld() {}
		protected:
			implementsConfigOld();
		private:
			static const std::set<std::string>& known_formats();
		};

		/// Provides local readers and writers for new configuration format for xml, json, ini (it's a binder)
		/// \todo Implement
		class RYAN_DEBUG_DLEXPORT implementsConfigBoost :
			private Ryan_Debug::io::implementsIObasic<configsegment, configsegment_IO_output_registry,
			configsegment_IO_input_registry, configsegment_Boost>
		{
		public:
			virtual ~implementsConfigBoost() {}
		protected:
			implementsConfigBoost();
		private:
			static const std::set<std::string>& known_formats();
			friend class configsegment;
			//static void writeSegment(
			//	const boost::shared_ptr<const Ryan_Debug::config::configsegment> it,
			//	boost::property_tree::ptree& parent,
			//	std::map<boost::shared_ptr<const Ryan_Debug::config::configsegment>, size_t > &encountered,
			//	size_t &id_count);
		};

		/// \note Upgrading this to just be a wrapper for a boost property tree
		/// \todo fix findSegment so that it works
		/// \todo findSegment check for not found condition (currently returns garbage)
		/// \todo restructure to explicitly enable symlinks
		class RYAN_DEBUG_DLEXPORT configsegment : 
			virtual public boost::enable_shared_from_this<configsegment>,
			virtual public ::Ryan_Debug::registry::usesDLLregistry<
			::Ryan_Debug::config::configsegment_IO_input_registry,
			::Ryan_Debug::registry::IO_class_registry_reader<configsegment> >,
			virtual public ::Ryan_Debug::registry::usesDLLregistry<
			::Ryan_Debug::config::configsegment_IO_output_registry,
			::Ryan_Debug::registry::IO_class_registry_writer<configsegment> >,
			virtual public ::Ryan_Debug::io::implementsStandardWriter<configsegment, configsegment_IO_output_registry>,
			virtual public ::Ryan_Debug::io::implementsStandardReader<configsegment, configsegment_IO_input_registry>,
			virtual public implementsConfigOld,
			virtual public implementsConfigBoost
		{
			// Need readVector as a friend class
			friend boost::shared_ptr<configsegment> io::customGenerator<configsegment>();
			configsegment(const std::string &name);
			configsegment(const std::string &name, boost::shared_ptr<configsegment> &parent);
		public:
			~configsegment();
			static boost::shared_ptr<configsegment> generate(const std::string &name);
			static boost::shared_ptr<configsegment> generate(const std::string &name,
				boost::shared_ptr<configsegment> &parent);
			
			static void readOld(boost::shared_ptr<configsegment>, std::istream&, std::shared_ptr<registry::IO_options>);
			static void writeOld(const boost::shared_ptr<const configsegment>, std::ostream &, std::shared_ptr<registry::IO_options>);
			static void readBoost(boost::shared_ptr<configsegment>, std::istream&, std::shared_ptr<registry::IO_options>);
			static void writeBoost(const boost::shared_ptr<const configsegment>, std::ostream &, std::shared_ptr<registry::IO_options>);

			boost::shared_ptr<configsegment> getPtr() const;
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
			void addVal(const std::string &key, const std::string &value);

			void name(std::string &res) const;
			inline std::string name() const { std::string res; name(res); return res; }
			boost::shared_ptr<configsegment> findSegment(const std::string &key) const;
			boost::shared_ptr<configsegment> getChild(const std::string &name) const;
			void addChild(boost::shared_ptr<configsegment> child);
			void removeChild(boost::shared_ptr<configsegment> child);
			void uncouple(); // Delete entire object from all parents.
			std::set<boost::shared_ptr<configsegment> > getParents() const;
			void listKeys(std::multimap<std::string,std::string> &output) const;
			const std::multimap<std::string, std::string>& listKeys() const { return _mapStr; }
			void listKeys(std::multiset<std::string> &res) const;
			inline std::multiset<std::string> enumKeys() const { std::multiset<std::string> res; listKeys(res); return res; }
			void listChildren(std::multiset<boost::shared_ptr<configsegment> > &res) const;
			void listChildren(std::multiset<std::string> &res) const;
			inline std::multiset<boost::shared_ptr<configsegment> > listChildren() const { std::multiset<boost::shared_ptr<configsegment> > res; listChildren(res); return res; }
			void getCWD(std::string &cwd) const; // Returns directory of the loaded config file node.

			
		protected:
			std::string _segname, _cwd;
			std::set<boost::weak_ptr<configsegment> > _parents;
			//std::weak_ptr<configsegment> _self;
			std::multimap<std::string, std::string> _mapStr;
			std::multiset<boost::shared_ptr<configsegment> > _children;
			//std::multiset<boost::weak_ptr<configsegment> > _symlinks;
		public: // And let's have a static loading function here!
			//static boost::shared_ptr<configsegment> loadFile(const char* filename, boost::shared_ptr<configsegment> root);
			//static boost::shared_ptr<configsegment> loadFile(std::istream &indata, boost::shared_ptr<configsegment> root, const std::string &cwd = "./");
			//friend RYAN_DEBUG_DLEXPORT std::ostream& operator<< (std::ostream& stream, const Ryan_Debug::config::configsegment &ob);
			//friend std::istream& std::operator>> (std::istream &stream, std::shared_ptr<Ryan_Debug::config::configsegment> &ob);
		};

		// Easy-to-use function that looks in config for a property. If not found, ask the user!
		inline std::string queryConfig(boost::shared_ptr<configsegment> &root, const std::string &key, const std::string &question)
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

		void RYAN_DEBUG_DLEXPORT getConfigDefaultFile(std::string &filename);
		boost::shared_ptr<configsegment> RYAN_DEBUG_DLEXPORT getRtconfRoot();
		/// Load the appropriate default Ryan_Debug configuration file (default may be overridden in command line, see registry.cpp)
		boost::shared_ptr<configsegment> RYAN_DEBUG_DLEXPORT loadRtconfRoot(const std::string &filename = "");
		void RYAN_DEBUG_DLEXPORT setRtconfRoot(boost::shared_ptr<configsegment> &root);

		//extern std::shared_ptr<configsegment> _rtconfroot;

		/// Take the object, and print in the appropriate form, using recursion
		//RYAN_DEBUG_DLEXPORT std::ostream& operator<< (std::ostream &stream, const Ryan_Debug::config::configsegment &ob);
		/// Take the object, and input in the appropriate form, using recursion
		RYAN_DEBUG_DLEXPORT std::istream& operator>> (std::istream &stream, boost::shared_ptr<Ryan_Debug::config::configsegment> &ob);

	}
}

