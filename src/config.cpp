#include <algorithm>
#include <iostream>
#include <fstream>
#include <sstream>
#include <boost/filesystem.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <boost/program_options.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/tokenizer.hpp>
#include <boost/log/sources/global_logger_storage.hpp>

#include "../Ryan_Debug/debug.h"
#include "../Ryan_Debug/fs.h"
#include "../Ryan_Debug/logging.h"
#include "internal.h"

#include "../Ryan_Debug/config.h"
#include "../Ryan_Debug/splitSet.h"
#include "../Ryan_Debug/Serialization.h"
#include "../Ryan_Debug/debug.h"
#include "../Ryan_Debug/error.h"


// Special compile-time generated files that build needs
#include "debug_subversion.h"

#include "cmake-settings.h"

namespace {
	std::set<std::string> mtypes, mnewtypes;
	std::mutex cmlock;

	boost::shared_ptr<::Ryan_Debug::config::configsegment> _rtconfroot = nullptr;

	

	void writeSegment(
		const boost::shared_ptr<const Ryan_Debug::config::configsegment> it, 
		boost::property_tree::ptree& parent,
		std::map<boost::shared_ptr<const Ryan_Debug::config::configsegment>, size_t > &encountered,
		size_t &id_count)
	{
		using namespace boost::property_tree;
		//path.push_back(it->name());
		auto makeId = [&]() -> size_t {
			id_count++;
			return id_count;
		};

		std::string name = it->name();
		if (!name.size()) name = "Ryan_Debug";
		if (encountered.count(it))
		{
			// Hard link
			std::string slink = "hardlink:";
			slink.append(boost::lexical_cast<std::string>(encountered.at(it)));
			ptree self;
			self.add(it->name(), slink);
			parent.add_child(name, self);
		} else {
			const size_t id = makeId();
			encountered[it] = id;
			ptree self;
			// Add keys
			for (const auto &k : it->listKeys())
				self.add(k.first, k.second);
			// Add children
			for (const auto &k : it->listChildren())
				writeSegment(k, self, encountered, id_count);

			parent.add_child(name, self);
		}
		//path.pop_back();
	}

	void readSegment(
		boost::shared_ptr<Ryan_Debug::config::configsegment> cs,
		boost::property_tree::ptree& pobj,
		std::map< size_t, boost::shared_ptr<Ryan_Debug::config::configsegment> > &encountered,
		size_t &id_count
		)
	{
		using namespace boost::property_tree;
		// Iterate over all child objects and value keys
		for (auto it = pobj.begin(); it != pobj.end(); ++it)
		{
			if (it->second.data().size())
			{
				// This is a value key
				std::string sdata = it->second.data();
				if (sdata.find("hardlink:") == 0) {
					// Hard link reference
					std::string sid = sdata.substr(9);
					size_t refId = boost::lexical_cast<size_t>(sid);
					if (!encountered.count(refId)) RDthrow(Ryan_Debug::error::xCannotFindReference()) << Ryan_Debug::error::ref_number(refId);
					cs->addChild(encountered.at(refId));
				} else if (sdata.find("include:") == 0) {
					std::string sid = sdata.substr(8);
					boost::filesystem::path pInc(sid);
					boost::filesystem::path pIncSym = Ryan_Debug::fs::expandSymlink<boost::filesystem::path, boost::filesystem::path>(pInc);

					auto doReadFile = [&](const boost::filesystem::path &p)
					{
						auto ncs = cs->generate(it->first, cs);
						size_t cid_count = 0;
						std::map< size_t, boost::shared_ptr<Ryan_Debug::config::configsegment> > cenc;

						boost::property_tree::ptree pt_child;
						// Read file into stream. Handle compression.
						std::string sfile;
						Ryan_Debug::serialization::read(sfile, p.string());
						std::istringstream sin(sfile);
						boost::property_tree::read_xml(sin, pt_child);
						readSegment(ncs, pt_child, cenc, cid_count);
					};

					if (boost::filesystem::is_directory(pIncSym))
					{
						std::vector<boost::filesystem::path> vs;
						Ryan_Debug::fs::expandFolder(pIncSym, vs, false);
						for (const auto & v : vs)
						{
							auto sym = Ryan_Debug::fs::expandSymlink<boost::filesystem::path, boost::filesystem::path>(v);
							boost::filesystem::path sym_unc; std::string cmeth;
							Ryan_Debug::serialization::uncompressed_name(sym, sym_unc, cmeth);
							if (boost::filesystem::is_regular_file(sym) && sym_unc.extension().string() == ".xml")
								doReadFile(sym);
						}
					} else {
						doReadFile(pIncSym);
					}
				} else cs->addVal(it->first, it->second.data());
			} else {
				// This is a child key
				id_count++;
				auto ncs = cs->generate(it->first, cs);
				encountered[id_count] = ncs;
				readSegment(ncs, it->second, encountered, id_count);
			}
		}
	}
}

namespace Ryan_Debug {
	namespace registry {
		template struct IO_class_registry_writer
			<::Ryan_Debug::config::configsegment>;

		template struct IO_class_registry_reader
			<::Ryan_Debug::config::configsegment>;

		template class usesDLLregistry<
			::Ryan_Debug::config::configsegment_IO_output_registry,
			IO_class_registry_writer<::Ryan_Debug::config::configsegment> >;

		template class usesDLLregistry<
			::Ryan_Debug::config::configsegment_IO_input_registry,
			IO_class_registry_reader<::Ryan_Debug::config::configsegment> >;
	}

	namespace io {
		template <>
		boost::shared_ptr<::Ryan_Debug::config::configsegment> customGenerator()
		{
			boost::shared_ptr<::Ryan_Debug::config::configsegment> res
				(new ::Ryan_Debug::config::configsegment(""));
			return res;
		}
	}
	namespace config {
		BOOST_LOG_INLINE_GLOBAL_LOGGER_CTOR_ARGS(
			m_config,
			boost::log::sources::severity_channel_logger_mt< >,
			(boost::log::keywords::severity = Ryan_Debug::log::error)(boost::log::keywords::channel = "config"));

		implementsConfigOld::implementsConfigOld() :
			Ryan_Debug::io::implementsIObasic<configsegment, configsegment_IO_output_registry,
			configsegment_IO_input_registry, configsegment_OldStandard>(configsegment::writeOld, configsegment::readOld, known_formats())
		{
			static bool registered = false;
			if (!registered)
				Ryan_Debug::io::emit_io_log("Just registered config - implementsConfigOld", Ryan_Debug::log::normal);
			registered = true;
		}

		const std::set<std::string>& implementsConfigOld::known_formats()
		{
			// Moved to hidden file scope to avoid race condition
			//static std::set<std::string> mtypes;
			//static std::mutex cmlock;
			// Prevent threading clashes
			{
				std::lock_guard<std::mutex> lck(cmlock);
				if (!mtypes.size())
				{
					mtypes.insert(".Ryan_Debug");
					mtypes.insert("Ryan_Debug.conf");
					if (io::TextFiles::serialization_handle::compressionEnabled())
					{
						std::string sctypes;
						std::set<std::string> ctypes;
						serialization::known_compressions(sctypes, ".Ryan_Debug");
						serialization::known_compressions(sctypes, "Ryan_Debug.conf");
						Ryan_Debug::splitSet::splitSet(sctypes, ctypes);
						for (const auto & t : ctypes)
							mtypes.emplace(t);
					}
				}
			}
			return mtypes;
		}

		implementsConfigBoost::implementsConfigBoost() :
			Ryan_Debug::io::implementsIObasic<configsegment, configsegment_IO_output_registry,
			configsegment_IO_input_registry, configsegment_Boost>(configsegment::writeBoost, configsegment::readBoost, known_formats())
		{
			static bool registered = false;
			if (!registered)
				Ryan_Debug::io::emit_io_log("Just registered config - implementsConfigBoost", Ryan_Debug::log::normal);
			registered = true;
		}

		const std::set<std::string>& implementsConfigBoost::known_formats()
		{
			// Moved to hidden file scope to avoid race condition
			//static std::set<std::string> mtypes;
			//static std::mutex cmlock;
			// Prevent threading clashes
			{
				std::lock_guard<std::mutex> lck(cmlock);
				if (!mnewtypes.size())
				{
					mnewtypes.insert(".xml");
					//mnewtypes.insert(".json");
					//mnewtypes.insert(".ini");
					if (io::TextFiles::serialization_handle::compressionEnabled())
					{
						std::string sctypes;
						std::set<std::string> ctypes;
						serialization::known_compressions(sctypes, ".xml");
						//serialization::known_compressions(sctypes, ".json");
						//serialization::known_compressions(sctypes, ".ini");
						Ryan_Debug::splitSet::splitSet(sctypes, ctypes);
						for (const auto & t : ctypes)
							mnewtypes.emplace(t);
					}
				}
			}
			return mnewtypes;
		}


		void configsegment::writeBoost(const boost::shared_ptr<const configsegment> ob, std::ostream & stream, std::shared_ptr<registry::IO_options>)
		{
			/// \todo allow for include statements.
			using namespace std;
			using boost::property_tree::ptree;
			ptree pt;

			// Take the property and iterate over its values and children.
			// Keep track of hard and soft links. In these cases, add a reference object to the appropriate place.
			std::map<boost::shared_ptr<const configsegment>, size_t > encountered;
			//std::vector<std::string> path;
			size_t id_count = 0;
			
			writeSegment(ob, pt, encountered, id_count);
			/*
			{
			for (auto ut = ob->_symlinks.begin(); ut != ob->_symlinks.end(); ++ut)
			{
			if ((*ut).expired() == false)
			{
			stream << (*ut).lock();
			//stream << (*ut);
			}
			}
			for (auto ot = ob->_mapStr.begin(); ot != ob->_mapStr.end(); ++ot)
			stream << " " << ot->first << " " << ot->second << endl;
			for (auto it = ob->_children.begin(); it != ob->_children.end(); ++it)
			stream << (*it);
			}
			*/
			//boost::property_tree::xml_writer_settings<char> settings(' ', 4);
			//const char* spacer = " ";
			//boost::property_tree::xml_writer_settings <
			//	boost::property_tree::ptree::key_type > settings(spacer, 4);
			//boost::property_tree::write_xml(stream, pt, settings ); // , std::locale(), settings);
			boost::property_tree::write_xml(stream, pt); // , std::locale(), settings);

		}

		void configsegment::readBoost(boost::shared_ptr<configsegment> root, std::istream& indata, std::shared_ptr<registry::IO_options> opts)
		{
			// First, check that the file can be opened. If not, return NULL.
			using namespace std;
			using namespace boost::filesystem;
			using boost::property_tree::ptree;

			ptree pt;
			boost::property_tree::read_xml(indata, pt);

			std::map<size_t, boost::shared_ptr<configsegment> > encountered;
			size_t id_count = 0;
			// root should be constructed already

			// Just convert the ptree into configsegment objects
			readSegment(root, pt, encountered, id_count);
		}

		boost::shared_ptr<configsegment> configsegment::generate(const std::string &name)
		{
			boost::shared_ptr<configsegment> obj(new configsegment(name));
			return obj;
		}

		boost::shared_ptr<configsegment> configsegment::generate(const std::string &name, boost::shared_ptr<configsegment> &parent)
		{
			boost::shared_ptr<configsegment> obj(new configsegment(name));
			obj->_parents.insert(parent);
			parent->_children.insert(obj);
			return obj;
		}

		configsegment::configsegment(const std::string &name)
		{
			// No parent
			this->_segname = name;
		}

		configsegment::configsegment(const std::string &name, boost::shared_ptr<configsegment> &parent)
		{
			this->_segname = name;
			_parents.insert(parent);
			parent->_children.insert(getPtr());
		}

		configsegment::~configsegment()
		{
			// Thanks to shared_ptr, children will delete naturally when nothing holds them!
		}

		void configsegment::addChild(boost::shared_ptr<configsegment> child)
		{
			child->_parents.insert(shared_from_this());
			this->_children.insert(child);

		}

		void configsegment::removeChild(boost::shared_ptr<configsegment> child)
		{
			child->_parents.erase(shared_from_this());
			this->_children.erase(child);
		}

		void configsegment::uncouple()
		{
			for (auto &i : _parents)
			{
				if (!i.expired())
				{
					auto j = i.lock();
					j->removeChild(shared_from_this());
				}
			}
			_parents.clear();
		}

		boost::shared_ptr<configsegment> configsegment::getPtr() const
		{
			boost::shared_ptr<const configsegment> a = shared_from_this();
			return boost::const_pointer_cast<configsegment>(a);
		}

		/// Deprecated in shared_ptr conversion
		boost::shared_ptr<configsegment> configsegment::findSegment(const std::string &key) const
		{
			using namespace std;
			boost::shared_ptr<configsegment> cseg = getPtr();

			std::string dkey = key.substr(0, key.find_last_of('/') + 1);
			// Go down the tree, pulling out one '/' at a time, until done
			// If entry is missing, create it

			std::string segname;
			size_t s_start, s_end;
			s_start = 0;
			while ((s_end = dkey.find_first_of('/', s_start)) != std::string::npos)
			{
				segname = dkey.substr(s_start, s_end - s_start);
				if (segname.size() == 0) break;
				boost::shared_ptr<configsegment> newChild = cseg->getChild(segname);
				//if (newChild == nullptr) newChild = boost::shared_ptr<configsegment> (new configsegment(segname,cseg));
				if (newChild == nullptr) newChild = generate(segname, cseg);

				// Advance into the child
				cseg = newChild;
				// Increment counter
				if (s_start == s_end) s_end++;
				s_start = s_end;
			}

			// Done advancing. Return result.
			return cseg;
		}

		bool configsegment::hasVal(const std::string &key) const
		{
			using namespace std;
			// If this container does not have the value, look at the parent

			if (_mapStr.count(key))
			{
				return true;
			}
			return false;
		}

		bool configsegment::getVal(const std::string &key, std::string &value) const
		{
			using namespace std;
			if (_mapStr.count(key))
			{
				value = _mapStr.find(key)->second;
			}
			else
			{
				return false;
			}
			return true;
		}

		void configsegment::setVal(const std::string &key, const std::string &value)
		{
			using namespace std;// Set the value here. Overwrite any pre-existing value(s)
			if (this->_mapStr.count(key))
				_mapStr.erase(key);
			_mapStr.insert(std::pair<std::string,std::string>(key,value));
		}

		void configsegment::addVal(const std::string &key, const std::string &value)
		{
			using namespace std;
			_mapStr.insert(std::pair<std::string, std::string>(key, value));
		}

		boost::shared_ptr<configsegment> configsegment::getChild(const std::string &name) const
		{
			using namespace std;
			// Search through the child list to find the child
			for (auto it = _children.begin(); it != _children.end(); it++)
			{
				if ((*it)->_segname == name) return *it;
			}
			return nullptr;
		}

		std::set<boost::shared_ptr<configsegment> > configsegment::getParents() const
		{
			std::set<boost::shared_ptr<configsegment> > p;
			for (const auto &i : _parents)
			{
				if (!i.expired())
					p.insert(boost::shared_ptr<configsegment>(i));
			}
			return p;
		}

		void configsegment::name(std::string &res) const
		{
			res = _segname;
		}

		void configsegment::listKeys(std::multimap<std::string, std::string> &output) const
		{
			output = _mapStr;
		}

		void configsegment::listKeys(std::multiset<std::string> &res) const
		{
			res.clear();
			for (auto it = _mapStr.begin(); it != _mapStr.end(); it++)
				res.insert(it->first);
		}

		void configsegment::listChildren(std::multiset<std::string> &res) const
		{
			res.clear();
			for (auto it = _children.begin(); it != _children.end(); it++)
				res.insert((*it)->_segname);
		}

		void configsegment::listChildren(std::multiset<boost::shared_ptr<configsegment> > &res) const
		{
			res = _children;
		}


		void configsegment::getCWD(std::string &cwd) const
		{
			cwd = _cwd;
		}

		/*
		boost::shared_ptr<configsegment> configsegment::loadFile
			(const char* filename, boost::shared_ptr<configsegment> root)
		{
			using namespace std;
			using namespace boost::filesystem;

			// Use boost_filesystem as a file existence check
			boost::filesystem::path p(filename);
			boost::filesystem::path pabs(boost::filesystem::canonical(p));
			if (!exists(p)) RDthrow Ryan_Debug::debug::xMissingFile(pabs.string().c_str());
			if (is_directory(p)) RDthrow Ryan_Debug::debug::xMissingFile(pabs.string().c_str());
			ifstream indata(filename);
			if (!indata) RDthrow Ryan_Debug::debug::xOtherError();
			if (indata.good() == false) RDthrow Ryan_Debug::debug::xEmptyInputFile(filename);

			return loadFile(indata, root, filename);
			//return root;
		}
		*/

		void configsegment::writeOld(const boost::shared_ptr<const configsegment> ob, std::ostream & stream, std::shared_ptr<registry::IO_options>)
		{
			/// \todo allow for include statements and support symlinks. Will never implement, as this is the old file format.
			using namespace std;
			string name = ob->name();
			stream << "<" << name << ">" << endl;
			{
				/*
				for (auto ut = ob->_symlinks.begin(); ut != ob->_symlinks.end(); ++ut)
				{
					if ((*ut).expired() == false)
					{
						stream << (*ut).lock();
						//stream << (*ut);
					}
				}
				*/
				for (auto ot = ob->_mapStr.begin(); ot != ob->_mapStr.end(); ++ot)
					stream << " " << ot->first << " " << ot->second << endl;
				for (auto it = ob->_children.begin(); it != ob->_children.end(); ++it)
					stream << (*it);
			}
			stream << "</" << name << ">" << endl;
		}

		void configsegment::readOld
			(boost::shared_ptr<configsegment> root,
			std::istream &indata, std::shared_ptr<registry::IO_options> opts)
			//const std::string &cwd) --- cwd is now a parameter passed in opts
		{
			// This will load a file and tack it into the root (if specified)
			// If this is a new config tree, pass NULL as the root
			// If new tree, returns the new root. If not, returns root

			// First, check that the file can be opened. If not, return NULL.
			using namespace std;
			using namespace boost::filesystem;

			std::string cwd = opts->getVal<std::string>("cwd", "./");
			std::string fname = opts->getVal<std::string>("filename", "");
			//boost::shared_ptr<configsegment> root = getRtconfRoot();
			// Okay then. File is good. If no root, create it now.
			if (!root)
			{
				//root = getRtconfRoot();
				if (!root)
					root = boost::shared_ptr<configsegment>(new configsegment("Ryan_Debug"));
				//setRtconfRoot(root);
			}

			//auto cseg = root;
			boost::shared_ptr<configsegment> cseg = root; // The current container in the tree
			std::vector<boost::shared_ptr<configsegment> > pseg;
			if (cseg->_cwd.size() == 0) cseg->_cwd = cwd;

			size_t lnum = 0;
			// Read in each line, one at a time.
			// This is Apache-style, so tags in <> are containers, ended by </> tags.
			// Everything else is a key-value combination.
			// Keys are always one word long!
			while (indata.good())
			{
				std::string line, key;
				lnum++;
				std::getline(indata, line); // Read in the line
				std::istringstream linestream(line); // A string stream for the line
				if (line.size() == 0) continue; // Skip empty lines

				linestream >> key; // Do formatted read to avoid whitespace-trimming issues
				if (key.size() == 0) continue;
				if (key[0] == '#') continue; // A comment

				if (key[0] == '<') // A container is being defined or released
				{
					if (key[1] == '/')
					{
						// Close container
						if (!pseg.size()) 
							RDthrow(Ryan_Debug::error::xBadInput()) 
							<< Ryan_Debug::error::line_number(lnum)
							<< Ryan_Debug::error::file_name(fname);
						// Shouldn't happen unless syntax error
						cseg = *(pseg.rbegin());
						pseg.pop_back();
						if (!cseg) RDthrow(Ryan_Debug::error::xBadInput())
							<< Ryan_Debug::error::file_name(fname)
							<< Ryan_Debug::error::line_number(lnum);
						// Shouldn't happen unless syntax error
					}
					else {
						// New container
						// Remove spaces, tabs, < and > from original input line
						size_t kstart = line.find_first_of('<') + 1;
						size_t kend = line.find_last_of('>');
						line = line.substr(kstart, kend - kstart);

						// Now, create the new container and switch to it
						//boost::shared_ptr<configsegment> child (new configsegment(line, cseg));
						boost::shared_ptr<configsegment> child = generate(line, cseg);
						
						pseg.push_back(cseg);
						cseg = child;
					}
				}
				else {
					// A key-value combination is being entered
					// First part is the key! The rest is the value
					// Key is in key
					string value;
					size_t vstart = line.find(key) + key.size() + 1;
					size_t vend = line.find_last_not_of(' ') + 1;
					// Trim line to get value
					value = line.substr(vstart, vend - vstart);

					// Check for special keywords, like Include!
					//if (key == "Include")
					{
						/*
						// Use Boost to get the full path of the file (use appropriate dir)
						// rootpath now relative to current file being loaded, NOT tree root (cseg->_cwd) or ROOT
						boost::filesystem::path rootpath(cwd);
						boost::filesystem::path inclpath(value);
						if (inclpath.is_relative())
						{
							// The path on the Include is relative, so make it relative to the first loaded file, typically the root
							string newfile = (rootpath.parent_path() / value).string();
							if (!exists(path(newfile))) RDthrow Ryan_Debug::debug::xMissingFile(newfile.c_str());
							loadFile(newfile.c_str(), cseg);
						}
						else {
							// The path is absolute, so use it
							if (!exists(path(value.c_str()))) RDthrow Ryan_Debug::debug::xMissingFile(value.c_str());
							loadFile(value.c_str(), cseg); // Load a file
						}
					}
					else if (key == "IncludeIfExists") {
						// Use Boost to get the full path of the file (use appropriate dir)
						static boost::filesystem::path rootpath(cwd); // static, so it is called on the very first file
						boost::filesystem::path inclpath(value);
						if (inclpath.is_relative())
						{
							// The path on the Include is relative, so make it relative to the first loaded file, typically the root
							string newfile = (rootpath.parent_path() / value).string();
							if (!exists(path(newfile))) continue;
							loadFile(newfile.c_str(), cseg);
						}
						else {
							// The path is absolute, so use it
							if (!exists(path(value.c_str()))) continue;
							loadFile(value.c_str(), cseg); // Load a file
						}

					}
					else {
						*/
						// Set the key-val combination
						cseg->setVal(key, value);
					}
				}

			}
		}

		/**
		* \brief Function that returns the location of the Ryan_Debug.conf file
		*
		* Finding the default config file has become a rather involved process.
		* First, check the application execution arguments (if using appEntry).
		* Second, check the environment variables. Uses the key Ryan_Debug_CONF, and accepts
		* multiple files, separated by semicolons. Searches for file existence in 
		* left-to-right order.
		*
		* \todo Third, check the system registry (if using Windows).
		*
		* Finally, check using the precompiled paths.
		**/
		void getConfigDefaultFile(std::string &filename)
		{
			filename = "";
			using namespace boost::filesystem;
			auto& lg = Ryan_Debug::config::m_config::get();

			BOOST_LOG_SEV(lg, Ryan_Debug::log::notification) << "Finding Ryan_Debug configuration file";

			// Check application execution arguments
			BOOST_LOG_SEV(lg, Ryan_Debug::log::debug_2) << "Checking app command line";
			path testCMD(Ryan_Debug::debug::sConfigDefaultFile);
			if (exists(testCMD))
			{
				filename = Ryan_Debug::debug::sConfigDefaultFile;
				BOOST_LOG_SEV(lg, Ryan_Debug::log::debug_2) << filename;
				return;
			}

			// Checking environment variables
			{
				using namespace Ryan_Debug;
				boost::shared_ptr<const processInfo> info(getInfo(getPID()), freeProcessInfo);

				BOOST_LOG_SEV(lg, Ryan_Debug::log::debug_2) << "Checking Ryan_Debug_conf environment variable";
				size_t sEnv = 0;
				const char* cenv = getEnviron(info.get(), sEnv);
				std::string env(cenv,sEnv);

				//Ryan_Debug::processInfo info = Ryan_Debug::getInfo(Ryan_Debug::getPID());
				std::map<std::string, std::string> mEnv;
				splitSet::splitNullMap(env, mEnv);
				//std::vector<std::string> mCands;
				auto it = std::find_if(mEnv.cbegin(), mEnv.cend(),
					[](const std::pair<std::string, std::string> &pred)
				{
					std::string key = pred.first;
					std::transform(key.begin(), key.end(), key.begin(), ::tolower);
					if (key == "Ryan_Debug_conf") return true;
					return false;
				});
				if (it != mEnv.cend())
				{
					typedef boost::tokenizer<boost::char_separator<char> >
						tokenizer;
					boost::char_separator<char> sep(";");

					BOOST_LOG_SEV(lg, Ryan_Debug::log::debug_2) << "Candidates are: " << it->second;
					std::string ssubst;
					tokenizer tcom(it->second, sep);
					for (auto ot = tcom.begin(); ot != tcom.end(); ot++)
					{
						path testEnv(it->second);
						if (exists(testEnv))
						{
							filename = it->second;
							BOOST_LOG_SEV(lg, Ryan_Debug::log::debug_2) << "Using " << filename;
							return;
						}
					}
				}
			}

			// Check the system registry
			// TODO

			// Check a few other places
			std::string sAppConfigDir (Ryan_Debug::getAppConfigDir());
			std::string sHomeDir(Ryan_Debug::getHomeDir());
			auto hm = boost::shared_ptr<const moduleInfo>(getModuleInfo((void*) &getConfigDefaultFile), freeModuleInfo);
			std::string dllPath(getPath(hm.get()));

			auto hp = boost::shared_ptr<const processInfo>(Ryan_Debug::getInfo(Ryan_Debug::getPID()), freeProcessInfo);
			std::string appPath(getPath(hp.get()));

			std::string sCWD(Ryan_Debug::getCwd(hp.get()));

			BOOST_LOG_SEV(lg, Ryan_Debug::log::debug_2) << "Checking app data directory: ";
			// For all of these places, search for file names matching Ryan_Debug.xml, Ryan_Debug.conf and .Ryan_Debug.
			// Compression is allowed.
			auto searchPath = [&](const std::string &base, const std::string &suffix, bool searchParent) -> bool
			{
				using namespace boost::filesystem;
				path pBase(base);
				BOOST_LOG_SEV(lg, Ryan_Debug::log::debug_2) << "Getting search path based on: " << pBase.string();
				if (!is_directory(pBase))
					pBase.remove_filename();
				if (searchParent) pBase.remove_leaf();
				if (suffix.size()) pBase = pBase / path(suffix);

				BOOST_LOG_SEV(lg, Ryan_Debug::log::debug_2) << "Searching in: " << pBase.string();

				path p1 = pBase / "Ryan_Debug.xml";
				path p2 = pBase / "Ryan_Debug.conf";
				path p3 = pBase / ".Ryan_Debug";

				bool res = false;
				path pRes;
				std::string meth;
				res = Ryan_Debug::serialization::detect_compressed<path>(p1, meth, pRes);
				if (!res) res = Ryan_Debug::serialization::detect_compressed<path>(p2, meth, pRes);
				if (!res) res = Ryan_Debug::serialization::detect_compressed<path>(p3, meth, pRes);
				if (!res) return false;
				filename = pRes.string();
				return true;
			};
			bool found = false; // junk variable

			if (searchPath(sCWD, "", true)) found = true;
			else if (searchPath(sAppConfigDir, "Ryan_Debug", false)) found = true;
			else if (searchPath(sHomeDir, "", false)) found = true;
			else if (searchPath(dllPath, "", true)) found = true;
			else if (searchPath(appPath, "", true)) found = true;

			if (filename.size()) {
				BOOST_LOG_SEV(lg, Ryan_Debug::log::debug_2) << "Using conf file: " << filename;
				return;
			}


			// Finally, just use the default os-dependent path
			//filename = "/home/rhoneyag/.Ryan_Debug";
			// Macro defining the correct path
			BOOST_LOG_SEV(lg, Ryan_Debug::log::debug_2) << "Checking compile-time paths: "
				//<< "RTC: " << RTC << "\nRTCB: " << RTCB << "\nRTCC: " << RTCC
				<< "\nSYS_RTC: " << SYS_RTC;
			//path testUser(RTC);
			//path testUserB(RTCB);
			//path testUserC(RTCC);
			path testSys(SYS_RTC);
			//if (exists(testUser))
			//	filename = RTC;
			//else if (exists(testUserB))
			//	filename = RTCB;
			//else if (exists(testUserC))
			//	filename = RTCC;
			if (exists(testSys))
				filename = SYS_RTC;
			if (filename.size()) BOOST_LOG_SEV(lg, Ryan_Debug::log::debug_2) << "Using conf file: " << filename;
			else BOOST_LOG_SEV(lg, Ryan_Debug::log::critical) << "Unable to find Ryan_Debug configuration file. "
				<< "Log channel config at severity debug_2 lists the searched paths. You can specify the file by "
				"command-line (option --Ryan_Debug-config-file), environment variable (Ryan_Debug_conf), "
				"or place one in an at-compile-time-specified path.";

			return;
		}

		boost::shared_ptr<configsegment> getRtconfRoot()
		{
			return _rtconfroot;
		}

		void setRtconfRoot(boost::shared_ptr<configsegment> &root)
		{
			_rtconfroot = root;
		}

		boost::shared_ptr<configsegment> loadRtconfRoot(const std::string &filename)
		{
			if (_rtconfroot != nullptr) return _rtconfroot;
			auto& lg = Ryan_Debug::config::m_config::get();
			BOOST_LOG_SEV(lg, Ryan_Debug::log::normal) << "Loading Ryan_Debug config file.";
			if (filename.size())
				BOOST_LOG_SEV(lg, Ryan_Debug::log::normal) << "Override filename " << filename;
			std::string fn = filename;
			if (!fn.size()) getConfigDefaultFile(fn);
			if (fn.size()) BOOST_LOG_SEV(lg, Ryan_Debug::log::normal) << "Found Ryan_Debug config file " << fn;
			else {
				BOOST_LOG_SEV(lg, Ryan_Debug::log::critical) << "Cannot find Ryan_Debug config file!";
				RDthrow(error::xMissingRyan_DebugConf())
					<< error::file_name(filename) << error::default_file_name(fn);
			}
			//boost::shared_ptr<configsegment> cnf = configsegment::loadFile(fn.c_str(), nullptr);
			auto opts = Ryan_Debug::registry::IO_options::generate(Ryan_Debug::registry::IOhandler::IOtype::READONLY);
			opts->filename(fn);

			std::vector< boost::shared_ptr<configsegment> > rootcands;
			configsegment::readVector(nullptr, opts, rootcands, nullptr);
			boost::shared_ptr<configsegment> cnf;
			for (const auto &r : rootcands)
			{
				if (r->name() == "Ryan_Debug" || r->name() == "ROOT" || (r->name() == "" && rootcands.size() == 1)) cnf = r;
			}
			if (cnf) _rtconfroot = cnf;
			return cnf;
		}

		/*
		std::ostream& operator<< (std::ostream& stream, const ::Ryan_Debug::config::configsegment &ob)
		{
			/// \todo allow for include statements
			using namespace std;
			string name = ob.name();
			stream << "<" << name << ">" << endl;
			{
				for (auto ut = ob._symlinks.begin(); ut != ob._symlinks.end(); ++ut)
				{
					if ((*ut).expired() == false)
					{
						stream << (*ut).lock();
						//stream << (*ut);
					}
				}
				for (auto ot = ob._mapStr.begin(); ot != ob._mapStr.end(); ++ot)
					stream << " " << ot->first << " " << ot->second << endl;
				for (auto it = ob._children.begin(); it != ob._children.end(); ++it)
					stream << (*it);
			}
			stream << "</" << name << ">" << endl;
			return stream;
		}
		*/

	}
}

