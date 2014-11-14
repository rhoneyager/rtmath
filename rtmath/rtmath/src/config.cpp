#include "Stdafx-core.h"
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

#include <Ryan_Debug/debug.h>

#include "../rtmath/config.h"
#include "../rtmath/splitSet.h"
#include "../rtmath/Serialization/Serialization.h"
#include "../rtmath/error/debug.h"
#include "../rtmath/error/error.h"


// Special compile-time generated files that build needs
#include "debug_subversion.h"

#ifdef WITH_CMAKE
#include "cmake-settings.h"
#else
#define SYS_RTC RTC
#endif

namespace {
	std::set<std::string> mtypes, mnewtypes;
	std::mutex cmlock;

	boost::shared_ptr<::rtmath::config::configsegment> _rtconfroot = nullptr;

	

	void writeSegment(
		const boost::shared_ptr<const rtmath::config::configsegment> it, 
		boost::property_tree::ptree& parent,
		std::map<boost::shared_ptr<const rtmath::config::configsegment>, size_t > &encountered,
		size_t &id_count)
	{
		using namespace boost::property_tree;
		//path.push_back(it->name());
		auto makeId = [&]() -> size_t {
			id_count++;
			return id_count;
		};

		std::string name = it->name();
		if (!name.size()) name = "RTMATH";
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
		boost::shared_ptr<rtmath::config::configsegment> cs,
		boost::property_tree::ptree& pobj,
		std::map< size_t, boost::shared_ptr<rtmath::config::configsegment> > &encountered,
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
					if (!encountered.count(refId)) RTthrow rtmath::debug::xArrayOutOfBounds();
					cs->addChild(encountered.at(refId));
				} else if (sdata.find("include:") == 0) {
					std::string sid = sdata.substr(8);
					boost::filesystem::path pInc(sid);
					boost::filesystem::path pIncSym = rtmath::debug::expandSymlink(pInc);

					auto doReadFile = [&](const boost::filesystem::path &p)
					{
						auto ncs = cs->generate(it->first, cs);
						size_t cid_count = 0;
						std::map< size_t, boost::shared_ptr<rtmath::config::configsegment> > cenc;

						boost::property_tree::ptree pt_child;
						// Read file into stream. Handle compression.
						std::string sfile;
						rtmath::serialization::read(sfile, p.string());
						std::istringstream sin(sfile);
						boost::property_tree::read_xml(sin, pt_child);
						readSegment(ncs, pt_child, cenc, cid_count);
					};

					if (boost::filesystem::is_directory(pIncSym))
					{
						std::vector<boost::filesystem::path> vs;
						rtmath::debug::expandFolder(pIncSym, vs, false);
						for (const auto & v : vs)
						{
							auto sym = rtmath::debug::expandSymlink(v);
							boost::filesystem::path sym_unc; std::string cmeth;
							rtmath::serialization::uncompressed_name(sym, sym_unc, cmeth);
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

namespace rtmath {
	namespace registry {
		template struct IO_class_registry_writer
			<::rtmath::config::configsegment>;

		template struct IO_class_registry_reader
			<::rtmath::config::configsegment>;

		template class usesDLLregistry<
			::rtmath::config::configsegment_IO_output_registry,
			IO_class_registry_writer<::rtmath::config::configsegment> >;

		template class usesDLLregistry<
			::rtmath::config::configsegment_IO_input_registry,
			IO_class_registry_reader<::rtmath::config::configsegment> >;
	}

	namespace io {
		template <>
		boost::shared_ptr<::rtmath::config::configsegment> customGenerator()
		{
			boost::shared_ptr<::rtmath::config::configsegment> res
				(new ::rtmath::config::configsegment(""));
			return res;
		}
	}
	namespace config {

		implementsConfigOld::implementsConfigOld() :
			rtmath::io::implementsIObasic<configsegment, configsegment_IO_output_registry,
			configsegment_IO_input_registry, configsegment_OldStandard>(configsegment::writeOld, configsegment::readOld, known_formats())
		{
			auto& lg = rtmath::io::m_io::get();
			BOOST_LOG_SEV(lg, rtmath::debug::normal) << "Just registered config - implementsConfigOld\n";

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
					mtypes.insert(".rtmath");
					mtypes.insert("rtmath.conf");
					if (io::TextFiles::serialization_handle::compressionEnabled())
					{
						std::string sctypes;
						std::set<std::string> ctypes;
						serialization::known_compressions(sctypes, ".rtmath");
						serialization::known_compressions(sctypes, "rtmath.conf");
						rtmath::config::splitSet(sctypes, ctypes);
						for (const auto & t : ctypes)
							mtypes.emplace(t);
					}
				}
			}
			return mtypes;
		}

		implementsConfigBoost::implementsConfigBoost() :
			rtmath::io::implementsIObasic<configsegment, configsegment_IO_output_registry,
			configsegment_IO_input_registry, configsegment_Boost>(configsegment::writeBoost, configsegment::readBoost, known_formats())
		{
			auto& lg = rtmath::io::m_io::get();
			BOOST_LOG_SEV(lg, rtmath::debug::normal) << "Just registered config - implementsConfigBoost\n";
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
						rtmath::config::splitSet(sctypes, ctypes);
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
			boost::property_tree::xml_writer_settings<char> settings(' ', 4);
			boost::property_tree::write_xml(stream, pt, settings ); // , std::locale(), settings);
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
			if (!exists(p)) RTthrow rtmath::debug::xMissingFile(pabs.string().c_str());
			if (is_directory(p)) RTthrow rtmath::debug::xMissingFile(pabs.string().c_str());
			ifstream indata(filename);
			if (!indata) RTthrow rtmath::debug::xOtherError();
			if (indata.good() == false) RTthrow rtmath::debug::xEmptyInputFile(filename);

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
			//boost::shared_ptr<configsegment> root = getRtconfRoot();
			// Okay then. File is good. If no root, create it now.
			if (!root)
			{
				//root = getRtconfRoot();
				if (!root)
					root = boost::shared_ptr<configsegment>(new configsegment("RTMATH"));
				//setRtconfRoot(root);
			}

			//auto cseg = root;
			boost::shared_ptr<configsegment> cseg = root; // The current container in the tree
			std::vector<boost::shared_ptr<configsegment> > pseg;
			if (cseg->_cwd.size() == 0) cseg->_cwd = cwd;

			// Read in each line, one at a time.
			// This is Apache-style, so tags in <> are containers, ended by </> tags.
			// Everything else is a key-value combination.
			// Keys are always one word long!
			while (indata.good())
			{
				std::string line, key;
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
						if (!pseg.size()) RTthrow rtmath::debug::xOtherError(); // Shouldn't happen unless syntax error
						cseg = *(pseg.rbegin());
						pseg.pop_back();
						if (!cseg) RTthrow rtmath::debug::xOtherError(); // Shouldn't happen unless syntax error
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
							if (!exists(path(newfile))) RTthrow rtmath::debug::xMissingFile(newfile.c_str());
							loadFile(newfile.c_str(), cseg);
						}
						else {
							// The path is absolute, so use it
							if (!exists(path(value.c_str()))) RTthrow rtmath::debug::xMissingFile(value.c_str());
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
		* \brief Function that returns the location of the rtmath.conf file
		*
		* Finding the default config file has become a rather involved process.
		* First, check the application execution arguments (if using appEntry).
		* Second, check the environment variables. Uses the key RTMATH_CONF, and accepts
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

			// Check application execution arguments
			path testCMD(rtmath::debug::sConfigDefaultFile);
			if (exists(testCMD))
			{
				filename = rtmath::debug::sConfigDefaultFile;
				return;
			}

			// Checking environment variables
			{
				using namespace Ryan_Debug;
				boost::shared_ptr<const processInfo> info(getInfo(getPID()), freeProcessInfo);

				size_t sEnv = 0;
				const char* cenv = getEnviron(info.get(), sEnv);
				std::string env(cenv,sEnv);

				//Ryan_Debug::processInfo info = Ryan_Debug::getInfo(Ryan_Debug::getPID());
				std::map<std::string, std::string> mEnv;
				config::splitNullMap(env, mEnv);
				//std::vector<std::string> mCands;
				auto it = std::find_if(mEnv.cbegin(), mEnv.cend(),
					[](const std::pair<std::string, std::string> &pred)
				{
					std::string key = pred.first;
					std::transform(key.begin(), key.end(), key.begin(), ::tolower);
					if (key == "rtmath_conf") return true;
					return false;
				});
				if (it != mEnv.cend())
				{
					typedef boost::tokenizer<boost::char_separator<char> >
						tokenizer;
					boost::char_separator<char> sep(";");

					std::string ssubst;
					tokenizer tcom(it->second, sep);
					for (auto ot = tcom.begin(); ot != tcom.end(); ot++)
					{
						path testEnv(it->second);
						if (exists(testEnv))
						{
							filename = it->second;
							return;
						}
					}
				}
			}

			// Check the system registry
			// TODO

			// Finally, just use the default os-dependent path
			//filename = "/home/rhoneyag/.rtmath";
			// Macro defining the correct path
			path testUser(RTC);
			path testUserB(RTCB);
			path testUserC(RTCC);
			path testSys(SYS_RTC);
			if (exists(testUser))
				filename = RTC;
			else if (exists(testUserB))
				filename = RTCB;
			else if (exists(testUserC))
				filename = RTCC;
			else if (exists(testSys))
				filename = SYS_RTC;

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
			std::string fn = filename;
			if (fn == "") getConfigDefaultFile(fn);
			if (fn == "") RTthrow debug::xMissingFile("Cannot find the rtmath.conf file") 
				<< debug::file_name(filename) << debug::default_file_name(fn);
			//boost::shared_ptr<configsegment> cnf = configsegment::loadFile(fn.c_str(), nullptr);
			auto opts = rtmath::registry::IO_options::generate(rtmath::registry::IOhandler::IOtype::READONLY);
			opts->filename(fn);

			std::vector< boost::shared_ptr<configsegment> > rootcands;
			configsegment::readVector(nullptr, opts, rootcands, nullptr);
			boost::shared_ptr<configsegment> cnf;
			for (const auto &r : rootcands)
			{
				if (r->name() == "RTMATH" || r->name() == "ROOT" || (r->name() == "" && rootcands.size() == 1)) cnf = r;
			}
			if (cnf) _rtconfroot = cnf;
			return cnf;
		}

		/*
		std::ostream& operator<< (std::ostream& stream, const ::rtmath::config::configsegment &ob)
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

