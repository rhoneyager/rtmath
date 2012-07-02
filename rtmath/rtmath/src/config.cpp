/*
 * config.cpp
 *
 *  Created on: Jun 24, 2011
 *      Author: reh9650
 */
#include "../rtmath/Stdafx.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <boost/filesystem.hpp>
#include "../rtmath/config.h"
#include "../rtmath/error/debug.h"

// Special compile-time generated files that build needs
#include "debug_subversion.h"

#ifdef WITH_CMAKE
#include "cmake-settings.h"
#else
#define SYS_RTC RTC
#endif


namespace rtmath {
	namespace config {

		std::shared_ptr<configsegment> configsegment::create(const std::string &name)
		{
			std::shared_ptr<configsegment> obj(new configsegment(name));
			return obj;
		}

		std::shared_ptr<configsegment> configsegment::create(const std::string &name, std::shared_ptr<configsegment> &parent)
		{
			std::shared_ptr<configsegment> obj(new configsegment(name));
			obj->_parent = parent;
			parent->_children.insert(obj);
			return obj;
		}

		configsegment::configsegment(const std::string &name)
		{
			// No parent
			this->_segname = name;
		}

		configsegment::configsegment(const std::string &name, std::shared_ptr<configsegment> &parent)
		{
			this->_segname = name;
			_parent = parent;
			parent->_children.insert(getPtr());
		}

		configsegment::~configsegment()
		{
			// Thanks to shared_ptr, children will delete naturally when nothing holds them!
		}

		std::shared_ptr<configsegment> configsegment::getPtr() const
		{
			std::shared_ptr<const configsegment> a = shared_from_this();
			return std::const_pointer_cast<configsegment>(a);
		}

		// Deprecated in shared_ptr conversion
		std::shared_ptr<configsegment> configsegment::findSegment(const std::string &key) const
		{
			using namespace std;
			std::shared_ptr<configsegment> cseg = getPtr();

			// If first part of key is '/', seek to root
			
			if (key[0] == '/')
			{
				std::shared_ptr<configsegment> cpar = this->_parent.lock();
				while (cpar.use_count())
				{
					cseg = cpar;
					cpar = cseg->_parent.lock();
				}
				cseg = cpar;
			}
			

			std::string dkey = key.substr(0,key.find_last_of('/')+1);
			// Go down the tree, pulling out one '/' at a time, until done
			// If entry is missing, create it

			std::string segname;
			size_t s_start, s_end;
			s_start = 0;
			while ( (s_end = dkey.find_first_of('/',s_start)) != std::string::npos)
			{
				segname = dkey.substr(s_start,s_end-s_start);
				if (segname.size() == 0) break;
				std::shared_ptr<configsegment> newChild = cseg->getChild(segname);
				//if (newChild == nullptr) newChild = std::shared_ptr<configsegment> (new configsegment(segname,cseg));
				if (newChild == nullptr) newChild = create(segname,cseg);

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
			// If the key contains '/', we should search the path
			// a / at the beginning specifies an absolute path.
			// Otherwise, it is relative only going downwards
			if (key.find('/') != string::npos)
			{
				std::shared_ptr<configsegment> relseg = findSegment(key);
				if (relseg == NULL) throw;
				// keystripped is the key without the path. If ends in /, an error will occur
				string keystripped = key.substr(key.find_last_of('/')+1, key.size());
				bool res = relseg->hasVal(keystripped);
				return res;
			}

			// Let's be promiscuous!
			// If this container does not have the value, look at the parent

			if (_mapStr.count(key))
			{
				return true;
			}
			else
			{
				if (this->_parent.expired() == false)
				{
					return this->_parent.lock()->hasVal(key);
				} else {
					return false;
				}
			}
			return false;
		}

		bool configsegment::getVal(const std::string &key, std::string &value) const
		{
			using namespace std;
			// If the key contains '/', we should search the path
			// a / at the beginning specifies an absolute path.
			// Otherwise, it is relative only going downwards
			if (key.find('/') != string::npos)
			{
				std::shared_ptr<configsegment> relseg = findSegment(key);
				if (relseg == NULL) throw;
				// keystripped is the key without the path. If ends in /, an error will occur
				string keystripped = key.substr(key.find_last_of('/')+1, key.size());
				bool res = relseg->getVal(keystripped,value);
				return res;
			}

			// Let's be promiscuous!
			// If this container does not have the value, look at the parent

			if (_mapStr.count(key))
			{
				value = _mapStr.at(key);
			}
			else
			{
				if (this->_parent.expired() == false)
				{
					this->_parent.lock()->getVal(key, value);
					return true;
				} else {
					return false;
				}
			}
			return false;
		}

		void configsegment::setVal(const std::string &key, const std::string &value)
		{
			using namespace std;
			if (key.find('/') != string::npos)
			{
				std::shared_ptr<configsegment> relseg = findSegment(key);
				string keystripped = key.substr(key.find_last_of('/')+1, key.size());
				relseg->setVal(keystripped,value);
				return;
			}
			// Set the value here. Overwrite any pre-existing value
			if (this->_mapStr.count(key))
				_mapStr.erase(key);
			_mapStr[key] = value;
		}

		std::shared_ptr<configsegment> configsegment::getChild(const std::string &name) const
		{
			using namespace std;
			if (name.find('/') != string::npos)
			{
				std::shared_ptr<configsegment> relseg = findSegment(name);
				return relseg;
			}

			// Search through the child list to find the child
			for (auto it = _children.begin(); it != _children.end(); it++)
			{
				if ((*it)->_segname == name) return *it;
			}
			return NULL;
		}

		std::shared_ptr<configsegment> configsegment::getParent() const
		{
			if (_parent.expired()) return nullptr;
			return std::shared_ptr<configsegment>(_parent);
		}

		void configsegment::name(std::string &res) const
		{
			res = _segname;
		}

		void configsegment::listKeys(std::map<std::string, std::string> &output) const
		{
			output = _mapStr;
		}

		void configsegment::listKeys(std::set<std::string> &res) const
		{
			res.clear();
			for (auto it = _mapStr.begin(); it != _mapStr.end(); it++)
				res.insert( it->first );
		}

		void configsegment::listChildren(std::set<std::string> &res) const
		{
			res.clear();
			for (auto it = _children.begin(); it != _children.end(); it++)
				res.insert( (*it)->_segname );
		}

		std::shared_ptr<configsegment> configsegment::loadFile
			(const char* filename, std::shared_ptr<configsegment> root)
		{
			using namespace std;
			using namespace boost::filesystem;

			// Use boost_filesystem as a file existence check
			boost::filesystem::path p(filename);
			if (!exists(p)) throw rtmath::debug::xMissingFile(filename);
			if (is_directory(p)) throw rtmath::debug::xMissingFile(filename);
			ifstream indata(filename);
			if (!indata) throw rtmath::debug::xOtherError();
			if (indata.good() == false) throw rtmath::debug::xEmptyInputFile(filename);

			return loadFile(indata, root, filename);
			//return root;
		}

		std::shared_ptr<configsegment> configsegment::loadFile
			(std::istream &indata, std::shared_ptr<configsegment> root, const std::string &cwd)
		{
			// This will load a file and tack it into the root (if specified)
			// If this is a new config tree, pass NULL as the root
			// If new tree, returns the new root. If not, returns root

			// First, check that the file can be opened. If not, return NULL.
			using namespace std;
			using namespace boost::filesystem;

			// Okay then. File is good. If no root, create it now.
			if (!root)
				//root = std::shared_ptr<configsegment>(new configsegment("ROOT"));
				root = create("ROOT");

			std::shared_ptr<configsegment> cseg = root; // The current container in the tree

			// Read in each line, one at a time.
			// This is Apache-style, so tags in <> are containers, ended by </> tags.
			// Everything else is a key-value combination.
			// Keys are always one word long!
			while (indata.good())
			{
				std::string line, key;
				std::getline(indata,line); // Read in the line
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
						cseg = cseg->getParent();
						if (cseg == NULL) throw; // Shouldn't happen unless syntax error
					} else {
						// New container
						// Remove spaces, tabs, < and > from original input line
						size_t kstart = line.find_first_of('<')+1;
						size_t kend = line.find_last_of('>');
						line = line.substr(kstart,kend-kstart);

						// Now, create the new container and switch to it
						//std::shared_ptr<configsegment> child (new configsegment(line, cseg));
						std::shared_ptr<configsegment> child = create(line,cseg);
						cseg = child;
					}
				} else {
					// A key-value combination is being entered
					// First part is the key! The rest is the value
					// Key is in key
					string value;
					size_t vstart = line.find(key) + key.size()+1;
					size_t vend = line.find_last_not_of(' ')+1;
					// Trim line to get value
					value = line.substr(vstart, vend - vstart);

					// Check for special keywords, like Include!
					if (key == "Include")
					{
						// Use Boost to get the full path of the file (use appropriate dir)
						static boost::filesystem::path rootpath(cwd); // static, so it is called on the very first file
						boost::filesystem::path inclpath(value);
						if (inclpath.is_relative())
						{
							// The path on the Include is relative, so make it relative to the first loaded file, typically the root
							string newfile = (rootpath.parent_path() / value).string();
							if (!exists(path(newfile))) throw rtmath::debug::xMissingFile( newfile.c_str() );
							loadFile( newfile.c_str(), cseg);
						} else {
							// The path is absolute, so use it
							if (!exists(path(value.c_str()))) throw rtmath::debug::xMissingFile( value.c_str());
							loadFile(value.c_str(), cseg); // Load a file
						}
					} else if (key == "IncludeIfExists") {
						// Use Boost to get the full path of the file (use appropriate dir)
						static boost::filesystem::path rootpath(cwd); // static, so it is called on the very first file
						boost::filesystem::path inclpath(value);
						if (inclpath.is_relative())
						{
							// The path on the Include is relative, so make it relative to the first loaded file, typically the root
							string newfile = (rootpath.parent_path() / value).string();
							if (!exists(path(newfile))) continue;
							loadFile( newfile.c_str(), cseg);
						} else {
							// The path is absolute, so use it
							if (!exists(path(value.c_str()))) continue;
							loadFile(value.c_str(), cseg); // Load a file
						}

					} else {
						// Set the key-val combination
						cseg->setVal(key,value);
					}
				}

			}
			return root;
		}

		void configsegment::move(std::shared_ptr<configsegment> &newparent)
		{
			std::shared_ptr<configsegment> me = getPtr();
			if (_parent.expired() == false)
			{
				// Parent exists. Free child reference.
				_parent.lock()->_children.erase(me);
			}
			_parent = newparent;
			newparent->_children.insert(me);
		}

		void getConfigDefaultFile(std::string &filename)
		{
			// Finding the default config file has become a rather involved process.
			// First, check the application execution arguments (if using appEntry).
			// Then, check the system registry (if using Windows).
			// Third, check the system environment variables

			// Finally, just use the default os-dependent path
			//filename = "/home/rhoneyag/.rtmath";
			// Macro defining the correct path
			filename = "";
			using namespace boost::filesystem;
			path testUser(RTC);
			path testSys(SYS_RTC);
			if (exists(testUser))
				filename = RTC;
			else if (exists(testSys))
				filename = SYS_RTC;
			
			return;
		}

		std::shared_ptr<configsegment> _rtconfroot = nullptr;

		std::shared_ptr<configsegment> getRtconfRoot()
		{
			return _rtconfroot;
		}

		void setRtconfRoot(std::shared_ptr<configsegment> &root)
		{
			_rtconfroot = root;
		}

		std::shared_ptr<configsegment> loadRtconfRoot(const std::string &filename)
		{
			if (_rtconfroot != nullptr) return _rtconfroot;
			std::string fn = filename;
			if (fn == "") getConfigDefaultFile(fn);
			std::shared_ptr<configsegment> cnf = configsegment::loadFile(fn.c_str(), nullptr);
			if (cnf != nullptr) _rtconfroot = cnf;
			return cnf;
		}

		std::ostream& operator<< (std::ostream& stream, const rtmath::config::configsegment &ob)
		{
			// Take the object, and print in the appropriate form, using recursion
			// TODO: allow for include statements
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
					stream << " " << ot->first << " " << ot->second<< endl;
				for (auto it = ob._children.begin(); it != ob._children.end(); ++it)
					stream << (*it);
			}
			stream << "</" << name << ">" << endl;
			return stream;
		}


		std::istream& operator>> (std::istream &stream, std::shared_ptr<rtmath::config::configsegment> &ob)
		{
			//std::string fname;
			//stream >> fname;
			//ob = rtmath::config::configsegment::loadFile(fname.c_str(),ob);
			ob = rtmath::config::configsegment::loadFile(stream,nullptr);
			return stream;
		}

	}; // end config
};// end rtmath


