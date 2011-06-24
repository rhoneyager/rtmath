/*
 * config.cpp
 *
 *  Created on: Jun 24, 2011
 *      Author: reh9650
 */
#include "Stdafx.h"
#include "config.h"

#include <iostream>
#include <fstream>
#include <sstream>

namespace rtmath {
	namespace config {
		configsegment::configsegment(const std::string &name)
		{
			// No parent
			this->_parent = NULL;
		}

		configsegment::configsegment(const std::string &name, configsegment *parent)
		{
			_parent = parent;
			parent->_children.insert(this);
		}

		configsegment* configsegment::findSegment(const std::string &key) const
		{
			using namespace std;
			configsegment *cseg = this;

			// If first part of key is '/', seek to root
			if (key[0] == '/')
			{
				configsegment cpar = this->_parent;
				while (cpar != NULL)
				{
					cseg = cpar;
					cpar = cseg->_parent;
				}
				cseg = cpar;
			}

			std::string dkey = key.substr(0,key.find_last_not_of('/'));
			// Go down the tree, pulling out one '/' at a time, until done
			// If an entry is missing, create the container

			std::string segname;
			size_t s_start, s_end;
			s_start = 0;
			while ( (s_end = dkey.find_first_of('/',s_start)) != std::string::npos)
			{
				segname = dkey.substr(s_start,s_end-s_start);
				if (segname.size() == 0) continue;
				configsegment *newChild = cseg->getChild(segname);
				if (newChild == NULL) newChild = new configsegment(segname,cseg);

				// Advance into the child
				cseg = newChild;
				// Increment counter
				if (s_start == s_end) s_end++;
				s_start = s_end;
			}

			// Done advancing. Return result.
			return cseg;
		}
		void configsegment::getVal(const std::string &key, std::string &value) const
		{
			using namespace std;
			// If the key contains '/', we should search the path
			// a / at the beginning specifies an absolute path.
			// Otherwise, it is relative only going downwards
			if (key.find('/'))
			{
				configsegment *relseg = findSegment(key);
				if (relseg == NULL) throw;
				// keystripped is the key without the path. If ends in /, an error will occur
				string keystripped = key.substr(key.find_last_of('/')+1, key.size());
				relseg->getVal(keystripped,value);
				return;
			}

			// Let's be promiscuous!
			// If this container does not have the value, look at the parent

			if (_mapStr.count(key))
			{
				value = _mapStr[key];
			}
			else
			{
				if (this->_parent)
				{
					this->_parent->getVal(key, value);
				} else {
					value.clear(); // Return an empty value
				}
			}
		}

		void configsegment::setVal(const std::string &key, const std::string &value)
		{
			using namespace std;
			if (key.find('/'))
			{
				configsegment *relseg = findSegment(key);
				string keystripped = key.substr(key.find_last_of('/')+1, key.size());
				relseg->setVal(keystripped,value);
				return;
			}
			// Set the value here. Overwrite any pre-existing value
			if (this->_mapStr.count(key))
				_mapStr.erase(key);
			_mapStr[key] = value;
		}

		configsegment* configsegment::getChild(const std::string &name) const
		{
			using namespace std;
			if (name.find('/'))
			{
				configsegment *relseg = findSegment(name);
				return relseg;
			}

			// Search through the child list to find the child
			std::set<configsegment*>::const_iterator it;
			for (it = _children.begin(); it != _children.end(); it++)
			{
				if (it->_segname == name) return *it;
			}
			return NULL;
		}

		configsegment* configsegment::getParent() const
		{
			return _parent;
		}

		configsegment* configsegment::loadFile(const char* filename, configsegment* root)
		{
			// This will load a file and tack it into the root (if specified)
			// If this is a new config tree, pass NULL as the root
			// If new tree, returns the new root. If not, returns root

			// First, check that the file can be opened. If not, return NULL.
			using namespace std;

			ifstream indata(filename);
			if (indata.good() == false) return NULL;

			// Okay then. File is good. If no root, create it now.
			if (!root)
				root = new configsegment("ROOT",NULL);

			configsegment* cseg = root; // The current container in the tree

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
						std::remove(line.begin(), line.end(), ' ');
						std::remove(line.begin(), line.end(), '\t');
						std::remove(line.begin(), line.end(), '<');
						std::remove(line.begin(), line.end(), '>');
						std::remove(line.begin(), line.end(), '/');
						std::remove(line.begin(), line.end(), '\\');
						std::remove(line.begin(), line.end(), '\r');
						std::remove(line.begin(), line.end(), '\n');

						// Now, create the new container and switch to it
						configsegment *child = new configsegment(line, cseg);
						cseg = child;
					}
				} else {
					// A key-value combination is being entered
					// First part is the key! The rest is the value
					// Key is in key
					string value;
					size_t vstart = line.find(key) + key.size();
					size_t vend = line.find_last_not_of(' ');
					// Trim line to get value
					value = line.substr(vstart, vend - vstart);

					// Check for special keywords, like Include!
					if (key == "Include")
					{
						loadFile(value.c_str(), cseg); // Load a file
					} else {
						// Set the key-val combination
						cseg->setVal(key,value);
					}
				}

			}
			return root;
		}

	}; // end config
};// end rtmath


