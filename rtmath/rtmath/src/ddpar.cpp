#include "../rtmath/Stdafx.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <memory>
#include <string>
#include <vector>
#include <map>
#include <set>
#include <unordered_map>
#include <complex>
#include <boost/filesystem.hpp>
#include <boost/tokenizer.hpp>
#include <boost/lexical_cast.hpp>
#include <cmath>
#include "../rtmath/matrixop.h"
#include "../rtmath/error/error.h"
#include "../rtmath/ddscat/ddpar.h"
#include "../rtmath/command.h"

namespace rtmath {
	namespace ddscat {

		ddPar::ddPar()
		{
			_init();
		}

		ddPar::ddPar(const std::string &filename)
		{
			_init();
			loadFile(filename);
		}

		ddPar::~ddPar()
		{
		}

		void ddPar::loadFile(const std::string &filename)
		{
			// Check file existence
			using namespace std;
			using namespace boost::filesystem;
			path p(filename);
			if (!exists(p)) throw debug::xMissingFile(filename.c_str());
			ifstream in(filename.c_str());
			load(in);
		}

		void ddPar::saveFile(const std::string &filename) const
		{
			// Writing is much easier than reading!
			using namespace std;

			// Ensute that all necessary keys exist. If not, create them!!!
			_populateDefaults();

			// Open file for writing
			ofstream out(filename.c_str());
			// Loop through and write parameters and comments
			auto ct = _comments.begin();
			auto it = _parsedData.begin();
			size_t line = 1;
			while (it != _parsedData.end() && ct != _comments.end())
			{
				if (_comments.count(line))
				{
					out << _comments.at(line) << endl;
				} else {
					// If key is valid for this output version, write it
					if (it->second->versionValid(_version))
						it->second->write(out);
				}
				line++;
			}
		}

		void ddPar::_populateDefaults() const
		{
			// Populates missing items for this version with default
			// entries. Used when converting between ddscat file versions.
			// Also used when file information is incomplete.
			throw rtmath::debug::xUnimplementedFunction();
		}

		void ddPar::_init()
		{
			_version = 72;
		}

		void ddPar::load(std::istream &stream)
		{
			// Parse until end of stream, line-by-line
			// Split string based on equal sign, and do parsing
			// based on keys
			using namespace std;
			using namespace rtmath::config;
			_keys.clear();

			size_t line = 0;
			while (stream.good())
			{
				string lin;
				std::getline(stream,lin);
				line++;

				// Check if this is a comment line (ends with ')
				// Need extra logic if line ends in whitespace
				{
					bool skip = false;
					for (auto it = lin.rbegin(); it != lin.rend(); ++it)
					{
						if (*it == ' ') continue;
						if (*it == '\'') 
						{ 
							// Define a comment
							_comments[line] = lin;
							// End line parsing
							skip = true; 
							break; 
						}
						// If we make it here, the line is valid for 
						// key-value pair parsing
						skip = false;
						break;
					} // Awkward. TODO: redo.
					if (skip) continue;
				}

				// Split lin based on '='
				// Prepare tokenizer
				typedef boost::tokenizer<boost::char_separator<char> >
					tokenizer;
				boost::char_separator<char> sep("=");
				tokenizer tcom(lin,sep);
				vector<string> vals;
				for (auto it=tcom.begin(); it != tcom.end(); ++it)
					vals.push_back(*it);
				if (vals.size() < 2) 
				{
					ostringstream errmsg;
					errmsg << "This is not a valid ddscat.par file (error on file line " << line << ").";
					throw rtmath::debug::xUnknownFileFormat(errmsg.str().c_str());
				}

				// Populate map
				_keys[vals[1]] = vals[0];

			}

			// Parse map
			parseMap();

		}

		void ddPar::parseMap()
		{
			using namespace std;
			using namespace rtmath::ddscat::ddParParsers;
			for (auto it = _keys.begin(); it != _keys.end(); ++it)
			{
				string sout;
				vector<size_t> vS;
				vector<double> vD;
				double d;
				size_t st;
				if (it->first.find("CMTORQ") != string::npos)
				{
					std::shared_ptr<ddParLineSimple<std::string> > ptr
						( new ddParLineSimple<std::string>(CMTORQ) );
					ptr->read(it->second);
					_parsedData[CMTORQ] = ptr;
				}
				else if (it->first.find("CMDSOL") != string::npos)
				{
					std::shared_ptr<ddParLineSimple<std::string> > ptr
						( new ddParLineSimple<std::string>(CMDSOL) );
					ptr->read(it->second);
					_parsedData[CMDSOL] = ptr;
				}
				else if (it->first.find("CMDFFT") != string::npos)
				{
					std::shared_ptr<ddParLineSimple<std::string> > ptr
						( new ddParLineSimple<std::string>(CMDFFT) );
					ptr->read(it->second);
					_parsedData[CMDFFT] = ptr;
				}
				else if (it->first.find("CALPHA") != string::npos)
				{
					std::shared_ptr<ddParLineSimple<std::string> > ptr
						( new ddParLineSimple<std::string>(CALPHA) );
					ptr->read(it->second);
					_parsedData[CALPHA] = ptr;
				}
				else if (it->first.find("CBINFLAG") != string::npos)
				{
					std::shared_ptr<ddParLineSimple<std::string> > ptr
						( new ddParLineSimple<std::string>(CBINFLAG) );
					ptr->read(it->second);
					_parsedData[CBINFLAG] = ptr;
				}
				else if (it->first.find("dimension") != string::npos)
				{
					//TODO
					pNumeric<size_t>(it->second, vS);
				}
				else if (it->first.find("CSHAPE") != string::npos)
				{
					std::shared_ptr<ddParLineSimple<std::string> > ptr
						( new ddParLineSimple<std::string>(CSHAPE) );
					ptr->read(it->second);
					_parsedData[CSHAPE] = ptr;
				}
				else if (it->first.find("shape parameters") != string::npos)
				{
					//TODO
					pNumeric<double>(it->second, vD);
				}
				else if (it->first.find("NCOMP") != string::npos)
				{
					//TODO
					pNumeric<size_t>(it->second, st);
				}
				else if (it->first.find("refractive index") != string::npos)
				{
					std::shared_ptr<ddParLineSimple<std::string> > ptr
						( new ddParLineSimple<std::string>(IREFR) );
					ptr->read(it->second);
					_parsedData[IREFR] = ptr;
				}
				else if (it->first.find("TOL") != string::npos)
				{
					//TODO
					pNumeric<double>(it->second, d);
				}
				else if (it->first.find("GAMMA") != string::npos)
				{
					//TODO
					pNumeric<double>(it->second, d);
				}
				else if (it->first.find("ETASCA") != string::npos)
				{
					//TODO
					pNumeric<double>(it->second, d);
				}
				else if (it->first.find("wavelengths") != string::npos)
				{
					//TODO
					vector<double> wvlens;
					vector<string> vspacing;
					string spacing;
					pMixed<double,string>(it->second,3,wvlens,vspacing);
					pString(vspacing[0],spacing);
				}
				else if (it->first.find("aeff") != string::npos)
				{
					//TODO
					vector<double> as;
					vector<string> vspacing;
					string spacing;
					pMixed<double,string>(it->second,3,as,vspacing);
					pString(vspacing[0],spacing);
				}
				else if (it->first.find("Polarization state") != string::npos)
				{
					//TODO
					vector<vector<double> > vvPols;
					pTuples<double>(it->second,"( \t)",2,vvPols);
				}
				else if (it->first.find("IORTH") != string::npos)
				{
					//TODO
					pNumeric<size_t>(it->second, st);
				}
				else if (it->first.find("IWRKSC") != string::npos)
				{
					//TODO
					pNumeric<size_t>(it->second, st);
				}
				else if (it->first.find("IWRPOL") != string::npos)
				{
					//TODO
					_version = 70;
					pNumeric<size_t>(it->second, st);
				}
				else if (it->first.find("NBETA") != string::npos)
				{
					//TODO
					vector<double> betas;
					vector<size_t> vspacing;
					size_t spacing;
					pMixed<double,size_t>(it->second,3,betas,vspacing);
					spacing = vspacing[0];
				}
				else if (it->first.find("NTHETA") != string::npos)
				{
					//TODO
					vector<double> thetas;
					vector<size_t> vspacing;
					size_t spacing;
					pMixed<double,size_t>(it->second,3,thetas,vspacing);
					spacing = vspacing[0];
				}
				else if (it->first.find("NPHI") != string::npos)
				{
					//TODO
					vector<double> phis;
					vector<size_t> vspacing;
					size_t spacing;
					pMixed<double,size_t>(it->second,3,phis,vspacing);
					spacing = vspacing[0];
				}
				else if (it->first.find("IWAV") != string::npos)
				{
					//TODO
					pNumeric<size_t>(it->second, vS);
				}
				else if (it->first.find("NSMELTS") != string::npos)
				{
					//TODO
					pNumeric<size_t>(it->second, st);
				}
				else if (it->first.find("indices ij of") != string::npos)
				{
					//TODO
					pNumeric<size_t>(it->second, vS);
				}
				else if (it->first.find("CMDFRM") != string::npos)
				{
					std::shared_ptr<ddParLineSimple<std::string> > ptr
						( new ddParLineSimple<std::string>(CMDFRM) );
					ptr->read(it->second);
					_parsedData[CMDFRM] = ptr;
				}
				else if (it->first.find("NPLANES") != string::npos)
				{
					//TODO
					pNumeric<size_t>(it->second, st);
				}
				else if (it->first.find("for plane 1") != string::npos)
				{
					//TODO
					vector<double> vals;
					vector<size_t> vspacing;
					size_t spacing;
					pMixed<double,size_t>(it->second,3,vals,vspacing);
					spacing = vspacing[0];
				}
				else if (it->first.find("for plane 2") != string::npos)
				{
					//TODO
					vector<double> vals;
					vector<size_t> vspacing;
					size_t spacing;
					pMixed<double,size_t>(it->second,3,vals,vspacing);
					spacing = vspacing[0];
				}
				else if (it->first.find("NRFLD") != string::npos)
				{
					//TODO
					_version = 72;
					pNumeric<size_t>(it->second, st);
				}
				else if (it->first.find("fract. extens.") != string::npos)
				{
					//TODO
					_version = 72;
					pNumeric<double>(it->second, vD);
				}
				else if (it->first.find("MXITER") != string::npos)
				{
					//TODO
					_version = 72;
					pNumeric<size_t>(it->second, st);
				}
				/*else if (it->first.find("") != string::npos)
				{
				}*/
				else
				{
					cerr << "Unmatched key: " << it->first << endl;
					cerr << "\tWith value: " << it->second << endl;
				}
			}
		}

		namespace ddParParsers
		{
			bool ddParLine::versionValid(size_t ver) const
			{
				return true;
			}


			void pString(const std::string &in, std::string &out)
			{
				out.erase();
				// Remove all single quotes from the string
				for (auto it = in.begin(); it != in.end(); ++it)
					if (*it != '\'') out += *it;
			}

			void idString(ParId id, std::string &key)
			{
				throw rtmath::debug::xUnimplementedFunction();
			}
		}

	}
}

/*
				// Prepare tokenizer
				typedef boost::tokenizer<boost::char_separator<char> >
					tokenizer;
				boost::char_separator<char> sep(",");
				boost::char_separator<char> seprange(":-");
				{
					tokenizer tcom(instr,sep);
					for (auto ot = tcom.begin(); ot != tcom.end(); ot++)
					{
						// Separated based on commas. Expand for dashes and colons
						tokenizer trange(*ot,seprange);
						vector<size_t> range;
						for (auto rt = trange.begin(); rt != trange.end(); rt++)
						*/