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
			throw debug::xUnimplementedFunction();
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
						if (*it == '\'') { skip = true; break; }
						// If we make it here, the line is valid for parsing
						skip = false;
						break;
					}
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
					pString(it->second, sout);
					_valsStr["CMTORQ"] = sout;
				}
				else if (it->first.find("CMDSOL") != string::npos)
				{
					pString(it->second, sout);
					_valsStr["CMDSOL"] = sout;
				}
				else if (it->first.find("CMDFFT") != string::npos)
				{
					pString(it->second, sout);
					_valsStr["CMDFFT"] = sout;
				}
				else if (it->first.find("CALPHA") != string::npos)
				{
					pString(it->second, sout);
					_valsStr["CALPHA"] = sout;
				}
				else if (it->first.find("CBINFLAG") != string::npos)
				{
					pString(it->second, sout);
					_valsStr["CBINFLAG"] = sout;
				}
				else if (it->first.find("dimension") != string::npos)
				{
					pNumeric<size_t>(it->second, vS);
				}
				else if (it->first.find("CSHAPE") != string::npos)
				{
					pString(it->second, sout);
					_valsStr["CSHAPE"] = sout;
				}
				else if (it->first.find("shape parameters") != string::npos)
				{
					pNumeric<double>(it->second, vD);
				}
				else if (it->first.find("NCOMP") != string::npos)
				{
					pNumeric<size_t>(it->second, st);
				}
				else if (it->first.find("refractive index") != string::npos)
				{
					pString(it->second, sout);
					_valsStr["DIELTAB"] = sout;
				}
				else if (it->first.find("TOL") != string::npos)
				{
					pNumeric<double>(it->second, d);
				}
				else if (it->first.find("GAMMA") != string::npos)
				{
					pNumeric<double>(it->second, d);
				}
				else if (it->first.find("ETASCA") != string::npos)
				{
					pNumeric<double>(it->second, d);
				}
				else if (it->first.find("wavelengths") != string::npos)
				{
					vector<double> wvlens;
					vector<string> vspacing;
					string spacing;
					pMixed<double,string>(it->second,3,wvlens,vspacing);
					pString(vspacing[0],spacing);
				}
				else if (it->first.find("aeff") != string::npos)
				{
					vector<double> as;
					vector<string> vspacing;
					string spacing;
					pMixed<double,string>(it->second,3,as,vspacing);
					pString(vspacing[0],spacing);
				}
				else if (it->first.find("Polarization state") != string::npos)
				{
					vector<vector<double> > vvPols;
					pTuples<double>(it->second,"( \t)",2,vvPols);
				}
				else if (it->first.find("IORTH") != string::npos)
				{
					pNumeric<size_t>(it->second, st);
				}
				else if (it->first.find("IWRKSC") != string::npos)
				{
					pNumeric<size_t>(it->second, st);
				}
				else if (it->first.find("IWRPOL") != string::npos)
				{
					_version = 70;
					pNumeric<size_t>(it->second, st);
				}
				else if (it->first.find("NBETA") != string::npos)
				{
					vector<double> betas;
					vector<size_t> vspacing;
					size_t spacing;
					pMixed<double,size_t>(it->second,3,betas,vspacing);
					spacing = vspacing[0];
				}
				else if (it->first.find("NTHETA") != string::npos)
				{
					vector<double> thetas;
					vector<size_t> vspacing;
					size_t spacing;
					pMixed<double,size_t>(it->second,3,thetas,vspacing);
					spacing = vspacing[0];
				}
				else if (it->first.find("NPHI") != string::npos)
				{
					vector<double> phis;
					vector<size_t> vspacing;
					size_t spacing;
					pMixed<double,size_t>(it->second,3,phis,vspacing);
					spacing = vspacing[0];
				}
				else if (it->first.find("IWAV") != string::npos)
				{
					pNumeric<size_t>(it->second, vS);
				}
				else if (it->first.find("NSMELTS") != string::npos)
				{
					pNumeric<size_t>(it->second, st);
				}
				else if (it->first.find("indices ij of") != string::npos)
				{
					pNumeric<size_t>(it->second, vS);
				}
				else if (it->first.find("CMDFRM") != string::npos)
				{
					pString(it->second, sout);
					_valsStr["CMDFRM"] = sout;
				}
				else if (it->first.find("NPLANES") != string::npos)
				{
					pNumeric<size_t>(it->second, st);
				}
				else if (it->first.find("for plane 1") != string::npos)
				{
					vector<double> vals;
					vector<size_t> vspacing;
					size_t spacing;
					pMixed<double,size_t>(it->second,3,vals,vspacing);
					spacing = vspacing[0];
				}
				else if (it->first.find("for plane 2") != string::npos)
				{
					vector<double> vals;
					vector<size_t> vspacing;
					size_t spacing;
					pMixed<double,size_t>(it->second,3,vals,vspacing);
					spacing = vspacing[0];
				}
				else if (it->first.find("NRFLD") != string::npos)
				{
					_version = 72;
					pNumeric<size_t>(it->second, st);
				}
				else if (it->first.find("fract. extens.") != string::npos)
				{
					_version = 72;
					pNumeric<double>(it->second, vD);
				}
				else if (it->first.find("MXITER") != string::npos)
				{
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
			void pString(const std::string &in, std::string &out)
			{
				out.erase();
				// Remove all single quotes from the string
				for (auto it = in.begin(); it != in.end(); ++it)
					if (*it != '\'') out += *it;
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