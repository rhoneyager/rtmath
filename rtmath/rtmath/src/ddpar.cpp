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
#include "../rtmath/config.h"

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

		void ddPar::loadFile(const std::string &filename, bool overlay)
		{
			// Check file existence
			using namespace std;
			using namespace boost::filesystem;
			path p(filename);
			if (!exists(p)) throw debug::xMissingFile(filename.c_str());
			ifstream in(filename.c_str());
			load(in, overlay);
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

		std::shared_ptr<ddParParsers::ddParLine> ddPar::getKey(ddParParsers::ParId key)
		{
			std::shared_ptr<ddParParsers::ddParLine> res = nullptr;
			if (_parsedData.count(key))
				res = _parsedData[key];
			return res;
		}

		void ddPar::delKey(ddParParsers::ParId key)
		{
			if (_parsedData.count(key))
				_parsedData.erase(key);
		}

		void ddPar::insertKey(ddParParsers::ParId key, std::shared_ptr<ddParParsers::ddParLine> &ptr)
		{
			if (_parsedData.count(key))
				_parsedData.erase(key);
			_parsedData[key] = ptr;
		}

		void ddPar::_populateDefaults(bool overwrite) const
		{
			// Populates missing items for this version with default
			// entries. Used when converting between ddscat file versions.
			// Also used when file information is incomplete.
			using namespace rtmath::ddscat::ddParParsers;
			using namespace boost::filesystem;
			using namespace std;
			// Let's take the default entries from a default scattering file.
			// I don't want to hardcode these values, plus, by varying the path, 
			// this improves scriptability...

			shared_ptr<rtmath::config::configsegment> cRoot = config::loadRtconfRoot();
			string sBasePar;
			cRoot->getVal("ddscat/DefaultFile", sBasePar);
			if (sBasePar.size())
			{
				if (exists(path(sBasePar)))
				{
					ddPar base(sBasePar);
					for (auto it = base._parsedData.begin(); it != base._parsedData.end(); it++)
					{
						// If overwrite, then overwrite any existing key
						// If not, and key exists, skip to next one
						// If key does not exist, add it
						if (this->_parsedData.count(it->first))
						{
							if (overwrite)
							{
								this->_parsedData.erase(it->first);
							} else {
								continue;
							}
						}
						this->_parsedData[it->first] = it->second;
					}
				} else {
					// Default file not found
					// TODO!
					GETOBJKEY();
				}
			} else {
				// Default file not listed
				// TODO!
				GETOBJKEY();
			}
		}

		void ddPar::_init()
		{
			_version = 72;
		}

		void ddPar::load(std::istream &stream, bool overlay)
		{
			// Parse until end of stream, line-by-line
			// Split string based on equal sign, and do parsing
			// based on keys
			using namespace std;
			using namespace rtmath::config;
			// _keys are mostly useless. Just used for loading.
			std::map<std::string, std::string> _keys;
			if (!overlay)
			{
				_parsedData.clear();
			}

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

			// Map the keys
			using namespace rtmath::ddscat::ddParParsers;
			for (auto it = _keys.begin(); it != _keys.end(); ++it)
			{
				std::shared_ptr<ddParLine> ptr = mapKeys(it->first);
				ptr->read(it->second);
				if (_parsedData.count(ptr->id()))
				{
					if (overlay)
					{
						_parsedData.erase(ptr->id());
					} else {
						ostringstream ostr;
						ostr << "Duplicate ddscat.par key: ";
						ostr << it->first;
						throw rtmath::debug::xBadInput(ostr.str().c_str());
					}
				}
				_parsedData[ptr->id()] = ptr;
			}
		}

	

		namespace ddParParsers
		{
			std::shared_ptr<ddParLine> ddParParsers::mapKeys(const std::string &key)
			{
				using namespace std;
				using namespace rtmath::ddscat::ddParParsers;
				std::shared_ptr<ddParLine> ptr;

				if (key.find("CMTORQ") != string::npos)
					ptr = std::shared_ptr<ddParLineSimple<std::string> > 
					( new ddParLineSimple<std::string>(CMTORQ) );
				else if (key.find("CMDSOL") != string::npos)
					ptr = std::shared_ptr<ddParLineSimple<std::string> >
					( new ddParLineSimple<std::string>(CMDSOL) );
				else if (key.find("CMDFFT") != string::npos)
					ptr = std::shared_ptr<ddParLineSimple<std::string> >
					( new ddParLineSimple<std::string>(CMDFFT) );
				else if (key.find("CALPHA") != string::npos)
					ptr = std::shared_ptr<ddParLineSimple<std::string> >
					( new ddParLineSimple<std::string>(CALPHA) );
				else if (key.find("CBINFLAG") != string::npos)
					ptr = std::shared_ptr<ddParLineSimple<std::string> >
					( new ddParLineSimple<std::string>(CBINFLAG) );
				else if (key.find("dimension") != string::npos)
					ptr = std::shared_ptr<ddParLineSimplePlural<size_t> >
					( new ddParLineSimplePlural<size_t>(DIMENSION) );
				else if (key.find("CSHAPE") != string::npos)
					ptr = std::shared_ptr<ddParLineSimple<std::string> >
					( new ddParLineSimple<std::string>(CSHAPE) );
				else if (key.find("shape parameters") != string::npos)
					ptr = std::shared_ptr<ddParLineSimplePlural<double> > 
					( new ddParLineSimplePlural<double>(SHAPEPARAMS) );
				else if (key.find("NCOMP") != string::npos)
					ptr = std::shared_ptr<ddParLineSimple<std::size_t> > 
					( new ddParLineSimple<std::size_t>(NCOMP) );
				else if (key.find("refractive index") != string::npos)
					ptr = std::shared_ptr<ddParLineSimple<std::string> >
					( new ddParLineSimple<std::string>(IREFR) );
				else if (key.find("NRFLD") != string::npos)
					ptr = std::shared_ptr<ddParLineSimple<std::size_t> > 
					( new ddParLineSimple<std::size_t>(NRFLD) );
					// version 7.2 NRFLD
				else if (key.find("fract. extens.") != string::npos)
					ptr = std::shared_ptr<ddParLineSimplePlural<double> >
					( new ddParLineSimplePlural<double>(FRACT_EXTENS) );
					// version 7.2 FRACT_EXTENS
				else if (key.find("TOL") != string::npos)
					ptr = std::shared_ptr<ddParLineSimple<double> > 
					( new ddParLineSimple<double>(TOL) );
				else if (key.find("MXITER") != string::npos)
					ptr = std::shared_ptr<ddParLineSimple<std::size_t> > 
					( new ddParLineSimple<std::size_t>(MXITER) );
					// version 7.2 MXITER
				else if (key.find("GAMMA") != string::npos)
					ptr = std::shared_ptr<ddParLineSimple<double> > 
					( new ddParLineSimple<double>(GAMMA) );
				else if (key.find("ETASCA") != string::npos)
					ptr = std::shared_ptr<ddParLineSimple<double> > 
					( new ddParLineSimple<double>(ETASCA) );

				// TODO: fix wavelengths and aeff to read two doubles, a size_t and a string
				else if (key.find("wavelengths") != string::npos)
					ptr = std::shared_ptr<ddParLineMixed<double, std::string> >
					( new ddParLineMixed<double, std::string>(3, WAVELENGTHS));
				else if (key.find("NAMBIENT") != string::npos)
					ptr = std::shared_ptr<ddParLineSimple<double> > 
					( new ddParLineSimple<double>(NAMBIENT) );
				else if (key.find("aeff") != string::npos)
					ptr = std::shared_ptr<ddParLineMixed<double, std::string> >
					( new ddParLineMixed<double, std::string>(3, AEFF));

				else if (key.find("Polarization state") != string::npos)
					ptr = std::shared_ptr<ddParTuples<double> >
					( new ddParTuples<double>(2, POLSTATE));
				else if (key.find("IORTH") != string::npos)
					ptr = std::shared_ptr<ddParLineSimple<std::size_t> > 
					( new ddParLineSimple<std::size_t>(IORTH) );
				else if (key.find("IWRKSC") != string::npos)
					ptr = std::shared_ptr<ddParLineSimple<std::size_t> > 
					( new ddParLineSimple<std::size_t>(IWRKSC) );
				else if (key.find("IWRPOL") != string::npos)
					ptr = std::shared_ptr<ddParLineSimple<std::size_t> > 
					( new ddParLineSimple<std::size_t>(IWRPOL) );
					// IWRPOL is version 7.0

				else if (key.find("NBETA") != string::npos)
					ptr = std::shared_ptr<ddParLineMixed<double, size_t> >
					( new ddParLineMixed<double, size_t>(2, NBETA));
				else if (key.find("NTHETA") != string::npos)
					ptr = std::shared_ptr<ddParLineMixed<double, size_t> >
					( new ddParLineMixed<double, size_t>(2, NTHETA));
				else if (key.find("NPHI") != string::npos)
					ptr = std::shared_ptr<ddParLineMixed<double, size_t> >
					( new ddParLineMixed<double, size_t>(2, NPHI));
				else if (key.find("IWAV") != string::npos)
					ptr = std::shared_ptr<ddParLineSimplePlural<double> >
					( new ddParLineSimplePlural<double>(IWAV) );
				else if (key.find("NSMELTS") != string::npos)
					ptr = std::shared_ptr<ddParLineSimple<std::size_t> > 
					( new ddParLineSimple<std::size_t>(NSMELTS) );
				else if (key.find("indices ij of") != string::npos)
					ptr = std::shared_ptr<ddParLineSimplePlural<double> >
					( new ddParLineSimplePlural<double>(INDICESIJ) );
				else if (key.find("CMDFRM") != string::npos)
					ptr = std::shared_ptr<ddParLineSimple<std::string> >
					( new ddParLineSimple<std::string>(CMDFRM) );
				else if (key.find("NPLANES") != string::npos)
					ptr = std::shared_ptr<ddParLineSimple<std::size_t> > 
					( new ddParLineSimple<std::size_t>(NPLANES) );
				else if (key.find("for plane 1") != string::npos)
					ptr = std::shared_ptr<ddParLineMixed<double, size_t> >
					( new ddParLineMixed<double, size_t>(3, PLANE1));
				else if (key.find("for plane 2") != string::npos)
					ptr = std::shared_ptr<ddParLineMixed<double, size_t> >
					( new ddParLineMixed<double, size_t>(3, PLANE2));
				else
				{
					cerr << "Unmatched key: " << key << endl;
					throw rtmath::debug::xBadInput("ddscat.par");
				}

				return ptr;
			}

			bool ddParLine::versionValid(size_t ver) const
			{
				if (ver == 72)
				{
					switch (_id)
					{
					case IWRPOL:
						return false;
					default:
						return true;
					}
				}
				if (ver == 70)
				{
					switch (_id)
					{
					case NRFLD:
					case FRACT_EXTENS:
					case MXITER:
						return false;
					default:
						return true;
					}
				}
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
				switch (id)
				{
				case CMTORQ:
					key = "CMTORQ*6 (NOTORQ, DOTORQ) -- either do or skip torque calculations";
					break;
				case CMDSOL:
					key = "CMDSOL*6 (PBCGS2, PBCGST, PETRKP) -- select solution method";
					break;
				case CMDFFT:
					key = "CMDFFT*6 (GPFAFT, FFTMKL)";
					break;
				case CALPHA:
					key = "CALPHA*6 (GKDLDR, LATTDR)";
					break;
				case CBINFLAG:
					key = "CBINFLAG (NOTBIN, ORIBIN, ALLBIN)";
					break;
				case DIMENSION:
					key = "dimension";
					break;
				case CSHAPE:
					key = "CSHAPE*9 shape directive";
					break;
				case SHAPEPARAMS:
					key = "shape parameters 1-3";
					break;
				case NCOMP:
					key = "NCOMP = number of dielectric materials";
					break;
				case IREFR:
					key = "file with refractive index 1";
					break;
				case NRFLD:
					key = "NRFLD (=0 to skip nearfield calc., =1 to calculate nearfield E)";
					break;
				case FRACT_EXTENS:
					key = "(fract. extens. of calc. vol. in -x,+x,-y,+y,-z,+z)";
					break;
				case TOL:
					key = "TOL = MAX ALLOWED (NORM OF |G>=AC|E>-ACA|X>)/(NORM OF AC|E>)";
					break;
				case MXITER:
					key = "MXITER";
					break;
				case GAMMA:
					key = "GAMMA (1e-2 is normal, 3e-3 for greater accuracy)";
					break;
				case ETASCA:
					key = "ETASCA (number of angles is proportional to [(3+x)/ETASCA]^2 )";
					break;
				case WAVELENGTHS:
					key = "wavelengths";
					break;
				case NAMBIENT:
					key = "NAMBIENT";
					break;
				case AEFF:
					key = "aeff";
					break;
				case POLSTATE:
					key = "Polarization state e01 (k along x axis)";
					break;
				case IORTH:
					key = "IORTH  (=1 to do only pol. state e01; =2 to also do orth. pol. state)";
					break;
				case IWRKSC:
					key = "IWRKSC (=0 to suppress, =1 to write \".sca\" file for each target orient.";
					break;
				case IWRPOL:
					key = "IWRPOL (=0 to suppress, =1 to write \".pol\" file for each (BETA,THETA)";
					break;
				case NBETA:
					key = "BETAMI, BETAMX, NBETA  (beta=rotation around a1)";
					break;
				case NTHETA:
					key = "THETMI, THETMX, NTHETA (theta=angle between a1 and k)";
					break;
				case NPHI:
					key = "PHIMIN, PHIMAX, NPHI (phi=rotation angle of a1 around k)";
					break;
				case IWAV:
					key = "first IWAV, first IRAD, first IORI (0 0 0 to begin fresh)";
					break;
				case NSMELTS:
					key = "NSMELTS = number of elements of S_ij to print (not more than 9)";
					break;
				case INDICESIJ:
					key = "indices ij of elements to print";
					break;
				case CMDFRM:
					key = "CMDFRM (LFRAME, TFRAME for Lab Frame or Target Frame)";
					break;
				case NPLANES:
					key = "NPLANES = number of scattering planes";
					break;
				case PLANE1:
					key = "phi, thetan_min, thetan_max, dtheta (in deg) for plane 1";
					break;
				case PLANE2:
					key = "phi, thetan_min, thetan_max, dtheta (in deg) for plane 2";
					break;
				case UNKNOWN:
				default:
					throw rtmath::debug::xBadInput("Unknown parid");
				}
			}
		} // end ddparparsers
	} // end namespace ddscat
} // end rtmath


