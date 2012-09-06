#include "../rtmath/Stdafx.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <memory>
#include <string>
#include <vector>
#include <map>
#include <set>
#include <boost/filesystem.hpp>
#include <complex>
#include <boost/filesystem.hpp>
#include "../rtmath/matrixop.h"
#include "../rtmath/error/error.h"
#include "../rtmath/ddscat/ddOutputSingle.h"
#include "../rtmath/ddscat/ddScattMatrix.h"
#include "../rtmath/units.h"
#include "../rtmath/quadrature.h"

namespace rtmath {
	namespace ddscat {

		ddOutputSingleObj::ddOutputSingleObj()
		{
		}

		ddOutputSingleObj::~ddOutputSingleObj()
		{
		}

		void ddOutputSingle::writeFile(const std::string &filename) const
		{
			// Look at extension of file
			std::ofstream out(filename.c_str());
			boost::filesystem::path p(filename);
			boost::filesystem::path pext = p.extension();
			if (pext.string() == ".sca")
			{
				writeSCA(out);
			} else if (pext.string() == ".fml")
			{
				writeFML(out);
			} else if (pext.string() == ".avg")
			{
				writeAVG(out);
			} else {
				throw rtmath::debug::xBadInput(filename.c_str());
			}
		}

		void ddOutputSingle::writeStatTable(std::ostream &out) const
		{
			using namespace std;
			out << "          Qext       Qabs       Qsca      g(1)=<cos>  <cos^2>     Qbk       Qpha" << endl;
			out << " JO=1: ";
			out.width(11);
			for (size_t i=0; i< (size_t) QEXT2; i++)
				out << _statTable[i];
			out << endl;
			out.width(0);
			out << " JO=2: ";
			out.width(11);
			for (size_t i=(size_t) QEXT2; i< (size_t) QEXTM; i++)
				out << _statTable[i];
			out << endl;
			out.width(0);
			out << " mean: ";
			out.width(11);
			for (size_t i=(size_t) QEXTM; i< (size_t) QPOL; i++)
				out << _statTable[i];
			out << endl;
			out.width(0);
			out << " Qpol= " << _statTable[(size_t) QPOL] << 
				"                                                  " << 
				"dQpha=";
			out.width(11);
			out << _statTable[(size_t) DQPHA] << endl;

			out << "         Qsca*g(1)   Qsca*g(2)   Qsca*g(3)   iter  mxiter  Nsca";
			out << " JO=1: ";
			out.width(11);
			for (size_t i=(size_t) QSCAG11; i< (size_t) QSCAG12; i++)
				out << _statTable[i];
			out << endl;
			out.width(0);
			out << " JO=2: ";
			out.width(11);
			for (size_t i=(size_t) QSCAG12; i< (size_t) QSCAG1M; i++)
				out << _statTable[i];
			out << endl;
			out.width(0);
			out << " mean: ";
			for (size_t i=(size_t) QSCAG1M; i< (size_t) NUM_STAT_ENTRIES; i++)
				out << _statTable[i];
			out << endl;
			out.width(0);
		}

		void ddOutputSingle::writeAVG(std::ostream &out) const
		{
			using namespace std;
			// Make sure that all the necessary variables are known
			// TODO

			// Write the file in the appropriate order
			_objMap.at("version")->write(out);
			_objMap.at("target")->write(out);
			_objMap.at("solnmeth")->write(out);
			_objMap.at("polarizability")->write(out);
			_objMap.at("shape")->write(out);
			_objMap.at("numdipoles")->write(out);

			_objMap.at("d/aeff")->write(out);
			_objMap.at("d")->write(out);

			_objMap.at("aeff")->write(out);
			_objMap.at("wave")->write(out);
			_objMap.at("k.aeff")->write(out);
			if (_version >= 72)
				_objMap.at("nambient")->write(out);
			_objMap.at("neps")->write(out);
			_objMap.at("tol")->write(out);
			
			_objMap.at("a1tgt")->write(out);
			_objMap.at("a2tgt")->write(out);
			_objMap.at("navg")->write(out);

			_objMap.at("kveclf")->write(out);
			_objMap.at("incpol1lf")->write(out);
			_objMap.at("incpol2lf")->write(out);

			_objMap.at("betarange")->write(out);
			_objMap.at("thetarange")->write(out);
			_objMap.at("phirange")->write(out);

			out << endl;

			_objMap.at("etasca")->write(out);
			
			_objMap.at("avgnumori")->write(out);
			_objMap.at("avgnumpol")->write(out);

			// Write the odd table of Qsca and the others
			writeStatTable(out);

			// Write the P matrix
			out << "            Mueller matrix elements for selected scattering directions in Lab Frame" << endl;
			out << " theta    phi    Pol.    S_11        S_12        S_21       S_22       S_31       S_41";
			for (auto it = _scattMatricesRaw.begin(); it != _scattMatricesRaw.end(); ++it)
			{
				boost::shared_ptr<const ddscat::ddScattMatrix> sf(it->second);
				out << endl;
				out.width(6);
				out << it->first.get(0);
				out << it->first.get(1);
				out.width(9);
				out << sf->pol();
				out.width(12);
				matrixop p = sf->mueller();
				out << p.get(2,0,0);
				out << p.get(2,0,1);
				out << p.get(2,1,0);
				out << p.get(2,1,1);
				out << p.get(2,2,0);
				out << p.get(2,3,0);
			}
		}

		void ddOutputSingle::writeSCA(std::ostream &out) const
		{
			using namespace std;
			// Make sure that all the necessary variables are known
			// TODO

			// Write the file in the appropriate order
			_objMap.at("version")->write(out);
			_objMap.at("target")->write(out);
			_objMap.at("solnmeth")->write(out);
			_objMap.at("polarizability")->write(out);
			_objMap.at("shape")->write(out);
			_objMap.at("numdipoles")->write(out);

			_objMap.at("d/aeff")->write(out);
			_objMap.at("d")->write(out);
			out << "----- physical extent of target volume in Target Frame ------" << endl;
			_objMap.at("xtf")->write(out);
			_objMap.at("ytf")->write(out);
			_objMap.at("ztf")->write(out);

			_objMap.at("aeff")->write(out);
			_objMap.at("wave")->write(out);
			_objMap.at("k.aeff")->write(out);
			if (_version >= 72)
				_objMap.at("nambient")->write(out);
			_objMap.at("neps")->write(out);
			_objMap.at("tol")->write(out);
			
			_objMap.at("a1tgt")->write(out);
			_objMap.at("a2tgt")->write(out);
			_objMap.at("navg")->write(out);

			_objMap.at("kvectf")->write(out);
			_objMap.at("incpol1tf")->write(out);
			_objMap.at("incpol2tf")->write(out);
			_objMap.at("kveclf")->write(out);
			_objMap.at("incpol1lf")->write(out);
			_objMap.at("incpol2lf")->write(out);

			_objMap.at("beta")->write(out);
			_objMap.at("theta")->write(out);
			_objMap.at("phi")->write(out);

			_objMap.at("etasca")->write(out);

			// Write the odd table of Qsca and the others
			writeStatTable(out);

			// Write the P matrix
			out << "            Mueller matrix elements for selected scattering directions in Lab Frame" << endl;
			out << " theta    phi    Pol.    S_11        S_12        S_21       S_22       S_31       S_41";
			for (auto it = _scattMatricesRaw.begin(); it != _scattMatricesRaw.end(); ++it)
			{
				boost::shared_ptr<const ddscat::ddScattMatrix> sf(it->second);
				out << endl;
				out.width(6);
				out << it->first.get(0);
				out << it->first.get(1);
				out.width(9);
				out << sf->pol();
				out.width(12);
				matrixop p = sf->mueller();
				out << p.get(2,0,0);
				out << p.get(2,0,1);
				out << p.get(2,1,0);
				out << p.get(2,1,1);
				out << p.get(2,2,0);
				out << p.get(2,3,0);
			}
		}

		void ddOutputSingle::writeFML(std::ostream &out) const
		{
			using namespace std;
			// Make sure that all the necessary variables are known
			// TODO

			// Write the file in the appropriate order
			_objMap.at("version")->write(out);
			_objMap.at("target")->write(out);
			_objMap.at("solnmeth")->write(out);
			_objMap.at("polarizability")->write(out);
			_objMap.at("shape")->write(out);
			_objMap.at("numdipoles")->write(out);

			_objMap.at("aeff")->write(out);
			_objMap.at("wave")->write(out);
			_objMap.at("k.aeff")->write(out);
			if (_version >= 72)
				_objMap.at("nambient")->write(out);
			_objMap.at("neps")->write(out);
			_objMap.at("tol")->write(out);
			_objMap.at("navg")->write(out);
			_objMap.at("a1tgt")->write(out);
			_objMap.at("a2tgt")->write(out);
			_objMap.at("kvectf")->write(out);
			_objMap.at("incpol1tf")->write(out);
			_objMap.at("incpol2tf")->write(out);
			_objMap.at("kveclf")->write(out);
			_objMap.at("incpol1lf")->write(out);
			_objMap.at("incpol2lf")->write(out);
			_objMap.at("beta")->write(out);
			_objMap.at("theta")->write(out);
			_objMap.at("phi")->write(out);

			out << "     Finite target:" << endl;
			out << "     e_m dot E(r) = i*exp(ikr)*f_ml*E_inc(0)/(kr)" << endl;
			out << "     m=1 in scatt. plane, m=2 perp to scatt. plane" << endl;
			out << endl;

			// Write the f matrix
			out << " theta   phi  Re(f_11)   Im(f_11)   Re(f_21)   Im(f_21)   Re(f_12)   Im(f_12)   Re(f_22)   Im(f_22)";
			for (auto it = _scattMatricesRaw.begin(); it != _scattMatricesRaw.end(); ++it)
			{
				if (it->second->id() != F) continue;
				boost::shared_ptr<const ddscat::ddScattMatrixF> sf(
					boost::shared_dynamic_cast<const ddscat::ddScattMatrixF>(it->second));
				out << endl;
				out.width(6);
				out << it->first.get(0);
				out << it->first.get(1);
				out.width(11);
				matrixop f = sf->getF();
				out << f.get(2,0,0);
				out << f.get(2,0,1);
				out << f.get(2,1,0);
				out << f.get(2,1,1);
				out << f.get(2,0,2);
				out << f.get(2,0,3);
				out << f.get(2,1,2);
				out << f.get(2,1,3);
			}
		}

		ddOutputSingle::ddOutputSingle()
		{
			_init();
		}

		void ddOutputSingle::_init()
		{
			_version = 72;
			_constructGraph();
		}
		
		ddOutputSingle::~ddOutputSingle()
		{
		}

		void ddOutputSingle::_constructGraph()
		{
			/* Here, a dependency graph is constructed indicating the 
			 * relationships between various quantities. This allows for 
			 * file format interconversions and detecting whether 
			 * such a change is even possible
			 */
			using namespace std;
			using namespace rtmath::graphs;
			_vertices.clear();
			_vertexMap.clear();
			_depsAVG.clear();
			_depsFML.clear();
			_depsSCA.clear();

			const size_t varnames_size = 0; // TODO: get number
			const string rawvarnames[] = {	// TODO: finish
				"version",
				"target_line",
				"soln_meth",
				"polarizability",
				"shape",
				"numdipoles",
				"d/aeff",
				"d",
				"aeff",
				"wavelength",
				"k.aeff",
				"Nambient",
				"neps",
				"tol"
			};

			std::set<std::string> varnames( rawvarnames, rawvarnames + varnames_size );

			// Create vertices for end variables
			for (auto it = varnames.begin(); it != varnames.end(); ++it)
				_createVertex(*it, true);

			// Create vertices representing function relationships
			// This is a table that lists the name of the new node, the variable being calculated, 
			// and the necessary dependencies. This is much cleaner than repeated copying / pasting, 
			// and is much easier to read.
			const size_t varmapnames_size = 54;
			const std::string rawvarmapnames[varmapnames_size] = {
				// name,					target,				dependencies
				"DAEFF",					"d/aeff",			"aeff,d",
				"T_DENS",					"density",			"temp",
				"AEFF_V",					"volume",			"aeff",
				"V_AEFF",					"aeff",				"volume",
				"MASS_V__DENS",				"density",			"mass,volume",
				"MASS_DENS__V",				"volume",			"mass,density",
				"DENS_V__MASS",				"mass",				"density,volume",
				"FREQ_TEMP__IREFR_R",		"irefr_r",			"freq,temp",
				"FREQ_TEMP__IREFR_I",		"irefr_i",			"freq,temp"
			};

			for (size_t i=0; i< varmapnames_size; i = i + 3)
			{
				const string &name = rawvarmapnames[i];
				const string &starget = rawvarmapnames[i+1];
				const string &deps = rawvarmapnames[i+2];

				_createVertex(name, starget, deps);
			}
			

			// Create the graph from the vertices
			_graph = boost::shared_ptr<graph>(new graph(_vertices));
		}

		/*
		bool ddOutputSingle::operator< (const ddOutputSingle &rhs) const
		{
			if (_freq != rhs._freq) return _freq < rhs._freq;
			if (_reff != rhs._reff) return _reff < rhs._reff;
			if (_beta != rhs._beta) return _beta < rhs._beta;
			if (_theta != rhs._theta) return _theta < rhs._theta;
			if (_phi != rhs._phi) return _phi < rhs._phi;
			return false;
		}
		*/
		/*
		void ddOutputSingle::writeEvans(std::ostream &out, double freq) const
		{
			using namespace std;
			// Takes the scattering data, generates the appropriate
			// phase, extinction matrices and emission vectors for a given
			// frequency, and writes the necessary file.

			// Generate quadrature points
			set<double> qangles;
			// Let's get the quadrature angles from gaussian quadrature
			// Other quadrature methods can be coded in as well
			const size_t deg = 7;
			quadrature::getQuadPtsLeg(deg,qangles);
			// The quadrature points are on interval (-1,1)
			// Need to get mapping between angles in degrees and these points
			// can handle this by mapping mu = cos(theta).

			// For phase functions, need to look at both incoming and outgoing angle
			// in cos(theta)
			// ddscat output always has incoming angle at zero degrees, with a varying output angle.
			// However, the targets may be rotated to simulate the angle change.
			// In this case, however, we likely just have a single ensemble pf from the data
			// TODO!!!!!
			std::map<coords::cyclic<double>, std::shared_ptr<const ddscat::ddScattMatrix> >
				interped;
			// Need to interpolate phase matrices to the correct quadrature points
			for (auto it = qangles.begin(); it != qangles.end(); it++)
			{
				// TODO: convert angle into cyclic coords
				throw rtmath::debug::xUnimplementedFunction();
				coords::cyclic<double> crd;
				std::shared_ptr<const ddscat::ddScattMatrix> interres;
				interpolate(crd, interres);
				interped[crd] = interres;
			}

			// First, write commented header information
			// This includes where the scattering information is from, and a
			// discription of each of these files
			out << "C  ddscat rtmath output for " << endl;
			out << "C  theta " << _theta << " phi " << _phi << " beta " << _beta << endl;
			out << "C  at f = " << _freq << " GHz" << endl;

			// Next is the degree and type of quadrature
			cout << "   8    0   'GAUSSIAN         '" << endl;

			// Output each scattering matrix at the designated quadrature incoming
			// and outgoing angles
			out << "C   SCATTERING MATRIX" << endl;
			for (auto it = interped.begin(); it != interped.end(); ++it)
			{
				// Write incoming angle, outcoming angle, 0
			}

			// Write the extinction matrix at each quadrature angle
			out << "C   EXTINCTION MATRIX" << endl;
			for (auto it = interped.begin(); it != interped.end(); ++it)
			{
				// Write incoming angle
			}

			// Output the emission vectors
			out << "C   EMISSION VECTOR" << endl;
			for (auto it = interped.begin(); it != interped.end(); ++it)
			{
				// Write incoming angle and the four stokes parameters
			}
			// Evans fortran files lack a newline at EOF.
		}
		*/

		/*
		void ddOutputSingle::loadFile(const std::string &filename)
		{
			using namespace std;
			// File loading routine is important!
			// Load a standard .fml file. Parse each line for certain key words.
			clear();
			bool dataseg = false;
			this->_filename = filename;

			// If a shape is not loaded, then try to load the corresponding shapefile
			using namespace boost::filesystem;
			path pshapepath, p, pfile(filename);
			p = pfile.parent_path();
			string shapepath;
			{
				// Figure out where the shape file is located.
				path ptarget = p / "target.out";
				path pshapedat = p / "shape.dat";
				if (exists(ptarget))
				{ pshapepath = ptarget;
				} else if (exists(pshapedat))
				{ pshapepath = pshapedat;
				} else {
					throw rtmath::debug::xMissingFile("shape.dat or target.out");
				}
				shapepath = pshapepath.string();
				if (exists(pshapepath) && !_shape)
					_shape = boost::shared_ptr<shapefile>(new shapefile(shapepath));
			}

			ifstream in(filename.c_str(), std::ifstream::in);
			while (in.good())
			{
				// Read a line
				string lin;
				std::getline(in,lin);
				istringstream lss(lin);
				// Expand line using convenient expansion function
				//vector<string> seg;
				//splitString(lin, ' ', seg);

				// Are we in the data segment?
				if (dataseg)
				{
					// In data segment
					// Entry not modified after this, because it is const
					std::shared_ptr<const ddScattMatrix> nscat (new ddScattMatrix(_freq,lss));
					// Save to the maps and sets
					// Totally assuming that no duplicate entry exists
					_insert(nscat);
				} else {
					// Still in header segment
					string junk;
					// Search for key strings in file

					// BETA
					if (lin.find("BETA") != string::npos)
					{
						lss >> junk; // get rid of first word
						lss >> junk;
						lss >> _beta;
					}
					// THETA
					if (lin.find("THETA") != string::npos)
					{
						lss >> junk; // get rid of first word
						// Theta is unlike Beta and Phi, as there is
						// no space between THETA and =
						lss >> _theta;
					}
					// PHI
					if (lin.find("PHI") != string::npos)
					{
						lss >> junk; // get rid of first word
						lss >> junk;
						lss >> _phi;
					}
					// NAT0
					if (lin.find("NAT0") != string::npos)
					{
						lss >> _numDipoles;
					}
					// AEFF
					if (lin.find("AEFF") != string::npos)
					{
						lss >> junk; // get rid of first word
						lss >> _reff;
					}
					// WAVE
					if (lin.find("WAVE") != string::npos)
					{
						// BAD --- WAVE runs against size...
						//lss >> junk; // get rid of first word
						//lss >> _wavelength;
						// Instead, read wave from column 7 (starting at 0) to 17
						_wavelength = atof( lin.substr( 7, 10 ).c_str() );
						// Also do a conversion from wavelength to frequency,
						// for easier comparisons later
						units::conv_spec wvtof("um","GHz");
						_freq = wvtof.convert(_wavelength);
					}
					// theta --- indicates last line of header
					if (lin.find("Re(f_11)") != string::npos)
					{
						dataseg = true;
					}
				}
			}
		}
		*/

	} // end ddscat
} // end rtmath

