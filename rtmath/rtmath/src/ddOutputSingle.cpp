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
#include "../rtmath/matrixop.h"
#include "../rtmath/error/error.h"
#include "../rtmath/ddscat/ddOutputSingle.h"
#include "../rtmath/ddscat/ddScattMatrix.h"
#include "../rtmath/units.h"
#include "../rtmath/quadrature.h"

namespace rtmath {
	namespace ddscat {
		ddOutputSingle::ddOutputSingle()
		{
			_init();
		}

		void ddOutputSingle::_init()
		{
			_beta = 0;
			_theta = 0;
			_phi = 0;
			_freq = 0;
			_wavelength = 0;
			_reff = 0;
			_numDipoles = 0;
			_version = 0;
			_filename = "";
		}

		ddOutputSingle::~ddOutputSingle()
		{
		}

		ddOutputSingle::ddOutputSingle()
		{
			_init();
		}

		bool ddOutputSingle::operator< (const ddOutputSingle &rhs) const
		{
			if (_freq != rhs._freq) return _freq < rhs._freq;
			if (_reff != rhs._reff) return _reff < rhs._reff;
			if (_beta != rhs._beta) return _beta < rhs._beta;
			if (_theta != rhs._theta) return _theta < rhs._theta;
			if (_phi != rhs._phi) return _phi < rhs._phi;
			return false;
		}

		/*
		void ddOutputSingle::print(std::ostream &out) const
		{
			using namespace std;
			out << "ddOutputSingle for beta " << _beta << 
				" theta " << _theta << " phi " << _phi
				<< " frequency " << _freq << endl;
		}

		void ddOutputSingle::writeCSV(const std::string &filename) const
		{
			std::ofstream out(filename.c_str());
			//out << "CSV output for (beta,theta,phi,wavelength)=";
			out << "beta, theta, phi, ";
			out << "P11, P12, P13, P14, P21, P22, P23, P24, P31, P32, P33, P34, P41, P42, P43, P44, ";
			out << "K11, K12, K13, K14, K21, K22, K23, K24, K31, K32, K33, K34, K41, K42, K43, K44";
			out << std::endl;
			writeCSV(out);
		}
		
		void ddOutputSingle::write(const std::string &filename) const
		{
			using namespace std;
			using namespace boost::filesystem;
			// Do extension matching
			path fp(filename);
			path ext = fp.extension();

			// TODO: add evans series (for this, strip .evans, expand filename into
			// per-freq set, and write out each one).
			if (ext.string() == ".csv")
				writeCSV(filename);
			else // Write csv output by default. Others, like nc and tsv, will be added later.
				writeCSV(filename);
		}

		void ddOutputSingle::writeCSV(std::ostream &out) const
		{
			using namespace std;
			//out << _beta << ", " << _theta << ", " << _phi << ", " << _freq << endl;

			// Need to collect the objects into a map for sorting. Otherwise, csv writes 
			// irregularly.

			std::map<rtmath::coords::cyclic<double>, std::shared_ptr<const ddscat::ddScattMatrix> > mmap;
			for (auto it = _scattMatricesRaw.begin(); it != _scattMatricesRaw.end(); ++it)
			{
				rtmath::coords::cyclic<double> c = (*it)->genCoords();
				mmap[c] = *it;
			}

			for (auto it = mmap.begin(); it != mmap.end(); ++it)
			{
				it->second->writeCSV(out);
				//(*it)->writeCSV(out);
			}
		}

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
		void ddOutputSingle::_insert(boost::shared_ptr<const ddScattMatrix> &obj)
		{
			// Helps loadFile and collects all insertions in a convenient place.
			//_scattMatricesRaw.insert(obj);
			// Insert into interpolation handler
			ddOutputSingleInterp::_insert(obj);
		}

		void ddOutputSingle::clear()
		{
			ddOutputSingleInterp::_clear();
			_init();
		}

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

	} // end ddscat
} // end rtmath

