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
#include "../rtmath/matrixop.h"
#include "../rtmath/error/error.h"
#include "../rtmath/ddscat/ddOutputSingle.h"
#include "../rtmath/ddscat/ddScattMatrix.h"
#include "../rtmath/units.h"

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
			_filename = "";
		}

		ddOutputSingle::~ddOutputSingle()
		{
		}

		ddOutputSingle::ddOutputSingle(const std::string &filename)
		{
			_init();
			loadFile(filename);
		}

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
			out << "CSV output for (beta,theta,phi,wavelength)=";
			writeCSV(out);
		}

		void ddOutputSingle::writeCSV(std::ostream &out) const
		{
			using namespace std;
			out << _beta << ", " << _theta << ", " << _phi << ", " << _freq << endl;
			for (auto it = _scattMatricesRaw.begin(); it != _scattMatricesRaw.end(); ++it)
			{
				(*it)->writeCSV(out);
			}
		}

		void ddOutputSingle::_insert(std::shared_ptr<const ddScattMatrix> &obj)
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

		void ddOutputSingle::genCoords(coords::cyclic<double> &res) const
		{
			coords::cyclic<double> ret(4,freq(),beta(),theta(),phi());
			res = ret;
		}

	} // end ddscat
} // end rtmath


std::ostream & operator<<(std::ostream &stream, const rtmath::ddscat::ddOutputSingle &ob)
{
	ob.writeCSV(stream);
	return stream;
}

/*
std::istream &operator>>(std::istream &stream, rtmath::ddscat::ddOutputSingle &ob)
{
	ob.setF(stream);
	return stream;
}
*/

