#include "ddscat2.h"
#include "splitstring.h"
#include <iostream>

namespace rtmath {

	namespace ddscat {

	scattMatrix::scattMatrix()
	{
		_theta = 0;
		_phi = 0;
		// vals should be auto-initializing
	}

	scattMatrix::scattMatrix(double theta, double phi)
	{
		_theta = theta;
		_phi = phi;
	}

	scattMatrix& scattMatrix::operator=(const scattMatrix &rhs)
	{
		// Check for pointer equality. If equal, return.
		if (this == &rhs) return *this;
		_theta = rhs._theta;
		_phi = rhs._phi;
		for (size_t i=0; i<2; i++)
			for (size_t j=0; j<2; j++)
				vals[i][j] = rhs.vals[i][j];
	}

	bool scattMatrix::operator==(const scattMatrix &rhs) const
	{
		if (this == &rhs) return true;
		if (_theta != rhs._theta) return false;
		if (_phi != rhs._phi) return false;
		for (size_t i=0; i<2; i++)
			for (size_t j=0; j<2; j++)
				if (vals[i][j] != rhs.vals[i][j]) return false;
		return true;
	}

	bool scattMatrix::operator!=(const scattMatrix &rhs) const
	{
		return !operator==(rhs);
	}

	void scattMatrix::print() const
	{
		using namespace std;
		cout << "Scattering matrix for theta " << _ theta << " phi "
				<< _phi << endl;
		for (size_t i=0; i<2; i++)
			for (size_t j=0; j<2; j++)
				cout << i << "," << j << "\t" << vals[i][j] << endl;
	}

	void ddOutputSingle::_init()
	{
		_Beta = 0;
		_Theta = 0;
		_Phi = 0;
		_wavelength = 0;
		//_sizep = 0;
		_numDipoles = 0;
		_reff = 0;
		for (int i=0; i<3; i++)
			_shape[i] = 0;
	}

	ddOutputSingle::ddOutputSingle(double beta, double theta, double phi)
	{
		_init();
		_Beta = beta;
		_Theta = theta;
		_Phi = phi;
	}

	void ddOutputSingle::getF(const ddCoords &coords, scattMatrix &f) const
	{
		if (_fs.count(coords)) f = _fs[coords];
		else f = NULL;
	}

	void ddOutputSingle::setF(const ddCoords &coords, const scattMatrix &f)
	{
		_fs[coords] = f;
	}

	void ddOutputSingle::loadFile(const std::string &filename)
	{
		using namespace std;
		// File loading routine is important!
		// Load a standard .fml file. Parse each line for certain key words.
		bool dataseg = false;

		ifstream in(filename.c_str());
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
				// theta is seg[0], phi is seg[1]
				double theta, phi, re, im;
				lss >> theta;
				lss >> phi;
				scattMatrix nscat(theta,phi);
				// For the next eight quantities, load the complex f
				for (size_t i=0; i<2; i++)
					for (size_t j=0; j<2; j++)
					{
						lss >> re;
						lss >> im;
						complex<double> nval(re,im);
						nscat.vals[i][j] = nval;
					}
				// Save to the map
				setF(ddCoords(theta,phi),nscat);
			} else {
				// Still in header segment
				string junk;
				// Search for key strings in file

				// BETA
				if (lin.find("BETA") != string::npos)
				{
					lss >> junk; // get rid of first word
					lss >> _Beta;
				}
				// THETA
				if (lin.find("THETA") != string::npos)
				{
					lss >> junk; // get rid of first word
					lss >> _Theta;
				}
				// PHI
				if (lin.find("PHI") != string::npos)
				{
					lss >> junk; // get rid of first word
					lss >> _Phi;
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
					lss >> junk; // get rid of first word
					lss >> _wavelength;
				}
				// theta --- indicates last line of header
				if (lin.find("theta") != string::npos)
				{
					dataseg = true;
				}
			}
		}
	}

	ddOutput::ddOutput()
	{
		_init();
	}

	void ddOutput::_init()
	{
		// TODO!!
		throw;
	}

	void ddOutput::loadFile(const std::string ddscatparFile)
	{
		using namespace std;
		// TODO!!
		throw;
		// Open file

		// Use boost to select and open all files in path
		// Iterate through each .fml file and load
	}

	void ddOutput::get(const ddCoords3 &coords, ddOutputSingle &f) const
	{
		if (_data.count(coords)) f = _data[coords];
		else f = NULL;
	}

	void ddOutput::set(const ddCoords3 &coords, const ddOutputSingle &f)
	{
		_data[coords] = f;
	}

	}; // end namespace ddscat

}; // end namespace rtmath

