#include "../rtmath/Stdafx.h"
#include <iostream>
#include <fstream>
#include <boost/filesystem.hpp>
#include <boost/math/special_functions/erf.hpp>
#include <memory>
#include <netcdf.h>
#include <cmath>
#include "../rtmath/ddscat.h"
#include "../rtmath/ddweights.h"

namespace rtmath {

	namespace ddscat {

	ddScattMatrix::ddScattMatrix()
	{
		_theta = 0;
		_phi = 0;
		_wavelength = 0;
		lock = false;

		for (int i=0;i<4;i++)
			for(int j=0;j<4;j++)
			{
				Pnn[i][j] = 0;
				Knn[i][j] = 0;
			}

		// vals should be auto-initializing
	}

	ddScattMatrix::ddScattMatrix(double theta, double phi, double wavelength)
	{
		_theta = theta;
		_phi = phi;
		_wavelength = wavelength;
		lock = false;
		for (int i=0;i<4;i++)
			for(int j=0;j<4;j++)
			{
				Pnn[i][j] = 0;
				Knn[i][j] = 0;
			}
	}

	void ddScattMatrix::genS()
	{
		using namespace std;
		//complex<double> S[4];
		complex<double> i(0,1);

		complex<double> e01x(0,0), e01y(1,0), e01z(0,0), e02x(0,0), e02y(0,0), e02z(1,0);
		complex<double> a = conj(e01y), b=conj(e01z), c=conj(e02y), d=conj(e02z);

		//double cp = cos(2.0*M_PI*phi()/180.0);
		double cp = cos(phi() * M_PI / 180.0);
		//double sp = sin(2.0*M_PI*phi()/180.0);
		double sp = sin(phi() * M_PI / 180.0);
		S[0] = -i * ( vals[1][0] * (b * cp - a * sp) + vals[1][1] * (d * cp - c * sp) );
		S[1] = -i * (vals[0][0] * (a*cp + b * sp) + vals[0][1] * (c * cp + d * sp) );
		S[2] = i * ( vals[0][0] * (b * cp - a * sp) + vals[0][1] * (d * cp - c * sp) );
		S[3] = i * ( vals[1][0] * (a*cp + b * sp) + vals[1][1] * (c*cp + d * sp) );
	}

	void ddScattMatrix::mueller(double Snn[4][4]) const
	{
		//genS();
		rtmath::scattMatrix::_genMuellerMatrix(Snn,S);
	}

	void ddScattMatrix::mueller(matrixop &res) const
	{
		double Snn[4][4];
		mueller(Snn);
		res.fromDoubleArray(&Snn[0][0]);
	}

	void ddScattMatrix::extinction(double Knn[4][4]) const
	{
		//genS();
		double k = 2.0 * M_PI / _wavelength;
		// _wavelength is in micrometers. should I convert to standard units?

		rtmath::scattMatrix::_genExtinctionMatrix(Knn, S, k);
	}

	void ddScattMatrix::extinction(matrixop &res) const
	{
		double Knn[4][4];
		double k = 2.0 * M_PI / _wavelength;
		rtmath::scattMatrix::_genExtinctionMatrix(Knn, S, k);
		res.fromDoubleArray(&Knn[0][0]);
	}

	ddScattMatrix& ddScattMatrix::operator=(const ddScattMatrix &rhs)
	{
		// Check for pointer equality. If equal, return.
		if (this == &rhs) return *this;
		_theta = rhs._theta;
		_phi = rhs._phi;
		_wavelength = rhs._wavelength;
		for (size_t i=0; i<2; i++)
			for (size_t j=0; j<2; j++)
				vals[i][j] = rhs.vals[i][j];
		update();
		return *this;
	}

	ddScattMatrix ddScattMatrix::operator+(const ddScattMatrix &rhs)
	{
		// Used when adding two ddScattMatrices
		ddScattMatrix res;
		for (size_t i=0;i<2;i++)
			for (size_t j=0;j<2;j++)
				res.vals[i][j] = this->vals[i][j] + rhs.vals[i][j];
		res.update();
		return res;
	}

	ddScattMatrix& ddScattMatrix::operator+=(const ddScattMatrix &rhs)
	{
		for (size_t i=0;i<2;i++)
			for (size_t j=0;j<2;j++)
				this->vals[i][j] = this->vals[i][j] + rhs.vals[i][j];
		update();
		return *this;
	}

	void ddScattMatrix::update()
	{
		if (lock) return;
		genS();
		mueller(Pnn);
		extinction(Knn);
	}

	bool ddScattMatrix::operator==(const ddScattMatrix &rhs) const
	{
		if (this == &rhs) return true;
		if (_theta != rhs._theta) return false;
		if (_phi != rhs._phi) return false;
		for (size_t i=0; i<2; i++)
			for (size_t j=0; j<2; j++)
				if (vals[i][j] != rhs.vals[i][j]) return false;
		return true;
	}

	bool ddScattMatrix::operator!=(const ddScattMatrix &rhs) const
	{
		return !operator==(rhs);
	}

	void ddScattMatrix::writeCSV(const std::string &filename) const
	{
		using namespace std;
		ofstream out(filename.c_str());
		out << "CSV output for (theta,phi,wavelength)=\n";

		//streambuf *filebuf;
		//filebuf = out.rdbuf();
		writeCSV(out);
	}

	void ddScattMatrix::writeCSV(std::ofstream &out) const
	{
		using namespace std;
		//ofstream out;
		//out.rdbuf(filebuf);
		//streambuf *filebuf;
		//filebuf = out.rdbuf();
		//cout.rdbuf(filebuf);
		//out << "CSV output for (theta,phi,wavelength)=\n";

		out << _theta << ", " << _phi << ", " << _wavelength << ", ";

		for (size_t i=0; i<4; i++)
		{
			for (size_t j=0; j<4; j++)
			{
				out << Pnn[i][j] << ", ";
			}
		}

		for (size_t i=0; i<4; i++)
		{
			for (size_t j=0; j<4; j++)
			{
				out << Knn[i][j] << ", ";
			}
		}
		out << endl;
	}

	void ddScattMatrix::print() const
	{
		using namespace std;
		cout << "Matrices for theta " << _theta << " phi "
				<< _phi << " wavelength " << _wavelength << endl;

		cout << "f" << endl;
		for (size_t i=0; i<2; i++)
			for (size_t j=0; j<2; j++)
				cout << i << "," << j << "\t" << vals[i][j] << endl;
		cout << "S" << endl;
		for (size_t i=0; i<4; i++)
		{
			cout << "\t" <<  S[i] << endl;
		}

		cout << "Mueller" << endl;
		cout << _theta << "\t" << _phi << "\t" << _wavelength << "\t";
		//update();
		for (size_t i=0; i<4; i++)
		{
			for (size_t j=0; j<4; j++)
			{
				cout << Pnn[i][j] << "\t";
			}
			cout << endl;
		}
		cout << endl;

		cout << "Extinction" << endl;
		for (size_t i=0; i<4; i++)
		{
			for (size_t j=0; j<4; j++)
			{
				cout << Knn[i][j] << "\t";
			}
			cout << endl;
		}
		cout << endl;

	}

	ddOutputSingle& ddOutputSingle::operator=(const ddOutputSingle &rhs)
	{
		// Check for pointer equality. If equal, return.
		if (this == &rhs) return *this;
		_Beta = rhs._Beta;
		_Theta = rhs._Theta;
		_Phi = rhs._Phi;
		_wavelength = rhs._wavelength;
		_numDipoles = rhs._numDipoles;
		_reff = rhs._reff;
		_fs = rhs._fs;
		filename = rhs.filename;
		for (size_t i=0; i<3; i++)
			_shape[i] = rhs._shape[i];
		return *this;
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

	ddOutputSingle::ddOutputSingle(double beta, double theta, double phi, double wavelength)
	{
		_init();
		_Beta = beta;
		_Theta = theta;
		_Phi = phi;
		_wavelength = wavelength;
	}

	std::shared_ptr<matrixop> ddOutputSingle::eval(double alpha) const
	{
		// Evaluate the phase function at a given single-scattering angle
		// Not the most accurate method, as it assumes that there is only one
		// degree of freedom. However, it works by selecting the element in _fs
		// that has alpha equal to the request.
		// If not found, it will try and interpolate linearly to get a result.
		// TODO: eventually, have it save precomputed values using hashes
		//std::map<double, ddScattMatrix*> rankings; // Pointer set of ranked _fs
		std::map<ddCoords, ddScattMatrix, ddCoordsComp>::const_iterator it;
		ddScattMatrix prev, next;
		double aprev = 0, anext = 0;
		for (it = _fs.begin(); it != _fs.end(); it++)
		{
			// First, check for an exact alpha match
			// If so, just return
			if (it->first.alpha == alpha)
			{
				//it->second.mueller();
				std::shared_ptr<matrixop> res( new matrixop(it->second.mueller()));
				return res;
			}
			// If not, add to rankings
			//rankings[it->first.alpha] = &(it->second);
			if (it->first.alpha < anext && it->first.alpha > alpha)
			{
				anext = it->first.alpha;
				next = (it->second);
			}
			if (it->first.alpha > aprev && it->first.alpha < alpha)
			{
				aprev = it->first.alpha;
				prev = (it->second);
			}
		}
		// If we've hit this point, linearly interpolate to get a good alpha
		// Factors are weights for average on scale of 0 to 1.
		// Weird formulation, but it's just how I think...
		double facta = (anext - alpha)/ (anext - aprev);
		double factb = 1 - facta;
		matrixop resi = (prev.mueller() * facta) + (next.mueller() *factb);
		std::shared_ptr<matrixop> res( new matrixop(resi));
		return res;
	}

	void ddOutputSingle::getF(const ddCoords &coords, ddScattMatrix &f) const
	{
		if (_fs.count(coords)) f = _fs[coords];
		//else f = NULL;
	}

	void ddOutputSingle::setF(const ddCoords &coords, const ddScattMatrix &f)
	{
		_fs[coords] = f;
		_fs[coords].update();
	}

	void ddOutputSingle::size(std::set<double> &thetas, std::set<double> &phis) const
	{
		thetas.clear();
		phis.clear();

		std::map<ddCoords, ddScattMatrix, ddCoordsComp>::const_iterator it;
		for (it=_fs.begin(); it != _fs.end(); it++)
		{
			double th = it->first.theta;
			if (thetas.count(th) == 0)
				thetas.insert(th);
			double ph = it->first.phi;
			if (phis.count(ph) == 0)
				phis.insert(ph);
		}
	}

	void ddOutputSingle::interpolate(const ddCoords &coords, ddScattMatrix &res) const
	{
		// I already have the set of scattering matrices available. I'll use them
		// to perform a linear interpolation. This interpolation may be crude, but
		// it is necessary it I want to generate matrices with angles along the appropriate
		// quadrature points. I really with that ddscat could do this for me.

		// Conveniently, all of my ddscat stuff varies theta from 0 to 180, while
		// phi is either 0 or 90 degrees (covering co- and orthogonal polarizations).

		// So, I really should look at phi first
		// Find the four matrices needed for the planar average
		throw rtmath::debug::xUnimplementedFunction();
		// Store the resulting angles here
		ddCoords col, cou, cxl, cxu;

		std::map<ddCoords, ddScattMatrix, ddCoordsComp>::const_iterator it;
		for (it = _fs.begin(); it != _fs.end(); ++it)
		{
			// Find best value underneath and overshooting for each phi
			if (it->first.theta > col.theta && it->first.phi == 0 && it->first.theta <= coords.theta)
				col = it->first;
			if (it->first.theta > cxl.theta && it->first.phi == 90 && it->first.theta <= coords.theta)
				cxl = it->first;
			if (it->first.theta < cou.theta && it->first.phi == 0 && it->first.theta >= coords.theta)
				cou = it->first;
			if (it->first.theta > cxu.theta && it->first.phi == 90 && it->first.theta >= coords.theta)
				cxu = it->first;
		}

		// Okay, we have the set of the four closest coordinates on the sphere.
		// Average the scattmatrices for each phi
		double facttheta = (cou.theta - coords.theta) / (cou.theta - col.theta);
		double factphi = (cxu.phi - coords.phi) / (cxu.phi - cou.phi);

		res.lock = true;
		for (size_t i=0;i<4;i++)
		{
			for (size_t j=0;j<4;j++)
			{
				// Do interpolation on Pnn and Knn
				res.Pnn[i][j] = factphi * (facttheta * _fs[col].Pnn[i][j] + (1.0-facttheta) * _fs[cou].Pnn[i][j]);
				res.Pnn[i][j] += (1.0 - factphi) * (facttheta * _fs[cxl].Pnn[i][j] + (1.0-facttheta) * _fs[cxu].Pnn[i][j]);
				res.Knn[i][j] = factphi * (facttheta * _fs[col].Knn[i][j] + (1.0-facttheta) * _fs[cou].Knn[i][j]);
				res.Knn[i][j] += (1.0 - factphi) * (facttheta * _fs[cxl].Knn[i][j] + (1.0-facttheta) * _fs[cxu].Knn[i][j]);
			}
		}
	}

	void ddOutputSingle::loadFile(const std::string &filename)
	{
		using namespace std;
		// File loading routine is important!
		// Load a standard .fml file. Parse each line for certain key words.
		bool dataseg = false;
		this->filename = filename;
		//cout << "Loading " << filename << endl;
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
				// theta is seg[0], phi is seg[1]
				double theta, phi, re, im;
				lss >> theta;
				lss >> phi;
				// _wavelength will be saved so that extinction matrix may be calculated
				ddScattMatrix nscat(theta,phi,_wavelength);
				// For the next eight quantities, load the complex f
				// NOTE: The ordering in the file if f11, f21!!!, f12!!!, f22
				// It's annoyingly backwards in the middle, which was a bug
				for (size_t i=0; i<4; i++)
				{
					lss >> re;
					lss >> im;
					complex<double> nval(re,im);
					size_t j= i % 2;
					size_t k= i / 2;
					nscat.vals[j][k] = nval;
				}
				// Save to the map
				if (_fs.count(ddCoords(theta,phi)) == 0)
					setF(ddCoords(theta,phi),nscat);
			} else {
				// Still in header segment
				string junk;
				// Search for key strings in file

				// BETA
				if (lin.find("BETA") != string::npos)
				{
					lss >> junk; // get rid of first word
					lss >> junk;
					lss >> _Beta;
				}
				// THETA
				if (lin.find("THETA") != string::npos)
				{
					lss >> junk; // get rid of first word
					// Theta is unlike Beta and Phi, as there is
					// no space between THETA and =
					lss >> _Theta;
				}
				// PHI
				if (lin.find("PHI") != string::npos)
				{
					lss >> junk; // get rid of first word
					lss >> junk;
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
					// BAD --- WAVE runs against size...
					//lss >> junk; // get rid of first word
					//lss >> _wavelength;
					// Instead, read wave from column 7 (starting at 0) to 17
					_wavelength = atof( lin.substr( 7, 10 ).c_str() );
				}
				// theta --- indicates last line of header
				if (lin.find("Re(f_11)") != string::npos)
				{
					dataseg = true;
				}
			}
		}
		//print();
	}

	void ddOutputSingle::print() const
	{
		using namespace std;
		cerr << "ddOutputSingle output for " << _Beta << ", " << _Theta << ", " << _Phi << endl;
		cerr << _wavelength << ", " << _numDipoles << ", " << _reff << endl;
		cerr << endl;

		std::map<ddCoords, ddScattMatrix, ddCoordsComp>::const_iterator it, e = _fs.end();
		e--;
		for (it = _fs.begin(); it != e; it++)
		{
			it->second.print();
		}
	}

	void ddOutputSingle::writeCSV(const std::string &filename) const
	{
		using namespace std;
		ofstream out(filename.c_str());
		//streambuf *filebuf;
		//filebuf = out.rdbuf();
		//cout.rdbuf(filebuf);

		out << "ddOutputSingle output for " << _Beta << ", " << _Theta << ", " << _Phi << endl;
		out << _wavelength << ", " << _numDipoles << ", " << _reff << endl;
		out << endl;

		std::map<ddCoords, ddScattMatrix, ddCoordsComp>::const_iterator it, e = _fs.end();
		e--;
		for (it = _fs.begin(); it != e; it++)
		{
			//it->second.writeCSV(filebuf);
			it->second.writeCSV(out);
		}
	}

	ddOutput::ddOutput()
	{
		_init();
	}

	void ddOutput::_init()
	{
		// TODO!!
		//throw;
	}

	void ddOutput::loadFile(const std::string &ddscatparFile)
	{
		using namespace std;
		using namespace boost::filesystem;

		boost::filesystem::path p(ddscatparFile.c_str()), dir, ddfile;
		if (!exists(p)) throw;
		if (is_regular_file(p))
		{
			// Need to extract the directory path
			ddfile = p;
			dir = p.parent_path();
		}
		if (is_directory(p))
		{
			// Need to extract the path for ddscat.par
			dir = p;
			ddfile = p / "ddscat.par";
		}
		if (!exists(ddfile))
		{
			cout << "Invalid ddOutput directory. No ddscat.par\n";
			throw;
		}
		cerr << "Directory: " << dir << endl;
		cerr << "ddscat par: " << ddfile << endl;

		// Use boost to select and open all files in path
		// Iterate through each .fml file and load
		vector<path> files;
		copy(directory_iterator(dir), directory_iterator(), back_inserter(files));

		cerr << "There are " << files.size() << " files in the directory." << endl;
		// Iterate through file list for .fml files
		vector<path>::const_iterator it;
		size_t counter = 0;
		for (it = files.begin(); it != files.end(); it++)
		{
			//cout << *it << "\t" << it->extension() << endl;
			if (it->extension().string() == string(".fml") )
			{
				// Load the file
				//cout << "Loading " << it->string() << endl;
				ddOutputSingle news(it->string());
				ddCoords3 crds(news._Beta, news._Theta, news._Phi);
				if (_data.count(crds) == 0)
					set(crds, news);
				counter++;
			}
		}
		cerr << "Of these, " << counter << " fml files were loaded." << endl;
	}

	void ddOutput::get(const ddCoords3 &coords, ddOutputSingle &f) const
	{
		if (_data.count(coords)) f = _data[coords];
		//else f = NULL;
	}

	void ddOutput::set(const ddCoords3 &coords, const ddOutputSingle &f)
	{
		_data[coords] = f;
	}

	void ddOutputEnsembleGaussian::_genWeights(const std::set<double> &points,
		const std::map<double, unsigned int> &recs)
	{
		gaussianPosWeights a(_sigma, points);
		a.getWeights(_weights);

		// Tweak weights to account for multiple phi recurrances
		// Tweak divided the weight by the number of times it pops up

		std::set<double>::const_iterator it;
		double sum = 0;
		for (it = points.begin(); it != points.end(); it++)
		{
			_weights[*it] /= (double) recs.at(*it);
			sum += _weights[*it];
		}

		// And then they all get rescaled to unity
		for (it = points.begin(); it != points.end(); it++)
		{
			_weights[*it] /= sum;
		}

	}

	void ddOutputEnsembleGaussianPhi::generate()
	{
		// ensemble should be set. 
		// First, collect all phi into a set and generate the weights
		using namespace std;
		set<double> phis;
		map<double, unsigned int> phiRecs;
		std::map<ddCoords3, ddOutputSingle, ddCoordsComp>::const_iterator it, ot, kt;
		for (it = ensemble.begin(); it != ensemble.end(); it++)
		{
			// Note: normally, it would fail if multiple phis are introduced.
			// I'm just recording the recurrance for a weight adjustment later on.
			// Either that, or I could sum over those elements with the same phi,
			// but that will lead to an extra summation step later on
			double val = it->first.phi;
			if (phis.count(val) == 0)
				phis.insert(val);
			if (phiRecs.count(val))
				phiRecs[val]++;
			else phiRecs[val] = 1;
		}

		_genWeights(phis, phiRecs);

		// We now have weights. Time to take the elements in the ensemble
		// and sum according to the weighting function results.

		res._fs.clear();

		matrixop Peff(2,4,4), Keff(2,4,4), Pn(2,4,4), Kn(2,4,4);
		for (it = ensemble.begin(); it != ensemble.end(); it++)
		{
			// Iterate through and match the _fs element with res' element

			std::map<ddCoords, ddScattMatrix, ddCoordsComp>::iterator resf;
			std::map<ddCoords, ddScattMatrix, ddCoordsComp>::const_iterator srcf, srcfp;
			for (srcf=it->second._fs.begin(); srcf!=it->second._fs.end(); srcf++)
			{
				// Find the matching resf based on the key
				resf = res._fs.find(srcf->first);
				if (resf == res._fs.end()) // Iterator not found. Add a new key.
				{
					std::pair< std::map<ddCoords, ddScattMatrix, ddCoordsComp>::iterator, bool> op;
					ddCoords a( srcf->first );
					ddScattMatrix b(a.theta, a.phi, 0);
					// TODO: check how fs add together. WOuld be easier that way.
					b.lock = true; // Suppress overwriting of P and K (normally calculated from vals[][])
					res._fs[a] = b;
					resf = res._fs.find(srcf->first); // Ugly, but fast to code
				}
				// resf is the iterator pointing to the ensemble result's value for _fs
				// So, it points to a ddScattMartix
				// srcf is the iterator pointing to the current scattMatrix which is 
				// multiplied by the weight and then added to the value in resf.

				// Using matrixops for ease (and not having to write yet another set of loops)

				double wt = 0;
				wt = weight(it->first);

				matrixop Peff(2,4,4), Keff(2,4,4);
				matrixop Pn(2,4,4),   Kn(2,4,4);
				Peff.fromDoubleArray(&(resf->second.Pnn)[0][0]);
				Keff.fromDoubleArray(&(resf->second.Knn)[0][0]);
				Pn.fromDoubleArray(&(srcf->second.Pnn)[0][0]);
				Kn.fromDoubleArray(&(srcf->second.Knn)[0][0]);

				Peff = Peff + (Pn * wt);
				Keff = Keff + (Kn * wt);

				Peff.toDoubleArray(&(resf->second.Pnn)[0][0]);
				Keff.toDoubleArray(&(resf->second.Knn)[0][0]);
			}
		}
	}

	double ddOutputEnsembleGaussianPhi::weight(const ddCoords3 &coords) const
	{
		if (_weights.count(coords.phi))
			return _weights.at(coords.phi);
		throw rtmath::debug::xAssert("For some reason, the appropriate weight was not computed!");
		return 0;
	}

	}; // end namespace ddscat

}; // end namespace rtmath

