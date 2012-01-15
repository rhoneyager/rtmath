#include "../rtmath/Stdafx.h"
#include <iostream>
#include <fstream>
#include <boost/filesystem.hpp>
#include <boost/math/special_functions/erf.hpp>
#include <memory>
#include <netcdf.h>
#include <cmath>
#include "../rtmath/ddscat.h"

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

	void ddOutputSingle::writeCDFheader(cdfParams &params) const
	{
		using namespace cdf;
		using namespace std;
		int *p = &params.p[0]; // convenient alias

		// Define dimensions
		// Look at one of the _fs to see the bounds on theta and phi!
		std::set<double> thetas, phis;
		size(thetas,phis);

		int numthetas = thetas.size();
		int numphis = phis.size();
		nc_def_dim(p[fid],"theta",numthetas, &p[dtheta]); // 181 possible thetas: [0,180]
		nc_def_dim(p[fid],"phi",numphis, &p[dphi]); // one possible phi for now
		nc_def_dim(p[fid],"index",16, &p[didnum]); // 16 possible index points
		// Define variables
		nc_def_var(p[fid], "theta", NC_DOUBLE, 1, &p[dtheta], &p[theta]);
		nc_def_var(p[fid], "phi", NC_DOUBLE, 1, &p[dphi], &p[phi]);
		nc_def_var(p[fid], "index", NC_DOUBLE, 1, &p[didnum], &p[idnum]);

		// Set variable attributes for grads plotting
		string sLat("degrees_north");
		string sLon("degrees_east");
		string sT("t");
		string sTu("hours since 2001-1-1 00:00:0.0");
		string strLev("level");
		string strZaxis("z");

		//nc_put_att_text(p[fid], p[cdf::plevs], "units", strLev.size(), strLev.c_str());
		//nc_put_att_text(p[fid], p[cdf::plevs], "axis", strZaxis.size(), strZaxis.c_str());
		nc_put_att_text(p[fid], p[phi], "units", sLat.size(), sLat.c_str());
		nc_put_att_text(p[fid], p[theta], "units", sLon.size(), sLon.c_str());
		nc_put_att_text(p[fid], p[idnum], "axis", sT.size(), sT.c_str());
		nc_put_att_text(p[fid], p[idnum], "units", sTu.size(), sTu.c_str());

		// Define other variables
		int vdimp[] = { p[dtheta], p[dphi], p[didnum] };
		const int dvdimp = 3;

		nc_def_var(p[fid], "A", NC_DOUBLE, dvdimp, vdimp, &p[S]);
		nc_def_var(p[fid], "P", NC_DOUBLE, dvdimp, vdimp, &p[P]);
		nc_def_var(p[fid], "K", NC_DOUBLE, dvdimp, vdimp, &p[K]);
	}

	void ddOutputSingle::writeCDF(const std::string &filename) const
	{
		// Open netcdf file
		//using namespace cdf;
		using namespace std;
		cdfParams params;

		int *p = &params.p[0]; // convenient alias
		nc_create(filename.c_str(), 0, &p[cdf::fid]); // open cdf file for writing

		writeCDFheader(params);
		nc_enddef(p[cdf::fid]);

		// Write variable data

		// Write index variables first
		std::set<double> thetas, phis;
		size(thetas,phis);

		int numthetas = thetas.size();
		int numphis = phis.size();

		double *phia = new double[numphis];
		double *thetaa = new double[numthetas];

		{
			std::set<double>::iterator dt;
			size_t t;
			for (dt = thetas.begin(), t=0; dt != thetas.end(); dt++, t++)
				thetaa[t] = *dt;
			for (dt = phis.begin(), t=0; dt != phis.end(); dt++, t++)
				phia[t] = *dt;
		}
		//double phia[] = {0, 90};
		//double thetaa[181];
		double indexa[16];
		for (size_t i=0;i<16;i++) indexa[i] = i;
		nc_put_var_double(p[cdf::fid],p[cdf::theta],thetaa );
		nc_put_var_double(p[cdf::fid],p[cdf::idnum],indexa );
		nc_put_var_double(p[cdf::fid],p[cdf::phi],phia );

		delete[] thetaa;
		delete[] phia;

		// Loop through all _fs
		// Generate mueller and extinction matrices
		// Write out all values, one at a time
		std::map<ddCoords, ddScattMatrix, ddCoordsComp>::const_iterator it;
		size_t counter = 0; // keeps track of indices for theta and phi
		for (it = _fs.begin(), counter = 0; it != _fs.end(); it++, counter++)
		{
			//double theta = it->second.theta();
			//double phi = it->second.phi();
			//double Pnn[4][4], Knn[4][4];
			//double Sr[4], Si[4];
			//it->second.mueller(Pnn);
			//it->second.extinction(Knn);

			//int vdimp[] = { p[dtheta], p[dphi], p[dindex] };

			//for (int theta=0; theta <= 180; theta++)
			{
				//for (int phi = 0; phi < 2; phi++)
				{
					// Each variable has different types of indices.
					// P and K are the same (16), S has only 8.
					for (int i=0;i<16;i++)
					{
						// Output P and K
						size_t index[3]; // array for netcdf location specifying
						//index[0] = theta;// BAD
						//index[1] = phi; // BAD
						index[0] = counter / numphis;
						index[1] = counter % numphis;
						index[2] = i; // GOOD

						double data = 0;
						// I'm overriding my indices here to convert to single dimension array
						data = it->second.Pnn[(i/4)][(i%4)];
						nc_put_var1_double(p[cdf::fid],p[cdf::P],index,&data );
						data = it->second.Knn[(i/4)][(i%4)];
						nc_put_var1_double(p[cdf::fid],p[cdf::K],index,&data );

						if (i<8)
						{
							// Output S too
							int k = (i / 2) % 2;
							int l = i / 4;
							int j = i % 2; // selects real or imaginary part
							if (j == 0) // real
								data = it->second.vals[l][k].real();
							else // imaginary
								data = it->second.vals[l][k].imag();
							nc_put_var1_double(p[cdf::fid],p[cdf::S],index,&data);
						}
						if (i>=8)
						{
							data = 0;
							nc_put_var1_double(p[cdf::fid],p[cdf::S],index,&data);
						}
					}
				}
			}
		}

		// Close file
		nc_close(p[cdf::fid]);
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

	double ddOutputEnsembleIso::weight(const ddCoords3 &coords, const ddCoords3 &delta)
	{
		// Number of elements provided upon initialization
		return 1.0 / (double) this->_ensemble.size();
	}

	double ddOutputEnsembleGaussianPhi::weight(const ddCoords3 &coords, const ddCoords3 &delta)
	{
		double scaled = coords.phi - mu;
		double scaledb = delta.phi - mu;
		// NOTE: msvc2010 has no erf function support in either c99 or c++2011!!!
		// I'll use boost libraries here
		// Weights are doubled because orientations are +-  <--- 2 domains of interest
		double Pa = 1.0 + boost::math::erf( (scaled + scaledb) / (sqrt(2.0) * sigma));
		double Pb = 1.0 + boost::math::erf(scaled / (sqrt(2.0) * sigma));
		double P = Pa - Pb;
		return P;
	}

	double ddOutputEnsembleGaussianTheta::weight(const ddCoords3 &coords, const ddCoords3 &delta)
	{
		using namespace std;
		// Use cmath erf function because we need to integrate the error function
		double scaled = coords.theta - mu;
		double scaledb = delta.theta - mu;
		// NOTE: msvc2010 has no erf function support in either c99 or c++2011!!!
		double Pa = 1.0 + boost::math::erf( (scaled + scaledb) / (sqrt(2.0) * sigma));
		double Pb = 1.0 + boost::math::erf(scaled / (sqrt(2.0) * sigma));
		double P = Pa - Pb;
		//cerr << " Pa " << Pa << " Pb " << Pb << " P " << P << endl;
		return P;
	}

	void ddOutputEnsemble::generate()
	{
		// Take the set of ddOutputSingle, and average each rotation's s and phase matrices
		// Report the output in standard ddOutputSingle elements, as we are a derived class,
		// and it makes it easy this way
		std::map<ddCoords3, ddOutputSingle, ddCoordsComp>::const_iterator it, kt;
		double wt = 0, wtall = 0, wtwt = 1.0;
		//double weight = 1.0 / (double) numElems;
		// Assume that all ddOutputSingle have the same coordinate set for _fs
		//ddOutputSingle res;
		res._fs.clear();

		throw; // I have to fix bugs with total weighting

		for (it=_ensemble.begin(); it != _ensemble.end(); it++)
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
					b.lock = true; // Suppress overwriting of P and K (normally calculated from vals[][])
					res._fs[a] = b;
					resf = res._fs.find(srcf->first); // Ugly, but fast to code
				}
				// resf is the iterator pointing to the ensemble result's value for _fs
				// So, it points to a ddScattMartix
				// srcf is the iterator pointing to the current scattMatrix which is 
				// multiplied by the weight and then added to the value in resf.

				// Using matrixops for ease (and not having to write yet another set of loops)
				kt = it;
				kt++;

				// Also factor in the discrepancy caused by not starting precisely at zero
				// (missing most important part of erf)
				if (it==_ensemble.begin() && needwtwt() == true) // Must hit on first for loop iteration
				{
					ddCoords3 zc(0,0,0);
					wtwt = 1.0 - weight(zc,it->first); // Scaling factor for all subsequent weights
					if (wtwt < 0) throw; // Should never have the weight being greater than 1.
					if (wtwt == 0) throw; // to avoid division by zero
				}

				// Here come the weights
				if (kt == _ensemble.end()) wt = 1.0 - wtall; // prev 3 lines so that delta calc works
				else wt = weight(it->first, kt->first) / wtwt;

				wtall += wt;
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

	}; // end namespace ddscat

}; // end namespace rtmath

