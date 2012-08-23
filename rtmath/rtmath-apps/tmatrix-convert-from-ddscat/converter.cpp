#include "../../rtmath/rtmath/ROOTlink.h"
#include <algorithm>
#include <sstream>
#include "converter.h"
#include "../../rtmath/rtmath/ddscat/shapestats.h"
#include "../../rtmath/rtmath/ddscat/ddpar.h"
#include "../../rtmath/rtmath/ddscat/rotations.h"
#include "../../rtmath/rtmath/command.h"
#include "../../rtmath/rtmath/units.h"
#include "../../rtmath/rtmath/refract.h"
#include "../../rtmath/rtmath/serialization.h"
#include "../../deps/tmatrix/src/headers/tmatrix.h"

namespace {
	const size_t cmax = 1024;
	struct convHeader
	{
		Float_t temp, freq, wvlen, dipoleSpacing,
			shiv_nu, aeff, vFrac;
/*		char vFracMeth[cmax],
		     dielMeth[cmax],
		     shapeMeth[cmax],
		     parFile[cmax],
		     shapeFile[cmax];
*/
		convHeader()
		{
			temp = 0;
			freq = 0;
			wvlen = 0;
			dipoleSpacing = 0;
			shiv_nu = 0;
			aeff = 0;
			vFrac = 0;
		}
	};
	const char* convHeaderStr = 
		"temp:freq:wvlen:dipoleSpacing:shiv_nu:aeff:vFrac"; //":vFracMeth/C:dielMeth/C:shapeMeth/C:parFile/C:shapeFile/C";

	const char* tmOutConverterStr = 
		"Sreal[4]:Simag[4]:P[16]";
	struct tmOutConverter
	{
		Float_t Sreal[4], Simag[4];
		Float_t P[16];
		void import(const tmatrix::tmatrixOutVars &in)
		{
			Sreal[0] = in.S[0].real();
			Simag[0] = in.S[0].imag();
			Sreal[1] = in.S[1].real();
			Simag[1] = in.S[1].imag();
			Sreal[2] = in.S[2].real();
			Simag[2] = in.S[2].imag();
			Sreal[3] = in.S[3].real();
			Simag[3] = in.S[3].imag();

			P[0] = in.P[0][0];
			P[1] = in.P[0][1];
			P[2] = in.P[0][2];
			P[3] = in.P[0][3];
			P[4] = in.P[1][0];
			P[5] = in.P[1][1];
			P[6] = in.P[1][2];
			P[7] = in.P[1][3];
			P[8] = in.P[2][0];
			P[9] = in.P[2][1];
			P[10] = in.P[2][2];
			P[11] = in.P[2][3];
			P[12] = in.P[3][0];
			P[13] = in.P[3][1];
			P[14] = in.P[3][2];
			P[15] = in.P[3][3];
		}
	};
};

fileconverter::fileconverter()
{
	temp = 0;
	frequency = 0;
	dipoleSpacing = 0;
	tmatrix = false;
}

void fileconverter::doTMATRIX(bool tm)
{
	tmatrix = tm;
}

void fileconverter::setDDPARfile(const std::string &file)
{
	ddparFile = file;
}

void fileconverter::setTemp(double T)
{
	temp = T;
}

void fileconverter::setDipoleSpacing(double d)
{
	dipoleSpacing = d;
}

void fileconverter::setFreq(double f)
{
	frequency = f;
}

void fileconverter::setShapeMethod(const std::string &meth)
{
	shapeMeth = meth;
}

void fileconverter::setDielMethod(const std::string &meth, double nu)
{
	this->nu = nu;
	dielMeth = meth;
}

void fileconverter::setVolFracMethod(const std::string &meth)
{
	volMeth = meth;
}

void fileconverter::setStats(boost::shared_ptr<rtmath::ddscat::shapeFileStats> s)
{
	stats = s;
}

void fileconverter::convert(const std::string &outfile, bool ROOToutput) const
{
	using namespace std;

	// Get volume fraction from the shape statistics. This has no bearing on the 
	// other dimensioning calculations. It is purely for the dielectric calculation.
	double frac, aeff;
	// aeff is a united quantity. Needs to be rescaled from dipole coords into microns!
	{
		if (volMeth == "Minimal circumscribing sphere")
		{
			frac = stats->f_circum_sphere;
			aeff = stats->a_circum_sphere;
		} else if (volMeth == "Convex hull")
		{
			frac = stats->f_convex_hull;
			aeff = stats->aeff_V_convex_hull;
		} else if (volMeth == "Max Ellipsoid")
		{
			frac = stats->f_ellipsoid_max;
			aeff = stats->aeff_ellipsoid_max;
		} else if (volMeth == "RMS Ellipsoid")
		{
			frac = stats->f_ellipsoid_rms;
			aeff = stats->aeff_ellipsoid_rms;
		} else {
			throw rtmath::debug::xBadInput(volMeth.c_str());
		}
		// aeff is currently not in microns. I need the 
		// interdipole spacing, as read from the ddscat.par file.
		// Note: ddscat.par has the aeff for only the dipoles, not the whole shape!
	}
	
	// Calculate shape dimensioning (ellipsoid, sphere, ...)
	// Volume used for shape dimensioning defined above
	double asp; // ratio of horizontal to rotational axes; >1 for oblate, <1 for prolate
	// Initial rotation is about x, which would be the rotational axes
	// So, the horizontal axes are y and z
	// Need to take the mean of two of the axes
	{
		auto r = stats->rotations.begin();
		// Parse shapeMethod to figure this out
		// TODO: need to record stats of the max aspect ratio / maximally distorted 
		// line, which should occur along the two furthest points
		if (shapeMeth == "Equiv Aeff Sphere")
		{
			asp = 1.0;
		} else if (shapeMeth == "Same RMS aspect ratio")
		{
			double x = r->as_rms.get(2,0,0);
			double y = r->as_rms.get(2,1,0);
			double z = r->as_rms.get(2,2,0);

			asp = (0.5 * (y + z)) / x;
			//stats->
		} else if (shapeMeth == "Same real aspect ratio")
		{
			double x = r->as_abs.get(2,0,0);
			double y = r->as_abs.get(2,1,0);
			double z = r->as_abs.get(2,2,0);

			asp = (0.5 * (y + z)) / x;
		} else {
			throw rtmath::debug::xBadInput(shapeMeth.c_str());
		}
	}

	// Read ddscat.par file
	// Extract frequencies, reff, rotations, scattering angles and incident vectors
	rtmath::ddscat::ddPar par(ddparFile);
	set<double> freqs; // GHz
	set<double> wavelengths; // um
	if (!frequency)
	{
		double min, max;
		size_t n;
		string spacing;
		par.getWavelengths(min,max,n,spacing);
		// Wavelengths are in microns. Convert interval to frequencies
		ostringstream s;
		std::transform(spacing.begin(), spacing.end(), spacing.begin(), ::tolower);
		s << min << ":" << n << ":" << max << ":" << spacing;
		string ss(s.str());

		rtmath::config::splitSet<double>(ss,wavelengths);
		rtmath::units::conv_spec c("um","GHz");
		for (auto it = wavelengths.begin(); it != wavelengths.end(); it++)
		{
			double freq = c.convert(*it);
			if (freqs.count(freq) == 0)
				freqs.insert(freq);
		}
	} else {
		freqs.insert(frequency);
		rtmath::units::conv_spec c("GHz","um");
		double wvlen = c.convert(frequency);
		wavelengths.insert(wvlen);
	}

	// reff is needed, since the pure shapefile gives everything in terms of the interdipole spacing
	set<double> aeffs;
	double d;
	{
		double min, max;
		size_t n;
		string spacing;
		par.getAeff(min,max,n,spacing);
		// Aeffs are in microns. Convert interval to frequencies
		ostringstream s;
		std::transform(spacing.begin(),spacing.end(),spacing.begin(), ::tolower);
		s << min << ":" << n << ":" << max << ":" << spacing;
		string ss(s.str());
		rtmath::config::splitSet<double>(ss,aeffs);

		// Given my knowledge of aeffs (um), aeff can be rescaled factoring in the 
		// dipole spacings
		double aeff_dipoles_const = stats->aeff_dipoles_const;
		double aeff_ddpar = *(aeffs.begin());
		if (dipoleSpacing)
			d = dipoleSpacing;
		else
			d = aeff_ddpar / aeff_dipoles_const;
		aeff *= d; // Now it is in microns
	}
	

	// rotations and scattering angles combine to get overall choice of angles
	set<double> betas, thetas, phis;
	map<double, set<double> > angles; // (phi, set of thetas)
	{
		// valid only if cmdfrm is in the lab frame 'LFRAME'
		// throw if tframe (another approach is needed)
		string sframe;
		par.getCMDFRM(sframe);
		if (sframe != "LFRAME") 
			throw rtmath::debug::xBadInput("Only LFRAME supported");

		// The ellipsoid starts oriented with the z axis as the axis of rotation.
		// alpha and beta are then varied to rotate it about this axis. 
		rtmath::ddscat::rotations rots;
		par.getRots(rots);
		//set<double> betas, thetas, phis; // above
		rots.betas(betas);
		rots.thetas(thetas);
		rots.phis(phis);

		// the mapping is ddstat theta = tmatrix beta
		// tmatrix alpha = ddscat phi (but not really. tmatrix ordering is different, 
		// and given rot symmetry, it doesn't matter)

		size_t n = par.numPlanes();
		for (size_t i=1; i<=n; i++)
		{
			double phi, thetan_min, thetan_max, dtheta;
			par.getPlane(i, phi, thetan_min, thetan_max, dtheta);
			ostringstream sT;
			sT << thetan_min << ":" << dtheta << ":" << thetan_max;
			set<double> scatThetas;
			rtmath::config::splitSet<double>(sT.str(), scatThetas);
			if (angles.count(phi) == 0)
			{
				set<double> ns;
				angles[phi] = ns;
			}
			for (auto it = scatThetas.begin(); it != scatThetas.end(); ++it)
				angles[phi].insert(*it);
		}
	}

	// Calculate refractive index
	complex<double> mRes;
	{
		complex<double> mIce;
		complex<double> mAir(1.0,0); // TODO: use exact refractive index
		rtmath::refract::mice(*(freqs.begin()), temp, mIce);
		if (dielMeth == "Sihvola")
		{
			rtmath::refract::sihvola(mIce,mAir,frac,nu,mRes);
		} else if (dielMeth == "Debye")
		{
			rtmath::refract::debyeDry(mIce,mAir,frac,mRes);
		} else if (dielMeth == "Maxwell-Garnett")
		{
			rtmath::refract::maxwellGarnettSimple(mIce,mAir,frac,mRes);
		} else {
			throw rtmath::debug::xBadInput(dielMeth.c_str());
		}
	}

	// Write output
	{
		using namespace tmatrix;
		tmatrixInVars in;
		tmOutConverter ocnv;

		TFile *rfile = nullptr;
		string rfilename = outfile;
		rfilename.append(".root");
		TTree headers("headers", "T-MATRIX conversion input");
		TTree tree("raw","T-MATRIX raw run input");

		convHeader hdr;
		TBranch *header = headers.Branch("header", &hdr, convHeaderStr);
		{
			// Populate the header
			// Temp, nu, freq, wavelength, default par file name, shapefile name,
			// shape method, diel method, volume fraction, volume fraction method, 
			hdr.temp = temp;
			hdr.freq = *(freqs.begin());
			hdr.wvlen = *(wavelengths.begin());
			hdr.dipoleSpacing = d;
			hdr.shiv_nu = nu;
			hdr.aeff = aeff;
			hdr.vFrac = frac;
			// The rest are char arrays
			//strncpy(hdr.vFracMeth, volMeth.c_str(), cmax);;
			//strncpy(hdr.dielMeth, dielMeth.c_str(), cmax);
			//strncpy(hdr.shapeMeth, shapeMeth.c_str(), cmax);
			//strncpy(hdr.parFile, ddparFile.c_str(), cmax);
			//strncpy(hdr.shapeFile, stats->_shp->_filename.c_str(), cmax);
			headers.Fill();
		}
		TBranch *data = tree.Branch("tmatrixInVars", &in, "AXI/D:RAT/D:LAM/D:MRR/D:MRI/D:EPS/D:DDELT/D:ALPHA/D:BETA/D:THET0/D:THET/D:PHI0/D:PHI/D:NP/I:NDGS/I");

		TBranch *dataout = nullptr;
		if (tmatrix)
			dataout = tree.Branch("tmatrixOutVars",&ocnv, tmOutConverterStr);

		if (ROOToutput)
		{
			rfile = new TFile(rfilename.c_str(), "RECREATE");
		}
		
		in.AXI = aeff; // equiv vol sphere radius
		in.RAT = 1; // Indicates that AXI is the equiv volume radius, not equiv sa radius
		in.LAM = *(wavelengths.begin()); // incident light wavelength
		in.MRR = mRes.real(); // real refractive index
		in.MRI = mRes.imag(); // imag refractive index
		in.NP = -1; // shape is a spheroid
		in.EPS = asp; // ratio of horizontal to rotational axes; >1 for oblate, <1 for prolate

		// And vary ALPHA, BETA, THET0, THET, PHI0, PHI
		// ALPHA, BETA are Euler angles specifying the orientation of the scattering particle 
		// relative to the lab frame
		// Theta is the zenith angle
		// Phi is the azimuth angle
		// _0 is the incident beam, without is the scattered beam
		double thet0 = 0;
		double phi0 = 0;

		vector<tmatrixSet> jobs;
		tmatrixSet ts(in);

		for (auto p = phis.begin(); p != phis.end(); ++p)
		{
			double alpha = *p;
			for (auto t = thetas.begin(); t != thetas.end(); ++t)
			{
				double beta = *t;
				for (auto it = angles.begin(); it != angles.end(); ++it)
				{
					double phi = it->first;
					for (auto ot = it->second.begin(); ot != it->second.end(); ++ot)
					{
						double thet = *ot;

						// Create the tmatrix entry to write out!
						// Setting in two places for ROOT output
						in.ALPHA = alpha;
						in.BETA = beta;
						in.PHI = phi;
						in.PHI0 = phi0;
						in.THET = thet;
						in.THET0 = thet0;

						boost::shared_ptr<tmatrixAngleRes> ar(new tmatrixAngleRes);
						ar->alpha = alpha;
						ar->beta = beta;
						ar->phi = phi;
						ar->phi0 = phi0;
						ar->theta = thet;
						ar->theta0 = thet0;

						if (tmatrix)
						{
							tmatrix::tmatrix run;
							run.vars = in;

							run.run();
							ar->res = run.outs;

							ocnv.import(run.outs);
						}

						tree.Fill();
						ts.results.insert(ar);
					}
				}
			}
		}

		jobs.push_back(move(ts));

		rtmath::serialization::write<vector<tmatrixSet> >
			(jobs, outfile);

		if (ROOToutput)
		{
			headers.Write();
			tree.Write();
			rfile->Close();
			delete rfile;
		}
		
	}
}

