#pragma once
/* These classes exist to convert ddscat runs into tmatrix runs */

#include <map>
#include <string>
#include <set>
#include <vector>
#include <complex>
#include <boost/shared_ptr.hpp>

namespace rtmath
{
	namespace ddscat
	{
		class shapeFileStats;
		class dielTab;
		class ddPar;
	}
}

class fileconverter
{
public:
	fileconverter();
	void setStats(boost::shared_ptr<rtmath::ddscat::shapeFileStats>);
	//void setShapePattern(const std::string &);
	void setShapeMethod(const std::string &);
	void setDielMethod(const std::string &, double);
	void setVolFracMethod(const std::string &);
	void setTemp(double);
	void setFreq(double);
	void setDDPARfile(const std::string &);
	void setDielFile(const std::string &);
	void setDipoleSpacing(double);
	void setForce180(bool);
	void setFlipS(bool);
	void setMie(bool, int); // Use Mie with nth order approx (n>0)
	void convert(const std::string &) const;
private:
	void getAeff(double &frac, 
		std::set<double> &aeffs, const rtmath::ddscat::ddPar& par) const;
	void getM(std::complex<double> &mRes, double frac, 
		double freq, const boost::shared_ptr<rtmath::ddscat::dielTab> dt) const;
	void ReadDDPAR(const rtmath::ddscat::ddPar& par,
		std::set<double> &freqs, std::set<double> &wavelengths,
		std::set<double> &betas, std::set<double> &thetas, 
		std::set<double> &phis, std::map<double, std::set<double> > &angles) const;
	void getAsp(double &asp) const;
	std::string ddparFile, statsFile, dielFile;
	double temp, nu, frequency, dipoleSpacing;
	int mieOrder;
	bool force180, flipS;
	std::string shapeMeth, dielMeth, volMeth;
	boost::shared_ptr<rtmath::ddscat::shapeFileStats> stats;
};

