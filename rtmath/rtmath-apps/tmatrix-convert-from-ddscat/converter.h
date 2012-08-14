#pragma once
/* These classes exist to convert ddscat runs into tmatrix runs */

#include <string>
#include <set>
#include <vector>
#include <boost/shared_ptr.hpp>

namespace rtmath
{
	namespace ddscat
	{
		class shapeFileStats;
	}
}

class fileconverter
{
public:
	fileconverter();
	//void setShapeFile(const std::string &);
	void setStats(boost::shared_ptr<rtmath::ddscat::shapeFileStats>);
	//void setShapePattern(const std::string &);
	void setShapeMethod(const std::string &);
	void setDielMethod(const std::string &, double);
	void setVolFracMethod(const std::string &);
	void setTemp(double);
	void setDDPARfile(const std::string &);
	void convert(const std::string &) const;
private:
	std::string ddparFile, statsFile;
	double temp, nu;
	std::string shapeMeth, dielMeth, volMeth;
	boost::shared_ptr<rtmath::ddscat::shapeFileStats> stats;
};

