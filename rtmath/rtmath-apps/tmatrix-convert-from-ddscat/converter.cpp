#include "converter.h"
#include "../../rtmath/rtmath/ddscat/shapestats.h"

fileconverter::fileconverter()
{
	temp = 0;
}

void fileconverter::setDDPARfile(const std::string &file)
{
	ddparFile = file;
}

void fileconverter::setTemp(double T)
{
	temp = T;
}

void fileconverter::setShapeMethod(const std::string &meth)
{
	shapeMeth = meth;
}

void fileconverter::setDielMethod(const std::string &meth)
{
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

void fileconverter::convert(const std::string &outfile) const
{
	// Calculate shape dimensioning (ellipsoid, sphere, ...)

	// Calculate volume fraction

	// Read ddscat.par file
	// Extract frequency, reff, rotations, scattering angles and incident vectors

	// Calculate refractive index

	// Write output
}

