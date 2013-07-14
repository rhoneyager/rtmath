#pragma once
#include <complex>
#include <vector>
#include <boost/shared_ptr.hpp>

class dataset;

class ddPoint
{
public:
	float ddQbk, ddQsca, ddQabs, ddQext;
	float ddAeff;
	float ddWave;
	float ddSizeP;
	std::complex<double> ddM;
	float volFrac;
	static std::vector<boost::shared_ptr<const ddPoint> > 
		genFromDataset(const dataset&);
};

class tmPoint
{
public:
	tmPoint();
	boost::shared_ptr<ddPoint> source;
	float tmQbk, tmQsca, tmQabs, tmQext;
	float scaleAeff, scaleF;
	float tmAeff, tmF;
	std::complex<double> tmM;

	float errQabs, errQsca, errQbk, errQext;
};

class tmRun
{
public:
	tmRun(const std::vector<boost::shared_ptr<const ddPoint> >&);
	// Scaling factors
	float scaleAeff;
	float scaleF;
	// Chi^2 error statistics
	float errQabs;
	float errQbk;
	float errQsca;
	float errQext;

	void run();

	std::vector<boost::shared_ptr<const ddPoint> > sources;
	std::vector<boost::shared_ptr<const tmPoint> > derived;
};
