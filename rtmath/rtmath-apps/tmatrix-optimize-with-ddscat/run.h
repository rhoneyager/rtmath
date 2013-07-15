#pragma once
#include <complex>
#include <vector>
#include <boost/shared_ptr.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <boost/serialization/access.hpp>

class dataset;

class ddPoint
{
	friend boost::serialization::access;
	template<class Archive>
	void serialize(Archive & ar, const unsigned int version);
public:
	double ddQbk, ddQsca, ddQabs, ddQext;
	double ddAeff;
	double ddWave;
	double ddSizeP;
	std::complex<double> ddM;
	double volFrac;
	double ddAsp;
	typedef std::vector<boost::shared_ptr<const ddPoint> > ddPointSet;
	static ddPointSet genFromDataset(const dataset&);
};

class tmPoint
{
	friend boost::serialization::access;
	template<class Archive>
	void serialize(Archive & ar, const unsigned int version);
public:
	tmPoint();
	boost::shared_ptr<const ddPoint> source;
	double tmQbk, tmQsca, tmQabs, tmQext;
	double scaleAeff, scaleF;
	double tmAeff, tmF;
	std::complex<double> tmM;

	double errQabs, errQsca, errQbk, errQext;

	typedef std::vector<boost::shared_ptr<const tmPoint> > tmPointSet;
};

class tmRun : public boost::enable_shared_from_this<tmRun>
{
	friend boost::serialization::access;
	template<class Archive>
	void serialize(Archive & ar, const unsigned int version);
public:
	static boost::shared_ptr<tmRun> genTMrun(
		const ddPoint::ddPointSet&,
		float scaleAeff,
		float scaleF);
	tmRun(const ddPoint::ddPointSet&);
	tmRun() {}
	// Scaling factors
	double scaleAeff;
	double scaleF;
	// Chi^2 error statistics
	double errQabs;
	double errQbk;
	double errQsca;
	double errQext;

	boost::shared_ptr<tmRun> run();

	ddPoint::ddPointSet sources;
	tmPoint::tmPointSet derived;
};

class tmRunSet : public boost::enable_shared_from_this<tmRunSet>
{
	friend boost::serialization::access;
	template<class Archive>
	void serialize(Archive & ar, const unsigned int version);
public:
	typedef std::vector<boost::shared_ptr<tmRun> > t_tmRuns;
	t_tmRuns tmRuns;
	ddPoint::ddPointSet ddPoints;

	boost::shared_ptr<tmRun>
		minQbk, 
		minQsca,
		minQbkQsca;
};


BOOST_CLASS_EXPORT_KEY(ddPoint);
BOOST_CLASS_EXPORT_KEY(tmPoint);
BOOST_CLASS_EXPORT_KEY(tmRun);
BOOST_CLASS_EXPORT_KEY(tmRunSet);
