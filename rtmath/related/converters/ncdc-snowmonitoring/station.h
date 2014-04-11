#pragma once
/// Station structure definitions

#include <boost/date_time.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <ostream>
#include <string>
#include <vector>

class station
{
public:
	station();
	station(int COOP, float lat, float lon, float elev, 
		const std::string &stnID, const std::string &state,
		const std::string &city, const std::string &county);
	~station();

	float Lat, Lon, Elev;

	int COOP;

	std::string StnID, State, City, County;

	size_t numObs;
	/// When observations at this station were first recorded
	boost::gregorian::date startTime;
	/// When observations at this station were last recorded
	boost::gregorian::date endTime;

	bool operator<(const station&) const;

	void addObs(const boost::gregorian::date &time, float val);
private:
	void _init();
	std::vector<std::pair<boost::gregorian::date, float> > _obs;
};


std::ostream & operator<<(std::ostream &stream, const station &ob);
