#pragma once

#include <boost/date_time.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <ostream>

struct observation
{
public:
	observation();
	/*
	observation(float lat, float lon,
		boost::posix_time::ptime time,
		float temp, float wbTemp, int rain_snowFlag,
		float pres, float skinTemp, float lapseRate);
	*/
	observation(const double ivals[10]);
	~observation();

	int lat;
	int lon;

	boost::posix_time::ptime time;
	int sTime;

	int temp, wbTemp;

	int rain_snowFlag;

	int pres;
	int skinTemp;
	int lapseRate;

};


//std::ostream & operator<<(std::ostream &observation, const station &ob);
