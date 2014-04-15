#include <iostream>
#include "station.h"

void station::_init()
{
	Lat = 0;
	Lon = 0;
	Elev = 0;
	COOP = 0;
	numObs = 0;
	_obs.reserve(1800);
}

station::station() { _init(); }
station::~station() {}
station::station(int COOP, float lat, float lon, float elev, 
		const std::string &stnID, const std::string &state,
		const std::string &city, const std::string &county)
{
	_init();
	this->COOP = COOP;
	Lat = lat;
	Lon = lon;
	Elev = elev;
	StnID = stnID;
	State = state;
	County = county;
	City = city;
}

bool station::operator<(const station &rhs) const
{
	if (COOP != rhs.COOP) return COOP < rhs.COOP;
	return false;
}

void station::addObs(const boost::gregorian::date &time, float val)
{
	_obs.push_back(std::pair<boost::gregorian::date, float>(time, val));
	if (!numObs)
	{
		startTime = time;
		endTime = time;
	} else {
		if (startTime > time) startTime = time;
		if (endTime < time) endTime = time;
	}
	if (val >= 0)
		numObs++;
}

std::ostream & operator<<(std::ostream &out, const station &ob)
{
	out << "COOP: " << ob.COOP << ", ID: " << ob.StnID << "\n\t"
		<< ob.City << ", " << ob.County << ", " << ob.State << "\n\t"
		<< ob.Lat << ", " << ob.Lon << ", " << ob.Elev << "\n\t"
		<< ob.startTime << "\t" << ob.endTime << "\t" << ob.numObs << std::endl;
	return out;
}

