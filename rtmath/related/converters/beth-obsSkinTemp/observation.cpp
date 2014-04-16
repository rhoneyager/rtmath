#include <iostream>
#include "observation.h"

observation::observation() :
lat(0), lon(0), temp(0), wbTemp(0), rain_snowFlag(0), pres(0), 
skinTemp(0), lapseRate(99.99f), sTime(0)
{}

observation::~observation() {}
observation::observation(float lat, float lon,
	boost::posix_time::ptime time,
	float temp, float wbTemp, int rain_snowFlag,
	float pres, float skinTemp, float lapseRate) :
	lat(lat), lon(lon), time(time), temp(temp), wbTemp(wbTemp),
	rain_snowFlag(rain_snowFlag), pres(pres), skinTemp(skinTemp), lapseRate(lapseRate), sTime(0)
{
	using namespace boost::posix_time;
	using namespace boost::gregorian;
	date b(1970, Jan, 1);
	sTime = static_cast<long>((time - ptime(b)).total_seconds());
}

observation::observation(const float p[10])
{
	lat = p[0];
	lon = p[1];
	{ // time
		int ip = static_cast<int>(p[2]);
		int year = ip / 10000;
		int day = ip % 100;
		int md = ip - (year * 10000);
		int month = md / 100;

		float hour = p[3];
		// Second conversion because ptime duration needs a long int.
		float sec = hour * 3600.f;

		using namespace boost::posix_time;
		using namespace boost::gregorian;
		date b(year, month, day);
		time = ptime(b, seconds(static_cast<long>(sec)));

		date a(1970, Jan, 1);
		sTime = static_cast<long>((time - ptime(a)).total_seconds());
	}
	temp = p[4];
	wbTemp = p[5];
	rain_snowFlag = (p[6] > 0) ? 1 : 0;
	pres = p[7];
	skinTemp = p[8];
	lapseRate = p[9];
}

/*
std::ostream & operator<<(std::ostream &out, const observation &ob)
{
	out << "(
	return out;
}
*/

