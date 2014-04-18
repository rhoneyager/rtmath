#include <iostream>
#include "observation.h"

observation::observation() :
lat(0), lon(0), temp(0), wbTemp(0), rain_snowFlag(0), pres(0), 
skinTemp(0), lapseRate(9999), sTime(0)
{}

observation::~observation() {}
/*
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
	*/

observation::observation(const double p[10])
{
	lat = static_cast<int>(p[0]*100);
	lon = static_cast<int>(p[1]*100);
	{ // time
		int ip = static_cast<int>(p[2]);
		int year = ip / 10000;
		int day = ip % 100;
		int md = ip - (year * 10000);
		int month = md / 100;

		float hour = static_cast<float>(p[3]);
		// Second conversion because ptime duration needs a long int.
		float sec = hour * 3600.f;

		using namespace boost::posix_time;
		using namespace boost::gregorian;
		date b(year, month, day);
		time = ptime(b, seconds(static_cast<long>(sec)));

		date a(1970, Jan, 1);
		sTime = static_cast<int>((time - ptime(a)).total_seconds());
	}
	temp = static_cast<int>(p[4]*1000);
	wbTemp = static_cast<int>(p[5]*1000);
	rain_snowFlag = ((p[6] > 0) ? 1 : 0 );
	pres = static_cast<int>(p[7]*10);
	skinTemp = static_cast<int>(p[8]*10000);
	lapseRate = static_cast<int>(p[9]*100);
}

/*
std::ostream & operator<<(std::ostream &out, const observation &ob)
{
	out << "(
	return out;
}
*/

