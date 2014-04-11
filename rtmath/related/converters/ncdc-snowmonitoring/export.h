#pragma once
#include <map>
#include <string>
#include <boost/date_time.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

class station;

void exportToHDF(const std::string &filename, 
				 const boost::gregorian::date &start,
				 const boost::gregorian::date &end,
				 const std::map<int,station> &stations);
