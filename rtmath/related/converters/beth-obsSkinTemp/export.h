#pragma once
#include <vector>
#include <string>
#include <boost/date_time.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

struct observation;

void exportToHDF(const std::string &filename, 
				 const std::vector<observation> &obs);
