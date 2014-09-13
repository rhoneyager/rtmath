#pragma once
#include <vector>
#include <string>
#include <boost/date_time.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

struct observation;


/// Reads inputs while determining max and min bounds for each field
/// Does not read zeroed inputs - these correcpond to chunk filler.
void readHDF(const std::string &filename,
	std::vector<observation> &obs,
	observation& maxObs, observation& minObs);
