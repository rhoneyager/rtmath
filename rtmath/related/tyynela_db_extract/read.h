#pragma once
#include <string>
#include <vector>

struct data_entry {
	std::string id, source;
	float aeff, freq, md, wave, dspacing,
		qabs, qbk, qext, qsca, g, ar;
};

void readFile(const std::string &cross,
	const std::string &phase,
	const std::string &phys,
	std::vector<data_entry> &);

