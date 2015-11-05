#pragma once
#include <string>
#include <vector>

struct data_entry {
	std::string id;
	size_t version, nb, nt, np;
	float aeff, freq, md, wave, dspacing,
		qabs, qbk, qext, qsca, g, mass, ar;
};

void readFile(const std::string &in,
	std::vector<data_entry> &);

