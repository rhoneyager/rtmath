#pragma once
#include <string>
#include <vector>

struct observation;

void parse_file(const std::string &filename, std::vector<observation> &obs);
