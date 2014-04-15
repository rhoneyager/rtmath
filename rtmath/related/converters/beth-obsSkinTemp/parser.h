#pragma once
#include <string>
#include <map>

class station;

void parse_file(int month, int year, const std::string &filename, std::map<int,station> &stations);
