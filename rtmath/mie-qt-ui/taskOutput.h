#pragma once
#include<map>
#include<vector>
#include<string>

// This defines the output structure for the task

struct taskOutput
{
public:
	// Assume zero is the indep. coord.
	std::vector<std::vector<double> > coords;
	std::vector<std::string> coordNames;
};

