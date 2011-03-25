#pragma once
#include <vector>
#include "task.h"

class job
{
public:
	job(void);
	~job(void);
	std::vector<task> tasks;
	void run();
};

