#pragma once
#include <map>
#include "taskOutput.h"
#include <string>

// This class is meant to be overridden, so that task-specific features may be executed
// Other classes provide the setting data - but this shouldn't be accessable from here
class task
{
public:
	task(void);
	virtual bool run() = 0; // Run the task
	virtual bool modify() = 0; // Open task-specific modification window
	virtual void listProps(std::string &out) = 0; // Get string for properties listing
	virtual ~task(void);
	virtual void getData(taskOutput &out);
	virtual bool canModify();
protected:
	taskOutput _taskData;
};

