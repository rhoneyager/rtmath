#pragma once
#include "task.h"
class taskNothing :
	public task
{
public:
	taskNothing(void);
	virtual ~taskNothing(void);
	virtual bool run(); // Run the task
	virtual bool modify(); // Open task-specific modification window
	virtual void listProps(std::string &out); // Get string for properties listing
	virtual void getData(taskOutput &out);
	virtual bool canModify();
};

