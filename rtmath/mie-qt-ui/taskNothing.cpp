#include "StdAfx.h"
#include "taskNothing.h"

#include<string>

taskNothing::taskNothing(void)
{
}


taskNothing::~taskNothing(void)
{
}

bool taskNothing::run() {return true;}
bool taskNothing::modify() {return false;}
bool taskNothing::canModify() {return false;}

void taskNothing::listProps(std::string &out)
{
	static std::string nothingEntry = "This is a placeholder task entry. It takes no arguments and does nothing.";
	out = nothingEntry;
	return;
}

void taskNothing::getData(taskOutput &out)
{
	// Return an empty data set
	taskOutput newout;
	out = newout;
	return;
}
