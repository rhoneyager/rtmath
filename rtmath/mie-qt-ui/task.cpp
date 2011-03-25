#include "StdAfx.h"
#include "task.h"


task::task(void)
{
}


task::~task(void)
{
}

bool task::canModify() {return false;}

void task::getData(taskOutput &out)
{
	out = this->_taskData;
	return;
}

