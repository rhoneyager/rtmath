#pragma once

namespace ryan_debug
{
	void appEntry();
	void appExit();
	bool pidExists(int pid);
	bool waitOnExit();
	int getPID();
	int getPPID(int pid);
}


