// rtmath-win-ui.cpp : main project file.

#include "stdafx.h"
#include "frmMain.h"

using namespace rtmathwinui;

[STAThreadAttribute]
int main(array<System::String ^> ^args)
{
	// Enabling Windows XP visual effects before any controls are created
	Application::EnableVisualStyles();
	Application::SetCompatibleTextRenderingDefault(false); 

	// TODO: parse and execute and arguments
	// This will include switches and multiple files (from the command line)
	// or groups of files representing runs to load and perform
	//Application::

	// Create the main window and run it
	Application::Run(gcnew frmMain());
	return 0;
}
