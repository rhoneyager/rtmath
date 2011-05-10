#include "StdAfx.h"
#include "frmMain.h"
//#include <iostream>

namespace rtmathwinui
{
	frmMain::frmMain(void)
		{
			InitializeComponent();
			//
			//TODO: Add the constructor code here
			//
			currRun = (gcnew rtRun() );
			
		}
	System::Void frmMain::exitToolStripMenuItem_Click(System::Object^  sender, System::EventArgs^  e) {
		//using namespace std;
		// Note: std exit requires something from the standard headers to be loaded
		// So I'll use the managed equivalent
		System::Windows::Forms::Application::Exit();
		//exit(0);
		}
	System::Void frmMain::layersToolStripMenuItem1_Click(System::Object^  sender, System::EventArgs^  e) {
		// This calls for the atmospheric layer properties panel to be opened
		 }
}; // end rtmathwinui