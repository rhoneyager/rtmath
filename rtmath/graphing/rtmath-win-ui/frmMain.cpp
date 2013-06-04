#include "StdAfx.h"
#include "frmMain.h"
//#include "../glgraph-win/camera.h"
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
		propPlots->SelectedObject = glgraphwinControl1->activeCamera;
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

	System::Void frmMain::baseAtmosphereToolStripMenuItem_Click(System::Object^  sender, System::EventArgs^  e)
	{
	}

	System::Void frmMain::menuStrip1_ItemClicked(System::Object^  sender, System::Windows::Forms::ToolStripItemClickedEventArgs^  e)
	{
	}

	System::Void frmMain::frmMain_Load(System::Object^  sender, System::EventArgs^  e) {
		//this->glgraphwinControl1;
		// Needed since the form doesn't fully exist at initialization
		this->glgraphwinControl1->redraw();
		this->glgraphwinControl1->render();
	}

	System::Void frmMain::frmMain_Move(System::Object^  sender, System::EventArgs^  e) {
		this->glgraphwinControl1->render();
	}

	System::Void frmMain::tabResults_Click(System::Object^  sender, System::EventArgs^  e) {
		this->glgraphwinControl1->redraw();
		this->glgraphwinControl1->render();
	}

	System::Void frmMain::propPlots_PropertyValueChanged(System::Object^  s, System::Windows::Forms::PropertyValueChangedEventArgs^  e) {
		//Refresh();
		// System::ComponentModel::PropertyTabScope::Component
		//propPlots->RefreshTabs(System::ComponentModel::PropertyTabScope::Document);
	}

}; // end rtmathwinui