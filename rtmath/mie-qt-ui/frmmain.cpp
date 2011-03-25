#include "stdafx.h"
#include "frmmain.h"

frmMain::frmMain(QWidget *parent, Qt::WFlags flags)
	: QMainWindow(parent, flags)
{
	ui.setupUi(this);
}

frmMain::~frmMain()
{

}

/* These are all of the slots that link to buttons and menu items */

void frmMain::jobRun() {}
void frmMain::jobItemAdd() {}
void frmMain::jobItemDelete() {}
void frmMain::jobItemModify() {}
void frmMain::jobNew()
{
	// First check if modified and ask user
	// Then, dump all stuff and start from a blank slate
}
void frmMain::jobOpen() {}
void frmMain::jobSave() {}
void frmMain::jobSaveOutput() {}
void frmMain::jobExportImage() {}
void frmMain::imageProperties() {}

