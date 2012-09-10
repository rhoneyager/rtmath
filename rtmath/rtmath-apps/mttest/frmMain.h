#pragma once
#include "ui_frmMain.h"

class frmMain
{
	public:
		frmMain();
		~frmMain();
		void exec();
		bool textChanged(const rycurses::ncHasText *);
	protected:
		void _init();
		ui_frmMain ui;
};

