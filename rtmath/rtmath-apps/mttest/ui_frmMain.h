#pragma once
#include <functional>
#include <iostream>
#include <memory>
#include <vector>

#include <rycurses/ui.h>
#include <rycurses/items.h>
#include <rycurses/messagebox.h>

class frmMain;

class ui_frmMain
{
	friend class frmMain;
	ui_frmMain(frmMain*);
	~ui_frmMain();
	std::shared_ptr<rycurses::ncUi> wndMain;
	std::shared_ptr<rycurses::ncLabel> lblTitle,
		lblAXI, lblRAT, lblLAM,
		lblMRR, lblMRI, lblEPS,
		lblDDELT, lblALPHA, lblBETA,
		lblTHET0, lblTHET, lblPHI0,
		lblPHI, lblNP, lblNDGS,
		lblTS, lblTP, lblX, lblXV,
		lblMS, lblMP;
	std::vector<std::shared_ptr<rycurses::ncLabel> >
		lblTSi, lblTPi, lblMSi, lblMPi;
	std::shared_ptr<rycurses::ncTextBox> txtAXI,
		txtRAT, txtLAM, txtMRR, txtMRI,
		txtEPS, txtDDELT, txtALPHA,
		txtBETA, txtTHET0, txtTHET, 
		txtPHI0, txtPHI, txtNP, txtNDGS;
	std::function<void(rycurses::ncHasText*)> fTxtEdited;
};

