#include <functional>
#include "ui_frmMain.h"
#include "frmMain.h"

ui_frmMain::~ui_frmMain()
{
}

ui_frmMain::ui_frmMain(frmMain* fm)
{
	// Construct the lambda function that triggers the validators
	//std::function<void(rycurses::ncHasText*)> fTxtEdited = 
	fTxtEdited = 
		[&](rycurses::ncHasText* t)
		{
			//return false;
			//lblX->text = "YES";
			fm->textChanged(t);
		};

	using namespace rycurses;
	wndMain = ncUi::create(34,110,0,0);
	lblTitle = ncLabel::create(wndMain,"lblTitle", " T-MATRIX / Mie Testing");
	lblTitle->resize(2,0,24,1);
	lblAXI = ncLabel::create(wndMain,"lblAXI","AXI: ");
	lblAXI->resize(1,3,8,1);
	txtAXI = ncTextBox::create(wndMain,"txtAXI","10");
	txtAXI->resize(10,3,8,1);
	txtAXI->methEdited(&fTxtEdited);
	lblRAT = ncLabel::create(wndMain,"lblRAT","RAT: ");
	lblRAT->resize(1,4,8,1);
	txtRAT = ncTextBox::create(wndMain,"txtRAT","1");
	txtRAT->resize(10,4,8,1);
	txtRAT->methEdited(&fTxtEdited);
	lblLAM = ncLabel::create(wndMain,"lblLAM","LAM: ");
	lblLAM->resize(1,5,8,1);
	txtLAM = ncTextBox::create(wndMain,"txtLAM", "6.283");
	txtLAM->resize(10,5,8,1);
	txtLAM->methEdited(&fTxtEdited);
	lblMRR = ncLabel::create(wndMain,"lblMRR","MRR: ");
	lblMRR->resize(1,6,8,1);
	txtMRR = ncTextBox::create(wndMain,"txtMRR", "1.5");
	txtMRR->resize(10,6,8,1);
	txtMRR->methEdited(&fTxtEdited);
	lblMRI = ncLabel::create(wndMain,"lblMRI","MRI: ");
	lblMRI->resize(1,7,8,1);
	txtMRI = ncTextBox::create(wndMain,"txtMRI","0.02");
	txtMRI->resize(10,7,8,1);
	txtMRI->methEdited(&fTxtEdited);
	lblEPS = ncLabel::create(wndMain,"lblEPS","EPS: ");
	lblEPS->resize(1,8,8,1);
	txtEPS = ncTextBox::create(wndMain,"txtEPS","1.0");
	txtEPS->resize(10,8,8,1);
	txtEPS->methEdited(&fTxtEdited);
	lblDDELT = ncLabel::create(wndMain,"lblDDELT", "DDELT: ");
	lblDDELT->resize(1,9,8,1);
	txtDDELT = ncTextBox::create(wndMain,"txtDDELT", "0.001");
	txtDDELT->resize(10,9,8,1);
	txtDDELT->methEdited(&fTxtEdited);
	lblALPHA = ncLabel::create(wndMain,"lblALPHA", "ALPHA: ");
	lblALPHA->resize(1,10,8,1);
	txtALPHA = ncTextBox::create(wndMain,"txtALPHA","0");
	txtALPHA->resize(10,10,8,1);
	txtALPHA->methEdited(&fTxtEdited);
	lblBETA = ncLabel::create(wndMain,"lblBETA", "BETA: ");
	lblBETA->resize(1,11,8,1);
	txtBETA = ncTextBox::create(wndMain,"txtBETA", "0");
	txtBETA->resize(10,11,8,1);
	txtBETA->methEdited(&fTxtEdited);
	lblTHET0 = ncLabel::create(wndMain,"lblTHET0", "THET0: ");
	lblTHET0->resize(1,12,8,1);
	txtTHET0 = ncTextBox::create(wndMain,"txtTHET0", "0");
	txtTHET0->resize(10,12,8,1);
	txtTHET0->methEdited(&fTxtEdited);
	lblTHET = ncLabel::create(wndMain,"lblTHET", "THET: ");
	lblTHET->resize(1,13,8,1);
	txtTHET = ncTextBox::create(wndMain,"txtTHET", "65.0");
	txtTHET->resize(10,13,8,1);
	txtTHET->methEdited(&fTxtEdited);
	lblPHI0 = ncLabel::create(wndMain,"lblPHI0", "PHI0: ");
	lblPHI0->resize(1,14,8,1);
	txtPHI0 = ncTextBox::create(wndMain,"txtPHI0", "0");
	txtPHI0->resize(10,14,8,1);
	txtPHI0->methEdited(&fTxtEdited);
	lblPHI = ncLabel::create(wndMain,"lblPHI","PHI: ");
	lblPHI->resize(1,15,8,1);
	txtPHI = ncTextBox::create(wndMain,"txtPHI", "0");
	txtPHI->resize(10,15,8,1);
	txtPHI->methEdited(&fTxtEdited);
	lblNP = ncLabel::create(wndMain,"lblNP","NP: ");
	lblNP->resize(1,16,8,1);
	txtNP = ncTextBox::create(wndMain,"txtNP", "-1");
	txtNP->resize(10,16,8,1);
	txtNP->methEdited(&fTxtEdited);
	lblNDGS = ncLabel::create(wndMain,"lblNDGS", "NDGS: ");
	lblNDGS->resize(1,17,8,1);
	txtNDGS = ncTextBox::create(wndMain,"txtNDGS","3");
	txtNDGS->resize(10,17,8,1);
	txtNDGS->methEdited(&fTxtEdited);

	lblTS = ncLabel::create(wndMain,"lblTS", "tmatrix S[2][2]");
	lblTS->resize(40,3,20,1);
	lblTSi.resize(4);
	lblTSi[0] = ncLabel::create(wndMain,"lblTS11","0");
	lblTSi[0]->resize(45,4,28,1);
	lblTSi[1] = ncLabel::create(wndMain,"lblTS12","0");
	lblTSi[1]->resize(75,4,28,1);
	lblTSi[2] = ncLabel::create(wndMain,"lblTS21","0");
	lblTSi[2]->resize(45,5,28,1);
	lblTSi[3] = ncLabel::create(wndMain,"lblTS22","0");
	lblTSi[3]->resize(75,5,28,1);

	lblTP = ncLabel::create(wndMain,"lblTP", "tmatrix P[4][4]");
	lblTP->resize(40,7,20,1);
	lblTPi.resize(16);
	lblTPi[0] = ncLabel::create(wndMain,"lblTP11","0");
	lblTPi[0]->resize(45,8,14,1);
	lblTPi[1] = ncLabel::create(wndMain,"lblTP12","0");
	lblTPi[1]->resize(60,8,14,1);
	lblTPi[2] = ncLabel::create(wndMain,"lblTP13","0");
	lblTPi[2]->resize(75,8,14,1);
	lblTPi[3] = ncLabel::create(wndMain,"lblTP14","0");
	lblTPi[3]->resize(90,8,14,1);
	lblTPi[4] = ncLabel::create(wndMain,"lblTP21","0");
	lblTPi[4]->resize(45,9,14,1);
	lblTPi[5] = ncLabel::create(wndMain,"lblTP22","0");
	lblTPi[5]->resize(60,9,14,1);
	lblTPi[6] = ncLabel::create(wndMain,"lblTP23","0");
	lblTPi[6]->resize(75,9,14,1);
	lblTPi[7] = ncLabel::create(wndMain,"lblTP24","0");
	lblTPi[7]->resize(90,9,14,1);
	lblTPi[8] = ncLabel::create(wndMain,"lblTP31","0");
	lblTPi[8]->resize(45,10,14,1);
	lblTPi[9] = ncLabel::create(wndMain,"lblTP32","0");
	lblTPi[9]->resize(60,10,14,1);
	lblTPi[10] = ncLabel::create(wndMain,"lblTP33","0");
	lblTPi[10]->resize(75,10,14,1);
	lblTPi[11] = ncLabel::create(wndMain,"lblTP34","0");
	lblTPi[11]->resize(90,10,14,1);
	lblTPi[12] = ncLabel::create(wndMain,"lblTP41","0");
	lblTPi[12]->resize(45,11,14,1);
	lblTPi[13] = ncLabel::create(wndMain,"lblTP42","0");
	lblTPi[13]->resize(60,11,14,1);
	lblTPi[14] = ncLabel::create(wndMain,"lblTP43","0");
	lblTPi[14]->resize(75,11,14,1);
	lblTPi[15] = ncLabel::create(wndMain,"lblTP44","0");
	lblTPi[15]->resize(90,11,14,1);

	lblX = ncLabel::create(wndMain,"lblX","Size Param:");
	lblX->resize(40, 13,12,1);
	lblXV = ncLabel::create(wndMain,"lblXV","0");
	lblXV->resize(55,13,10,1);

	lblMS = ncLabel::create(wndMain,"lblMS","Mie S[2][2]");
	lblMS->resize(40,15,20,1);
	lblMSi.resize(4);
	lblMSi[0] = ncLabel::create(wndMain,"lblMS11","0");
	lblMSi[0]->resize(45,16,28,1);
	lblMSi[1] = ncLabel::create(wndMain,"lblMS12","0");
	lblMSi[1]->resize(75,16,28,1);
	lblMSi[2] = ncLabel::create(wndMain,"lblMS21","0");
	lblMSi[2]->resize(45,17,28,1);
	lblMSi[3] = ncLabel::create(wndMain,"lblMS22","0");
	lblMSi[3]->resize(75,17,28,1);

	lblMP = ncLabel::create(wndMain,"lblMP","Mie P[4][4]");
	lblMP->resize(40,19,20,1);
	lblMPi.resize(16);
	lblMPi[0] = ncLabel::create(wndMain,"lblMP11","0");
	lblMPi[0]->resize(45,20,14,1);
	lblMPi[1] = ncLabel::create(wndMain,"lblMP12","0");
	lblMPi[1]->resize(60,20,14,1);
	lblMPi[2] = ncLabel::create(wndMain,"lblMP13","0");
	lblMPi[2]->resize(75,20,14,1);
	lblMPi[3] = ncLabel::create(wndMain,"lblMP14","0");
	lblMPi[3]->resize(90,20,14,1);
	lblMPi[4] = ncLabel::create(wndMain,"lblMP21","0");
	lblMPi[4]->resize(45,21,14,1);
	lblMPi[5] = ncLabel::create(wndMain,"lblMP22","0");
	lblMPi[5]->resize(60,21,14,1);
	lblMPi[6] = ncLabel::create(wndMain,"lblMP23","0");
	lblMPi[6]->resize(75,21,14,1);
	lblMPi[7] = ncLabel::create(wndMain,"lblMP24","0");
	lblMPi[7]->resize(90,21,14,1);
	lblMPi[8] = ncLabel::create(wndMain,"lblMP31","0");
	lblMPi[8]->resize(45,22,14,1);
	lblMPi[9] = ncLabel::create(wndMain,"lblMP32","0");
	lblMPi[9]->resize(60,22,14,1);
	lblMPi[10] = ncLabel::create(wndMain,"lblMP33","0");
	lblMPi[10]->resize(75,22,14,1);
	lblMPi[11] = ncLabel::create(wndMain,"lblMP34","0");
	lblMPi[11]->resize(90,22,14,1);
	lblMPi[12] = ncLabel::create(wndMain,"lblMP41","0");
	lblMPi[12]->resize(45,23,14,1);
	lblMPi[13] = ncLabel::create(wndMain,"lblMP42","0");
	lblMPi[13]->resize(60,23,14,1);
	lblMPi[14] = ncLabel::create(wndMain,"lblMP43","0");
	lblMPi[14]->resize(75,23,14,1);
	lblMPi[15] = ncLabel::create(wndMain,"lblMP44","0");
	lblMPi[15]->resize(90,23,14,1);


}

