#ifndef FRMMAIN_H
#define FRMMAIN_H

#include <QtGui/QMainWindow>
#include "ui_frmmain.h"

class frmMain : public QMainWindow
{
	Q_OBJECT

public:
	frmMain(QWidget *parent = 0, Qt::WFlags flags = 0);
	~frmMain();
public slots:
	void doGenerate();
	void findDefaultPar();
	void findBaseDir();
private:
	Ui::frmMainClass ui;
};

template < class T >
T getValText(QLineEdit *src);

//template <> int getValText(QLineEdit *src);
//template <> size_t getValText(QLineEdit *src);
template <> double getValText(QLineEdit *src);
template <> std::string getValText(QLineEdit *src);

#endif // FRMMAIN_H
