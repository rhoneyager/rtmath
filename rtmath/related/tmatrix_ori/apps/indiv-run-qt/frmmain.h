#ifndef FRMMAIN_H
#define FRMMAIN_H

#include <QtGui/QMainWindow>
#include <QtCore/QThread>
#include <string>
#include "ui_frmmain.h"
#include "../../src/headers/tmatrix.h"

static inline std::string toUtf8(const QString& s) 
{ 
	QByteArray sUtf8 = s.toUtf8(); 
	return std::string(sUtf8.constData(), sUtf8.size()); 
}

class tmExecThread : public QThread
{
	Q_OBJECT
public:
	void initt(tmatrix::tmatrix *tm, Ui::frmmainClass *ui);
	//virtual ~tmExecThread() {}
private:
	void run();
	tmatrix::tmatrix *_tm;
	Ui::frmmainClass *_ui;
};

class frmmain : public QMainWindow
{
	Q_OBJECT

public:
	frmmain(QWidget *parent = 0, Qt::WFlags flags = 0);
	~frmmain();

private:
	Ui::frmmainClass ui;
	tmatrix::tmatrix tm;
	tmExecThread thr;
private slots:
	void populate();
	void run();
	void done();
};



#endif // FRMMAIN_H
