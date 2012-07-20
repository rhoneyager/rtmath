#ifndef FRMTARGET_H
#define FRMTARGET_H

#include <QMainWindow>
#include "ui_frmTarget.h"
#include "../../rtmath/rtmath/ddscat/shapes.h"

class frmTarget : public QDialog
{
	Q_OBJECT

public:
	frmTarget(QWidget *parent = 0, Qt::WFlags flags = 0);
	~frmTarget();
	void setTarget(boost::shared_ptr<rtmath::ddscat::shapeModifiable> tgt);
private:
	Ui::frmTarget ui;
	boost::shared_ptr<rtmath::ddscat::shapeModifiable> tgt;
	void fromShapeModifiable();
	void toShapeModifiable();
private slots:
	void processOK();
	void targetTypeChanged();
	void dimReInsChanged();
	void editTreeItem(QTreeWidgetItem*,int);
	void menuTargetProps(const QPoint &);
	void menuShapeDatProps(const QPoint &);
};

#endif // FRMTARGET_H
