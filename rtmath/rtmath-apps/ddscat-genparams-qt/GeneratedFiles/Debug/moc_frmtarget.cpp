/****************************************************************************
** Meta object code from reading C++ file 'frmtarget.h'
**
** Created: Fri Jul 20 18:22:25 2012
**      by: The Qt Meta Object Compiler version 63 (Qt 4.8.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "StdAfx.h"
#include "../../frmtarget.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'frmtarget.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_frmTarget[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
       6,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: signature, parameters, type, tag, flags
      11,   10,   10,   10, 0x08,
      23,   10,   10,   10, 0x08,
      43,   10,   10,   10, 0x08,
      63,   61,   10,   10, 0x08,
      98,   10,   10,   10, 0x08,
     122,   10,   10,   10, 0x08,

       0        // eod
};

static const char qt_meta_stringdata_frmTarget[] = {
    "frmTarget\0\0processOK()\0targetTypeChanged()\0"
    "dimReInsChanged()\0,\0"
    "editTreeItem(QTreeWidgetItem*,int)\0"
    "menuTargetProps(QPoint)\0"
    "menuShapeDatProps(QPoint)\0"
};

void frmTarget::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        frmTarget *_t = static_cast<frmTarget *>(_o);
        switch (_id) {
        case 0: _t->processOK(); break;
        case 1: _t->targetTypeChanged(); break;
        case 2: _t->dimReInsChanged(); break;
        case 3: _t->editTreeItem((*reinterpret_cast< QTreeWidgetItem*(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2]))); break;
        case 4: _t->menuTargetProps((*reinterpret_cast< const QPoint(*)>(_a[1]))); break;
        case 5: _t->menuShapeDatProps((*reinterpret_cast< const QPoint(*)>(_a[1]))); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData frmTarget::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject frmTarget::staticMetaObject = {
    { &QDialog::staticMetaObject, qt_meta_stringdata_frmTarget,
      qt_meta_data_frmTarget, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &frmTarget::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *frmTarget::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *frmTarget::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_frmTarget))
        return static_cast<void*>(const_cast< frmTarget*>(this));
    return QDialog::qt_metacast(_clname);
}

int frmTarget::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QDialog::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 6)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 6;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
