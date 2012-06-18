/****************************************************************************
** Meta object code from reading C++ file 'frmtarget.h'
**
** Created: Mon Jun 18 15:35:45 2012
**      by: The Qt Meta Object Compiler version 63 (Qt 4.8.0)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "StdAfx.h"
#include "../../frmtarget.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'frmtarget.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.0. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_frmTarget[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
       3,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: signature, parameters, type, tag, flags
      11,   10,   10,   10, 0x08,
      23,   10,   10,   10, 0x08,
      43,   10,   10,   10, 0x08,

       0        // eod
};

static const char qt_meta_stringdata_frmTarget[] = {
    "frmTarget\0\0processOK()\0targetTypeChanged()\0"
    "dimReInsChanged()\0"
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
        default: ;
        }
    }
    Q_UNUSED(_a);
}

const QMetaObjectExtraData frmTarget::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject frmTarget::staticMetaObject = {
    { &QMainWindow::staticMetaObject, qt_meta_stringdata_frmTarget,
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
    return QMainWindow::qt_metacast(_clname);
}

int frmTarget::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QMainWindow::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 3)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 3;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
